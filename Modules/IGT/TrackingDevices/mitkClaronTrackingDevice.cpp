/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "mitkClaronTrackingDevice.h"
#include "mitkClaronTool.h"
#include "mitkIGTConfig.h"
#include "mitkIGTTimeStamp.h"
#include "mitkIGTHardwareException.h"
#include <itksys/SystemTools.hxx>
#include <iostream>
#include <itkMutexLockHolder.h>
#include <mitkIOUtil.h>
#include <mitkMicronTrackerTypeInformation.h>
#include <chrono>
#include <thread>

typedef itk::MutexLockHolder<itk::FastMutexLock> MutexLockHolder;


mitk::ClaronTrackingDevice::ClaronTrackingDevice(): mitk::TrackingDevice()
{
  //set the type of this tracking device
  this->m_Data = mitk::MicronTrackerTypeInformation::GetDeviceDataMicronTrackerH40();

  this->m_MultiThreader = itk::MultiThreader::New();
  m_ThreadID = 0;

  m_Device = mitk::ClaronInterface::New();
  //############################# standard directories ##################################
  if (m_Device->IsMicronTrackerInstalled())
  {
    m_ToolfilesDir = mitk::IOUtil::CreateTemporaryDirectory();
#ifdef MITK_MICRON_TRACKER_CALIBRATION_DIR
    m_CalibrationDir = std::string(MITK_MICRON_TRACKER_CALIBRATION_DIR);
#endif
  }
  else
  {
    m_ToolfilesDir = "Error - No Microntracker installed";
    m_CalibrationDir = "Error - No Microntracker installed";
  }
  //##################################################################################################
  m_Device->Initialize(m_CalibrationDir, m_ToolfilesDir);

  fnSetDesiredFps(20);
}

bool mitk::ClaronTrackingDevice::IsDeviceInstalled()
{
  mitk::ClaronInterface::Pointer tempInterface = mitk::ClaronInterface::New();
  return tempInterface->IsMicronTrackerInstalled();
}


mitk::ClaronTrackingDevice::~ClaronTrackingDevice()
{
}


mitk::TrackingTool* mitk::ClaronTrackingDevice::AddTool( const char* toolName, const char* fileName )
{
  mitk::ClaronTool::Pointer t = mitk::ClaronTool::New();
  if (t->LoadFile(fileName) == false)
  {
    return nullptr;
  }
  t->SetToolName(toolName);
  if (this->InternalAddTool(t) == false)
    return nullptr;
  return t.GetPointer();
}


bool mitk::ClaronTrackingDevice::InternalAddTool(ClaronTool::Pointer tool)
{
  m_AllTools.push_back(tool);
  return true;
}


std::vector<mitk::ClaronTool::Pointer> mitk::ClaronTrackingDevice::DetectTools()
{
  std::vector<mitk::ClaronTool::Pointer> returnValue;
  std::vector<claronToolHandle> allHandles = m_Device->GetAllActiveTools();
  for (auto iter = allHandles.begin(); iter != allHandles.end(); ++iter)
  {
    ClaronTool::Pointer newTool = ClaronTool::New();
    newTool->SetToolName(m_Device->GetName(*iter));
    newTool->SetCalibrationName(m_Device->GetName(*iter));
    newTool->SetToolHandle(*iter);
    returnValue.push_back(newTool);
  }
  return returnValue;
}


bool mitk::ClaronTrackingDevice::StartTracking()
{

  //By Alfred: next line because no temp directory is set if MicronTracker is not installed
  if (!m_Device->IsMicronTrackerInstalled())
    return false;
  //##################################################################################

  //be sure that the temp-directory is empty at start: delete all files in the tool files directory
  itksys::SystemTools::RemoveADirectory(m_ToolfilesDir.c_str());
  itksys::SystemTools::MakeDirectory(m_ToolfilesDir.c_str());

  //copy all toolfiles into the temp directory
  for (unsigned int i=0; i<m_AllTools.size(); i++)
  {
    itksys::SystemTools::CopyAFile(m_AllTools[i]->GetFile().c_str(), m_ToolfilesDir.c_str());
  }
  this->SetState(Tracking);            // go to mode Tracking
  this->m_StopTrackingMutex->Lock();  // update the local copy of m_StopTracking
  this->m_StopTracking = false;
  this->m_StopTrackingMutex->Unlock();

  //restart the Microntracker, so it will load the new tool files
  m_Device->StopTracking();
  m_Device->Initialize(m_CalibrationDir,m_ToolfilesDir);

  // HRS_NAVIGATION_MODIFICATION starts
  m_TrackingFinishedMutex->Unlock(); // transfer the execution rights to tracking thread
  // HRS_NAVIGATION_MODIFICATION ends

  if (m_Device->StartTracking())
  {
    mitk::IGTTimeStamp::GetInstance()->Start(this);
    m_ThreadID = m_MultiThreader->SpawnThread(this->ThreadStartTracking, this);    // start a new thread that executes the TrackTools() method
    return true;
  }
  else
    {mitkThrowException(mitk::IGTHardwareException) << "Error while trying to start the device!";}
}


bool mitk::ClaronTrackingDevice::StopTracking()
{
  Superclass::StopTracking();
  //delete all files in the tool files directory
  itksys::SystemTools::RemoveADirectory(m_ToolfilesDir.c_str());
  return true;
}


unsigned int mitk::ClaronTrackingDevice::GetToolCount() const
{
  return (unsigned int)this->m_AllTools.size();
}


mitk::TrackingTool* mitk::ClaronTrackingDevice::GetTool(unsigned int toolNumber) const
{
  if ( toolNumber >= this->GetToolCount())
    return nullptr;
  else
    return this->m_AllTools[toolNumber];
}


bool mitk::ClaronTrackingDevice::OpenConnection()
{
  bool returnValue;
  //Create the temp directory
  itksys::SystemTools::MakeDirectory(m_ToolfilesDir.c_str());

  m_Device->Initialize(m_CalibrationDir,m_ToolfilesDir);
  returnValue = m_Device->StartTracking();

  if (returnValue)
  {
    this->SetState(Ready);
  }
  else
  {
    //reset everything
    if (m_Device.IsNull())
    {
      m_Device = mitk::ClaronInterface::New();
      m_Device->Initialize(m_CalibrationDir, m_ToolfilesDir);
    }
    m_Device->StopTracking();
    this->SetState(Setup);
    mitkThrowException(mitk::IGTHardwareException) << "Error while trying to open connection to the MicronTracker.";
  }
  return returnValue;
}


bool mitk::ClaronTrackingDevice::CloseConnection()
{
  bool returnValue = true;
  if (this->GetState() == Setup)
    return true;

  returnValue = m_Device->StopTracking();

  //delete the temporary directory
  itksys::SystemTools::RemoveADirectory(m_ToolfilesDir.c_str());

  this->SetState(Setup);
  return returnValue;
}


mitk::ClaronInterface* mitk::ClaronTrackingDevice::GetDevice()
{
  return m_Device;
}


std::vector<mitk::ClaronTool::Pointer> mitk::ClaronTrackingDevice::GetAllTools()
{
  return this->m_AllTools;
}


void mitk::ClaronTrackingDevice::TrackTools()
{
  try
  {
    /* lock the TrackingFinishedMutex to signal that the execution rights are now transfered to the tracking thread */
    MutexLockHolder trackingFinishedLockHolder(*m_TrackingFinishedMutex); // keep lock until end of scope

    bool localStopTracking;       // Because m_StopTracking is used by two threads, access has to be guarded by a mutex. To minimize thread locking, a local copy is used here
    this->m_StopTrackingMutex->Lock();  // update the local copy of m_StopTracking
    localStopTracking = this->m_StopTracking;
    this->m_StopTrackingMutex->Unlock();

    while ((this->GetState() == Tracking) && (localStopTracking == false))
    {
      std::chrono::high_resolution_clock::time_point tStartTime = std::chrono::high_resolution_clock::now();
      this->GetDevice()->GrabFrame();

      std::vector<mitk::ClaronTool::Pointer> detectedTools = this->DetectTools();
      std::vector<mitk::ClaronTool::Pointer> allTools = this->GetAllTools();
      std::vector<mitk::ClaronTool::Pointer>::iterator itAllTools;
      for(itAllTools = allTools.begin(); itAllTools != allTools.end(); itAllTools++)
      {
        mitk::ClaronTool::Pointer currentTool = *itAllTools;
        //test if current tool was detected
        std::vector<mitk::ClaronTool::Pointer>::iterator itDetectedTools;
        bool foundTool = false;
        for(itDetectedTools = detectedTools.begin(); itDetectedTools != detectedTools.end(); itDetectedTools++)
        {
          mitk::ClaronTool::Pointer aktuDet = *itDetectedTools;
          std::string tempString(currentTool->GetCalibrationName());
          if (tempString.compare(aktuDet->GetCalibrationName())==0)
          {
            currentTool->SetToolHandle(aktuDet->GetToolHandle());
            foundTool = true;
          }
        }
        if (!foundTool)
        {
          currentTool->SetToolHandle(0);
        }

        if (currentTool->GetToolHandle() != 0)
        {
          // HRS_NAVIGATION_MODIFICATION starts
          // currentTool->SetDataValid(true);
          // HRS_NAVIGATION_MODIFICATION ends
          // get tip position of tool:
          std::vector<double> pos_vector = this->GetDevice()->GetTipPosition(currentTool->GetToolHandle());

          // HRS_NAVIGATION_MODIFICATION starts
          currentTool->SetToolThermalHazard(this->GetDevice()->GetToolThermalHazard());
          // HRS_NAVIGATION_MODIFICATION ends

          // write tip position into tool:
          mitk::Point3D pos;
          pos[0] = pos_vector[0];
          pos[1] = pos_vector[1];
          pos[2] = pos_vector[2];
          currentTool->SetPosition(pos);
          // get tip quaternion of tool
          std::vector<double> quat = this->GetDevice()->GetTipQuaternions(currentTool->GetToolHandle());

          // HRS_NAVIGATION_MODIFICATION starts
          currentTool->SetDataValid(this->GetDevice()->IsLastTrackerIdentifiedInRefSpace() ||
                                    this->GetDevice()->IsLastTrackerReferenceNotSet());
          // HRS_NAVIGATION_MODIFICATION ends

          // write tip quaternion into tool
          mitk::Quaternion orientation(quat[1], quat[2], quat[3], quat[0]);
          currentTool->SetOrientation(orientation);

          // TODO: read the timestamp data from the tracking device interface
          currentTool->SetIGTTimeStamp(mitk::IGTTimeStamp::GetInstance()->GetElapsed());
        }
        else
        {
          mitk::Point3D origin;
          origin.Fill(0);
          currentTool->SetPosition(origin);
          currentTool->SetOrientation(mitk::Quaternion(0,0,0,0));
          currentTool->SetDataValid(false);
        }
      }

      // Added by AmitRungta on 26-11-2021  now lets wait till our desired FPS if required.
      const auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tStartTime).count();
      if (timeDiff < m_iDesiredDelayInMsForFps)
        std::this_thread::sleep_for(std::chrono::milliseconds(m_iDesiredDelayInMsForFps - timeDiff));

      /* Update the local copy of m_StopTracking */
      this->m_StopTrackingMutex->Lock();
      localStopTracking = m_StopTracking;
      this->m_StopTrackingMutex->Unlock();
    }
  }
  catch(...)
  {
    this->StopTracking();
    // HRS_NAVIGATION_MODIFICATION starts
    MITK_ERROR << "Error while trying to track tools. Thread stopped.";
    // Disabled exception throwing as it cause application crash. As it is from a separate thread
    // Unwanted Stopped tracking indicates error to the client logic.
    // mitkThrowException(mitk::IGTHardwareException) << "Error while trying to track tools. Thread stopped.";
    // HRS_NAVIGATION_MODIFICATION ends
  }
}


bool mitk::ClaronTrackingDevice::IsMicronTrackerInstalled()
{
  return this->m_Device->IsMicronTrackerInstalled();
}


ITK_THREAD_RETURN_TYPE mitk::ClaronTrackingDevice::ThreadStartTracking(void* pInfoStruct)
{
  /* extract this pointer from Thread Info structure */
  struct itk::MultiThreader::ThreadInfoStruct * pInfo = (struct itk::MultiThreader::ThreadInfoStruct*)pInfoStruct;
  if (pInfo == nullptr)
  {
    return ITK_THREAD_RETURN_VALUE;
  }
  if (pInfo->UserData == nullptr)
  {
    return ITK_THREAD_RETURN_VALUE;
  }
  ClaronTrackingDevice *trackingDevice = (ClaronTrackingDevice*)pInfo->UserData;

  if (trackingDevice != nullptr)
    trackingDevice->TrackTools();

  return ITK_THREAD_RETURN_VALUE;
}


// HRS_NAVIGATION_MODIFICATION starts
void mitk::ClaronTrackingDevice::SetTrackerlessNavEnabledTracker(mtHandle trackerHandle, bool enable)
{
  this->GetDevice()->SetTrackerlessNavEnabledTracker(trackerHandle, enable);
}


// Added by AmitRungta on 25-11-2021
// This function will set if we want to process in background or normal after tracking starts.
void mitk::ClaronTrackingDevice::fnSetProcessingInBackgroundThread(bool baProcessInBackgroundThread) {
  this->GetDevice()->fnSetProcessingInBackgroundThread(baProcessInBackgroundThread);
}


// This function will get the current state of the tracking and not the one actual desired. As there can be a case
// that we may have desired to change the state but its already tracking with an older state.
bool mitk::ClaronTrackingDevice::fnGetProcessingInBackgroundThread() {
  return this->GetDevice()->fnGetProcessingInBackgroundThread();
}

// Get and set the desired FPS.
void mitk::ClaronTrackingDevice::fnSetDesiredFps(int iaFps)
{
  m_iDesiredFps = iaFps;
  if (m_iDesiredFps < 1 || m_iDesiredFps > 20)
    m_iDesiredFps = 20;

  m_iDesiredDelayInMsForFps = 1000 / m_iDesiredFps;
}



// HRS_NAVIGATION_MODIFICATION ends
