/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

// HRS_NAVIGATION_MODIFICATION starts

#include <MTC.h>
#include <math.h>
#include <mitkClaronInterface.h>
#include <mitkIGTHardwareException.h>
#include <mitkNumericTypes.h>
#include <string>
#include <chrono>
#include <thread>

const double __OneDivideByRootTwo = (1.0 / (sqrt(2.0)));

mitk::ClaronInterface::ClaronInterface()
{
  m_ClaronToolNameCache.resize(MT_MAX_STRING_LENGTH);
  isTracking = false;
  sprintf(calibrationDir, "No calibration dir set yet");
  sprintf(markerDir, "No marker dir set yet");
  m_LastTrackerIdentifiedInRefSpace = false;
  m_ReferenceNotSet = true;

  // Added by AmitRungta on 26-11-2021 
  m_bCurrentProcessingInBackgroundThread = true ;
  m_bDesiredProcessingInBackgroundThread = true ;
}

mitk::ClaronInterface::~ClaronInterface() {}

void mitk::ClaronInterface::Initialize(std::string calibrationDir, std::string toolFilesDir)
{
  sprintf(this->calibrationDir, calibrationDir.c_str());
  sprintf(this->markerDir, toolFilesDir.c_str());
  this->IdentifiedMarkers = 0;
  this->PoseXf = 0;
  this->CurrCamera = 0;
  this->IdentifyingCamera = 0;
  this->ToolThermalHazard = 0;
  m_HandleForTrackerless.clear();
  m_TrackerlessHandleLastValidData.clear();
}

bool mitk::ClaronInterface::StartTracking()
{
  isTracking = false;
  try
  {
    MTC(Cameras_AttachAvailableCameras(calibrationDir)); // Connect to camera
    if (Cameras_Count() < 1)
    {
      MITK_ERROR << "No camera found!";
      return false;
    }
    if (Cameras_Count() > 1)
    {
      MITK_ERROR << "More then one Camera found";
      return false;
    }

    // Step 1: initialize cameras
    MTC(Cameras_ItemGet(0, &CurrCamera)); // Obtain a handle to the first/only camera in the array

    // Step 2: Load the marker templates
    MTC(Markers_LoadTemplates(markerDir)); // Path to directory where the marker templates are
    MITK_INFO << "Loaded marker templates\n" << Markers_TemplatesCount();

    if (m_bDesiredProcessingInBackgroundThread)
    {
      MTC(Markers_BackGroundProcessSet(true));
      MTC(XPoints_BackGroundProcessSet(true));
    }
    m_bCurrentProcessingInBackgroundThread = m_bDesiredProcessingInBackgroundThread;

	Markers_SmallerXPFootprintSet(false); // by default it was on.

    m_tLastValidGrabTime = std::chrono::high_resolution_clock::now();
    Camera_FramesGrabbedGet(CurrCamera, &m_iLastValidFrameGrab);

    // Step 3: Wait for 20 frames
    for (int i = 0; i < 20; i++) // the first 20 frames are auto-adjustment frames, we ignore them
    {
      Sleep(50); // Let it sleep for some time before getting new data.
      if (fnGetProcessingInBackgroundThread())
      {
        MTC(Markers_GetIdentifiedMarkersFromBackgroundThread(CurrCamera));
      }
      else
      {
        GrabFrame();
      }
    }

	// We are changing it to false,
    // As it is auto equalizing image so image will look almost similar eveytime irrespective of light,
    // So we are unable to see where light is focusing and where it's dark
    // MTC(Cameras_HistogramEqualizeImagesSet(true)); // set the histogram equalizing

    // Step 4: Initialize IdentifiedMarkers and PoseXf
    IdentifiedMarkers = Collection_New();
    PoseXf = Xform3D_New();

    // now we are tracking...

    /* MTHome is not in use. The following code has to be activated if you want to use MTHome!
    //Initialize MTHome
    if ( getMTHome (MTHome, sizeof(MTHome)) < 0 )
    {
    // No Environment
    printf("MTHome environment variable is not set!\n");
    }*/
  }
  catch (...)
  {
    Cameras_Detach();
    MITK_INFO << " Error while connecting MicronTracker. " << MTLastErrorString();
    return false;
  }

  isTracking = true;
  return true;
}

bool mitk::ClaronInterface::StopTracking()
{
  if (isTracking)
  {
    // free up the resources
    Collection_Free(IdentifiedMarkers);
    Xform3D_Free(PoseXf);

    try
    {
      // stop the camera
      Cameras_Detach();
    }
    catch (...)
    {
      MITK_ERROR << "error in detaching camera. " << MTLastErrorString();
      isTracking = false;
      return false;
    }

    // now tracking is stopped
    isTracking = false;
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<mitk::claronToolHandle> mitk::ClaronInterface::GetAllActiveTools()
{
  // Set returnvalue
  std::vector<claronToolHandle> returnValue;

  // Here, MTC internally maintains the measurement results.
  // Those results can be accessed until the next call to Markers_ProcessFrame, when they
  // are updated to reflect the next frame's content.
  // First, we will obtain the collection of the markers that were identified.
  // bool result = false;
  // MTC ( Markers_BackGroundProcessGet(&result));
  // if (result == false)
  //{
  // // If camera disconnects then from background processing APIs we don't get any error / indication.
  // // but it sets the background process flag to 0;
  //
  // int result = Camera_GrabFrame(CurrCamera);
  // if( result != mtOK ) //Camera(s)_GrabFrame firsts gives frame grab error & then crashes when camera is powered off
  // while running
  // {
  //  MTC(Markers_AutoAdjustShortCycleHdrExposureLockedMarkersSet(false)); // To fix a bug in MTC library. If HDR is on
  //  & camera gets disconnected then when camera is reconnected
  //  // driver expect HDR to on but in initialization HDR is off. Which results into frames freeze
  //  MITK_INFO << "Camera Connection Problem " << MTLastErrorString();
  //  mitkThrowException(mitk::IGTHardwareException) << "Camera Connection Problem."; // It will be handled at later
  //  stage
  // }
  //}

  if (fnGetProcessingInBackgroundThread())
  {
    if (Markers_GetIdentifiedMarkersFromBackgroundThread(CurrCamera) != mtOK)
    {
      MTC(Markers_AutoAdjustShortCycleHdrExposureLockedMarkersSet(
        false)); // To fix a bug in MTC library. If HDR is on & camera gets disconnected then when camera is reconnected
      // driver expect HDR to on but in initialization HDR is off. Which results into frames freeze
      MITK_INFO << "Camera Connection Problem " << MTLastErrorString();
		mitkThrowException(mitk::IGTHardwareException) << "Camera Connection Problem."; // It will be handled at later stage
    };

    // Markers_GetIdentifiedMarkersFromBackgroundThread() locks the pose data buffer until a new pose data becomes
    // available therefore calling Markers_IdentifiedMarkersGet() might take a few milliseconds until the data becomes
    // ready. If you don't want to wait for the pose, Use Markers_IsBackgroundFrameProcessedGet() before calling
    // Markers_IdentifiedMarkersGet() to find out if data is ready. Sometimes Markers_IdentifiedMarkersGet throws
    // exception when data is not available. to avoid that following loop is being used.
    bool backgroundFrameProcessed = false;
    for (int i = 0; i < 100; i++)
    {
      MTC(Markers_IsBackgroundFrameProcessedGet(&backgroundFrameProcessed));
      if (backgroundFrameProcessed == true)
        break;
      else
        Sleep(5);
    }

    if (!backgroundFrameProcessed)
      return returnValue;
  }

  if (Markers_IdentifiedMarkersGet(CurrCamera, IdentifiedMarkers) != mtOK)
  {
    MTC(Markers_AutoAdjustShortCycleHdrExposureLockedMarkersSet(
      false)); // To fix a bug in MTC library. If HDR is on & camera gets disconnected then when camera is reconnected
    // driver expect HDR to on but in initialization HDR is off. Which results into frames freeze
    MITK_INFO << "Marker Identification Problem " << MTLastErrorString();
    mitkThrowException(mitk::IGTHardwareException) << "Camera Connection Problem."; // It will be handled at later stage
  }


  // Now we iterate on the identified markers and add them to the returnvalue
  for (int j = 1; j <= Collection_Count(IdentifiedMarkers); j++)
  {
    // Obtain the marker's handle, and use it to obtain the pose in the current camera's space
    // using our Xform3D object, PoseXf.
    mtHandle Marker = Collection_Int(IdentifiedMarkers, j);
    returnValue.push_back(Marker);
  }
  return returnValue;
}

void mitk::ClaronInterface::GrabFrame()
{
  // Modified by AmitRungta on 21-06-2023 as we want to get the error signal for frame capture error quickly.
  //const int niMaxTimeForCameraErrorInMs = 15000;// if wait is more than 15 sec then mark as camera disconnected...
  const int niMaxTimeForCameraErrorInMs = 500; // if wait is more than 500 millisec then mark as camera disconnected...
  std::chrono::high_resolution_clock::time_point tCurTime = std::chrono::high_resolution_clock::now();

  if (!fnGetProcessingInBackgroundThread())
  {
    static std::chrono::high_resolution_clock::time_point tLastTime = tCurTime;
    
    const auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(tCurTime - tLastTime).count();
    if (timeDiff < m_iDesiredDelayInMsForFps)
      std::this_thread::sleep_for(std::chrono::milliseconds(m_iDesiredDelayInMsForFps - timeDiff));

    const auto nuiRetVal = Camera_GrabFrame(CurrCamera);
    std::string clLastError = MTLastErrorString();

    if (nuiRetVal != mtOK)
    {
      const auto LastValidtimeDiff =
        std::chrono::duration_cast<std::chrono::milliseconds>(tCurTime - m_tLastValidGrabTime).count();
      if (LastValidtimeDiff > niMaxTimeForCameraErrorInMs) // if wait is more than 15 sec then mark as camera disconnected...
      {
        mitkThrowException(mitk::IGTHardwareException)
          << "Camera Connection Problem." << clLastError.c_str(); // It will be handled at later stage
      }
    }
    else
    {
      m_tLastValidGrabTime = std::chrono::high_resolution_clock::now();
    }

    MTC(Markers_ProcessFrame(CurrCamera)); // Process the frame(s)

    // Added by AmitRungta on 26-11-2021 for processing frames.
    MTC(XPoints_ProcessFrame(CurrCamera)); // Process the frame(s)

    tLastTime = std::chrono::high_resolution_clock::now();
  }
  else
  {
    // Added by AmitRungta on 01-12-2021 
    // as now we want to break even in case of background thread processing if the images are not changed for a long time.
    int iCurrentFrameGrab = 0; 
    Camera_FramesGrabbedGet(CurrCamera, &iCurrentFrameGrab);
    if (iCurrentFrameGrab > m_iLastValidFrameGrab)
    {
      m_iLastValidFrameGrab = iCurrentFrameGrab;
      m_tLastValidGrabTime = std::chrono::high_resolution_clock::now();
    }
    else
    {
      const auto LastValidtimeDiff =
        std::chrono::duration_cast<std::chrono::milliseconds>(tCurTime - m_tLastValidGrabTime).count();
      if (LastValidtimeDiff >
          niMaxTimeForCameraErrorInMs) // if wait is more than 15 sec then mark as camera disconnected...
      {
        mitkThrowException(mitk::IGTHardwareException)
          << "Camera Connection Problem." << "Unable to grab new frames..."; // It will be handled at later stage
      }
    }
  }
}

std::vector<double> mitk::ClaronInterface::GetTipPosition(mitk::claronToolHandle c)
{
  std::vector<double> returnValue;
  double Position[3];
  mtHandle t2m = Xform3D_New(); // tooltip to marker xform handle
  mtHandle t2c = Xform3D_New(); // tooltip to camera xform handle
  mtHandle m2c = Xform3D_New(); // marker to camera xform handle

  mtHandle refHandle;
  MTC(Marker_ReferenceMarkerHandleGet(c, &refHandle));

  m_ReferenceNotSet = (0 == refHandle || refHandle == CurrCamera);

  if (!m_ReferenceNotSet)
  {
    MTC(Marker_Tooltip2ReferenceXfGet(c, CurrCamera, t2c, &IdentifyingCamera));
    MTC(Marker_MarkerWasIdentifiedInRefSpaceGet(c, CurrCamera, &m_LastTrackerIdentifiedInRefSpace));

    if (!m_LastTrackerIdentifiedInRefSpace && (m_HandleForTrackerless.find(refHandle) != m_HandleForTrackerless.end()))
    {
      // Get m2c
      MTC(Marker_Marker2CameraXfGet(c, CurrCamera, m2c, &IdentifyingCamera));
      // Get t2m
      MTC(Marker_Tooltip2MarkerXfGet(c, t2m));
      // Transform both to t2c
      MTC(Xform3D_Concatenate(t2m, m2c, t2c));

      auto itr = m_TrackerlessHandleLastValidData.find(refHandle);
      if (itr != m_TrackerlessHandleLastValidData.end())
      {
        double positionToSet[3];
        positionToSet[0] = itr->second.first[0];
        positionToSet[1] = itr->second.first[1];
        positionToSet[2] = itr->second.first[2];

        double quarternionsToSet[4];
        quarternionsToSet[0] = itr->second.second[0];
        quarternionsToSet[1] = itr->second.second[1];
        quarternionsToSet[2] = itr->second.second[2];
        quarternionsToSet[3] = itr->second.second[3];

        // reference handle is enabled for trackerless navigation...
        // so will compute its data as per last valid data...
        mtHandle oldRefDataM2CHandle = Xform3D_New();
        Xform3D_ShiftSet(oldRefDataM2CHandle, positionToSet);
        Xform3D_RotQuaternionsSet(oldRefDataM2CHandle, quarternionsToSet);

        mtHandle oldRefDataC2MHandle = Xform3D_New();
        Xform3D_Inverse(oldRefDataM2CHandle, oldRefDataC2MHandle);
        Xform3D_Free(oldRefDataM2CHandle);

        mtHandle finalHandle = Xform3D_New();
        Xform3D_Concatenate(t2c, oldRefDataC2MHandle, finalHandle);
        Xform3D_Free(t2c);
        t2c = finalHandle;

        m_LastTrackerIdentifiedInRefSpace = true;

        Xform3D_Free(oldRefDataC2MHandle);
      }
    }
  }
  else
  {
    // Get m2c
    MTC(Marker_Marker2CameraXfGet(c, CurrCamera, m2c, &IdentifyingCamera));
    // Get t2m
    MTC(Marker_Tooltip2MarkerXfGet(c, t2m));
    // Transform both to t2c
    MTC(Xform3D_Concatenate(t2m, m2c, t2c));
  }

  // Get position
  MTC(Xform3D_ShiftGet(t2c, Position));

  // If this tracker is enable for trackerless than we need to keep its last valid value...
  if (m_HandleForTrackerless.find(c) != m_HandleForTrackerless.end())
  {
    auto &valRef = m_TrackerlessHandleLastValidData[c];
    valRef.first.clear();
    valRef.first.push_back(Position[0]);
    valRef.first.push_back(Position[1]);
    valRef.first.push_back(Position[2]);
  }

  // We will also check and report any measurement hazard
  mtMeasurementHazardCode Hazard;
  MTC(Xform3D_HazardCodeGet(t2c, &Hazard));
  SetToolThermalHazard(Hazard);
  // Here we have to negate the X- and Y-coordinates because of a bug of the
  // MTC-library.
  if (m_LastTrackerIdentifiedInRefSpace && !m_ReferenceNotSet)
  {
    returnValue.push_back(-Position[2]);
    returnValue.push_back(Position[1]);
    returnValue.push_back(Position[0]);
  }
  else
  {
    returnValue.push_back(-Position[0]);
    returnValue.push_back(-Position[1]);
    returnValue.push_back(Position[2]);
  }

  return returnValue;
}

std::vector<double> mitk::ClaronInterface::GetPosition(claronToolHandle c)
{
  std::vector<double> returnValue;
  double Position[3];

  mtHandle refHandle;
  MTC(Marker_ReferenceMarkerHandleGet(c, &refHandle));

  m_ReferenceNotSet = (0 == refHandle || refHandle == CurrCamera);

  if (!m_ReferenceNotSet)
  {
    MTC(Marker_Marker2ReferenceXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera));
    MTC(Marker_MarkerWasIdentifiedInRefSpaceGet(c, CurrCamera, &m_LastTrackerIdentifiedInRefSpace));

    if (!m_LastTrackerIdentifiedInRefSpace && (m_HandleForTrackerless.find(refHandle) != m_HandleForTrackerless.end()))
    {
      MTC(Marker_Marker2CameraXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera));

      auto itr = m_TrackerlessHandleLastValidData.find(refHandle);
      if (itr != m_TrackerlessHandleLastValidData.end())
      {
        double positionToSet[3];
        positionToSet[0] = itr->second.first[0];
        positionToSet[1] = itr->second.first[1];
        positionToSet[2] = itr->second.first[2];

        double quarternionsToSet[4];
        quarternionsToSet[0] = itr->second.second[0];
        quarternionsToSet[1] = itr->second.second[1];
        quarternionsToSet[2] = itr->second.second[2];
        quarternionsToSet[3] = itr->second.second[3];

        // reference handle is enabled for trackerless navigation...
        // so will compute its data as per last valid data...
        mtHandle oldRefDataM2CHandle = Xform3D_New();
        Xform3D_ShiftSet(oldRefDataM2CHandle, positionToSet);
        Xform3D_RotQuaternionsSet(oldRefDataM2CHandle, quarternionsToSet);

        mtHandle oldRefDataC2MHandle = Xform3D_New();
        Xform3D_Inverse(oldRefDataM2CHandle, oldRefDataC2MHandle);
        Xform3D_Free(oldRefDataM2CHandle);

        mtHandle finalHandle = Xform3D_New();
        Xform3D_Concatenate(PoseXf, oldRefDataC2MHandle, finalHandle);
        Xform3D_Free(PoseXf);
        PoseXf = finalHandle;

        m_LastTrackerIdentifiedInRefSpace = true;

        Xform3D_Free(oldRefDataC2MHandle);
      }
    }
  }
  else
    MTC(Marker_Marker2CameraXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera));

  MTC(Xform3D_ShiftGet(PoseXf, Position));

  // Here we have to negate the X- and Y-coordinates because of a bug of the
  // MTC-library.
  if (m_LastTrackerIdentifiedInRefSpace && !m_ReferenceNotSet)
  {
    returnValue.push_back(-Position[2]);
    returnValue.push_back(Position[1]);
    returnValue.push_back(Position[0]);
  }
  else
  {
    returnValue.push_back(-Position[0]);
    returnValue.push_back(-Position[1]);
    returnValue.push_back(Position[2]);
  }

  return returnValue;
}

std::vector<double> mitk::ClaronInterface::GetTipQuaternions(claronToolHandle c)
{
  std::vector<double> returnValue;

  mtHandle t2m = Xform3D_New(); // tooltip to marker xform handle
  mtHandle t2c = Xform3D_New(); // tooltip to camera xform handle
  mtHandle m2c = Xform3D_New(); // marker to camera xform handle

  mtHandle IdentifyingCamera;

  mtHandle refHandle;
  MTC(Marker_ReferenceMarkerHandleGet(c, &refHandle));

  m_ReferenceNotSet = (0 == refHandle || refHandle == CurrCamera);

  // get the Claron-Quaternion
  double Quarternions[4];

  if (!m_ReferenceNotSet)
  {
    /*MTC(Marker_Tooltip2ReferenceXfGet(c, CurrCamera, t2c, &IdentifyingCamera));
    MTC(Marker_MarkerWasIdentifiedInRefSpaceGet(c, CurrCamera, &m_LastTrackerIdentifiedInRefSpace));*/

    MTC(Marker_Marker2CameraXfGet(c, CurrCamera, m2c, &IdentifyingCamera)); // Get m2c
    MTC(Marker_Tooltip2MarkerXfGet(c, t2m));                                // Get t2m
    MTC(Xform3D_Concatenate(t2m, m2c, t2c));                                // Transform both to t2c

    mtHandle r2c = Xform3D_New(); // tooltip to camera xform handle

    MTC(Marker_Marker2CameraXfGet(refHandle, CurrCamera, m2c, &IdentifyingCamera)); // Get m2c

    MTC(Marker_WasIdentifiedGet(refHandle, CurrCamera, &m_LastTrackerIdentifiedInRefSpace));

    if (m_LastTrackerIdentifiedInRefSpace)
    {
      MTC(Marker_Tooltip2MarkerXfGet(refHandle, t2m)); // Get t2m
      MTC(Xform3D_Concatenate(t2m, m2c, r2c));         // Transform both to r2c

      // note: claron quarternion has different order than the mitk quarternion
      double quaternionT2C[4];
      MTC(Xform3D_RotQuaternionsGet(t2c, quaternionT2C));
      rotateYAxisNinetyDegree(quaternionT2C, quaternionT2C);

      double quaternionR2C[4];
      MTC(Xform3D_RotQuaternionsGet(r2c, quaternionR2C));
      rotateYAxisNinetyDegree(quaternionR2C, quaternionR2C);

      Xform3D_Free(r2c);

      mitk::Quaternion QuaternionToSet1;
      QuaternionToSet1[3] = quaternionT2C[0];
      QuaternionToSet1[0] = quaternionT2C[1];
      QuaternionToSet1[1] = quaternionT2C[2];
      QuaternionToSet1[2] = quaternionT2C[3];

      mitk::Quaternion QuaternionToSet2;
      QuaternionToSet2[3] = quaternionR2C[0];
      QuaternionToSet2[0] = quaternionR2C[1];
      QuaternionToSet2[1] = quaternionR2C[2];
      QuaternionToSet2[2] = quaternionR2C[3];

      auto finalToSet = (QuaternionToSet2.inverse() * QuaternionToSet1);
      Quarternions[0] = finalToSet[3];
      Quarternions[1] = finalToSet[0];
      Quarternions[2] = finalToSet[1];
      Quarternions[3] = finalToSet[2];

      t2c = 0; // as we don't have to take it from claron again...
    }

    if (!m_LastTrackerIdentifiedInRefSpace && (m_HandleForTrackerless.find(refHandle) != m_HandleForTrackerless.end()))
    {
      // Get m2c
      MTC(Marker_Marker2CameraXfGet(c, CurrCamera, m2c, &IdentifyingCamera));
      // Get t2m
      MTC(Marker_Tooltip2MarkerXfGet(c, t2m));
      // Transform both to t2c
      MTC(Xform3D_Concatenate(t2m, m2c, t2c));

      auto itr = m_TrackerlessHandleLastValidData.find(refHandle);
      if (itr != m_TrackerlessHandleLastValidData.end())
      {
        double quaternionT2C[4];
        MTC(Xform3D_RotQuaternionsGet(t2c, quaternionT2C));
        rotateYAxisNinetyDegree(quaternionT2C, quaternionT2C);

        double quaternionR2C[4];
        quaternionR2C[0] = itr->second.second[0];
        quaternionR2C[1] = itr->second.second[1];
        quaternionR2C[2] = itr->second.second[2];
        quaternionR2C[3] = itr->second.second[3];
        rotateYAxisNinetyDegree(quaternionR2C, quaternionR2C);

        mitk::Quaternion QuaternionToSet1;
        QuaternionToSet1[3] = quaternionT2C[0];
        QuaternionToSet1[0] = quaternionT2C[1];
        QuaternionToSet1[1] = quaternionT2C[2];
        QuaternionToSet1[2] = quaternionT2C[3];

        mitk::Quaternion QuaternionToSet2;
        QuaternionToSet2[3] = quaternionR2C[0];
        QuaternionToSet2[0] = quaternionR2C[1];
        QuaternionToSet2[1] = quaternionR2C[2];
        QuaternionToSet2[2] = quaternionR2C[3];

        auto finalToSet = (QuaternionToSet2.inverse() * QuaternionToSet1);
        Quarternions[0] = finalToSet[3];
        Quarternions[1] = finalToSet[0];
        Quarternions[2] = finalToSet[1];
        Quarternions[3] = finalToSet[2];

        m_LastTrackerIdentifiedInRefSpace = true;

        t2c = 0; // as we don't have to take it from claron again...
      }
    }
  }
  else
  {
    // Get m2c
    MTC(Marker_Marker2CameraXfGet(c, CurrCamera, m2c, &IdentifyingCamera));
    // Get t2m
    MTC(Marker_Tooltip2MarkerXfGet(c, t2m));
    // Transform both to t2c
    MTC(Xform3D_Concatenate(t2m, m2c, t2c));
  }

  if (0 != t2c)
    MTC(Xform3D_RotQuaternionsGet(t2c, Quarternions));

  // If this tracker is enable for trackerless than we need to keep its last valid value...
  if (m_HandleForTrackerless.find(c) != m_HandleForTrackerless.end())
  {
    auto &valRef = m_TrackerlessHandleLastValidData[c];
    valRef.second.clear();
    valRef.second.push_back(Quarternions[0]);
    valRef.second.push_back(Quarternions[1]);
    valRef.second.push_back(Quarternions[2]);
    valRef.second.push_back(Quarternions[3]);
  }

  ////////////////////////////////////////////////////////////////////
  // Below logic is for rotating Quaternion around axis
  // Just for example
  //
  /*Quaternion Quaternion::create_from_axis_angle(const double &xx, const double &yy, const double &zz, const double &a)
  {
    // Here we calculate the sin( theta / 2) once for optimization
    double factor = sin(a / 2.0);

    // Calculate the x, y and z of the quaternion
    double x = xx * factor;
    double y = yy * factor;
    double z = zz * factor;

    // Calcualte the w value by cos( theta / 2 )
    double w = cos(a / 2.0);

    return Quaternion(x, y, z, w).normalize();
  }*/

  mitk::Quaternion finalQuaternionToSet;

  if (m_LastTrackerIdentifiedInRefSpace && !m_ReferenceNotSet)
  {
    finalQuaternionToSet[3] = Quarternions[0];
    finalQuaternionToSet[0] = Quarternions[1];
    finalQuaternionToSet[1] = Quarternions[2];
    finalQuaternionToSet[2] = Quarternions[3];
  }
  else
  {
    // Here we have to make a -90°-turn around the Y-axis because of a bug of the
    // MTC-library.
    double finalRes[4];
    rotateYAxisNinetyDegree(Quarternions, finalRes);
    finalQuaternionToSet[3] = finalRes[0];
    finalQuaternionToSet[0] = finalRes[1];
    finalQuaternionToSet[1] = finalRes[2];
    finalQuaternionToSet[2] = finalRes[3];
  }

  returnValue.push_back(finalQuaternionToSet[3]);
  returnValue.push_back(finalQuaternionToSet[0]);
  returnValue.push_back(finalQuaternionToSet[1]);
  returnValue.push_back(finalQuaternionToSet[2]);

  return returnValue;
}

void mitk::ClaronInterface::rotateYAxisNinetyDegree(double quaternionWithFirstTheta[4],
                                                    double *outQuaternionWithFirstTheta)
{
  if (nullptr == outQuaternionWithFirstTheta)
    return;

  mitk::Quaternion claronQuaternion;

  // note: claron quarternion has different order than the mitk quarternion
  claronQuaternion[3] = quaternionWithFirstTheta[0];
  claronQuaternion[0] = quaternionWithFirstTheta[1];
  claronQuaternion[1] = quaternionWithFirstTheta[2];
  claronQuaternion[2] = quaternionWithFirstTheta[3];

  mitk::Quaternion minusNinetyDegreeY;
  minusNinetyDegreeY[3] = __OneDivideByRootTwo; // cos(-90/2)
  minusNinetyDegreeY[0] = 0;
  minusNinetyDegreeY[1] = -1 * __OneDivideByRootTwo; // sin(-90/2)
  minusNinetyDegreeY[2] = 0;

  // calculate the result...
  mitk::Quaternion finalQuaternion = (minusNinetyDegreeY * claronQuaternion);
  outQuaternionWithFirstTheta[0] = finalQuaternion[3];
  outQuaternionWithFirstTheta[1] = finalQuaternion[0];
  outQuaternionWithFirstTheta[2] = finalQuaternion[1];
  outQuaternionWithFirstTheta[3] = finalQuaternion[2];
}

std::vector<double> mitk::ClaronInterface::GetQuaternions(claronToolHandle c)
{
  std::vector<double> returnValue;

  double Quarternions[4];

  mtHandle refHandle;
  MTC(Marker_ReferenceMarkerHandleGet(c, &refHandle));

  m_ReferenceNotSet = (0 == refHandle || refHandle == CurrCamera);

  if (!m_ReferenceNotSet)
  {
    /*MTC(Marker_Marker2ReferenceXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera));
    MTC(Marker_MarkerWasIdentifiedInRefSpaceGet(c, CurrCamera, &m_LastTrackerIdentifiedInRefSpace));*/
    MTC(Marker_Marker2CameraXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera)); // Get m2c

    mtHandle r2c = Xform3D_New();                                                   // tooltip to camera xform handle
    MTC(Marker_Marker2CameraXfGet(refHandle, CurrCamera, r2c, &IdentifyingCamera)); // Get r2c

    MTC(Marker_WasIdentifiedGet(refHandle, CurrCamera, &m_LastTrackerIdentifiedInRefSpace));

    if (m_LastTrackerIdentifiedInRefSpace)
    {
      // note: claron quarternion has different order than the mitk quarternion
      double quaternionT2C[4];
      MTC(Xform3D_RotQuaternionsGet(PoseXf, quaternionT2C));
      rotateYAxisNinetyDegree(quaternionT2C, quaternionT2C);

      double quaternionR2C[4];
      MTC(Xform3D_RotQuaternionsGet(r2c, quaternionR2C));
      rotateYAxisNinetyDegree(quaternionR2C, quaternionR2C);

      Xform3D_Free(r2c);

      mitk::Quaternion QuaternionToSet1;
      QuaternionToSet1[3] = quaternionT2C[0];
      QuaternionToSet1[0] = quaternionT2C[1];
      QuaternionToSet1[1] = quaternionT2C[2];
      QuaternionToSet1[2] = quaternionT2C[3];

      mitk::Quaternion QuaternionToSet2;
      QuaternionToSet2[3] = quaternionR2C[0];
      QuaternionToSet2[0] = quaternionR2C[1];
      QuaternionToSet2[1] = quaternionR2C[2];
      QuaternionToSet2[2] = quaternionR2C[3];

      auto finalToSet = (QuaternionToSet2.inverse() * QuaternionToSet1);
      Quarternions[0] = finalToSet[3];
      Quarternions[1] = finalToSet[0];
      Quarternions[2] = finalToSet[1];
      Quarternions[3] = finalToSet[2];

      PoseXf = 0; // as we don't have to take it from claron again...
    }

    if (!m_LastTrackerIdentifiedInRefSpace && (m_HandleForTrackerless.find(refHandle) != m_HandleForTrackerless.end()))
    {
      MTC(Marker_Marker2CameraXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera));

      auto itr = m_TrackerlessHandleLastValidData.find(refHandle);
      if (itr != m_TrackerlessHandleLastValidData.end())
      {
        // note: claron quarternion has different order than the mitk quarternion
        double quaternionT2C[4];
        MTC(Xform3D_RotQuaternionsGet(PoseXf, quaternionT2C));
        rotateYAxisNinetyDegree(quaternionT2C, quaternionT2C);

        double quaternionR2C[4];
        quaternionR2C[0] = itr->second.second[0];
        quaternionR2C[1] = itr->second.second[1];
        quaternionR2C[2] = itr->second.second[2];
        quaternionR2C[3] = itr->second.second[3];
        rotateYAxisNinetyDegree(quaternionR2C, quaternionR2C);

        Xform3D_Free(r2c);

        mitk::Quaternion QuaternionToSet1;
        QuaternionToSet1[3] = quaternionT2C[0];
        QuaternionToSet1[0] = quaternionT2C[1];
        QuaternionToSet1[1] = quaternionT2C[2];
        QuaternionToSet1[2] = quaternionT2C[3];

        mitk::Quaternion QuaternionToSet2;
        QuaternionToSet2[3] = quaternionR2C[0];
        QuaternionToSet2[0] = quaternionR2C[1];
        QuaternionToSet2[1] = quaternionR2C[2];
        QuaternionToSet2[2] = quaternionR2C[3];

        auto finalToSet = (QuaternionToSet2.inverse() * QuaternionToSet1);
        Quarternions[0] = finalToSet[3];
        Quarternions[1] = finalToSet[0];
        Quarternions[2] = finalToSet[1];
        Quarternions[3] = finalToSet[2];

        m_LastTrackerIdentifiedInRefSpace = true;

        PoseXf = 0; // as we don't have to take it from claron again...
      }
    }
  }
  else
    MTC(Marker_Marker2CameraXfGet(c, CurrCamera, PoseXf, &IdentifyingCamera));

  if (0 != PoseXf)
    MTC(Xform3D_RotQuaternionsGet(PoseXf, Quarternions));

  ////////////////////////////////////////////////////////////////////
  // Below logic is for rotating Quaternion around axis
  // Just for example
  //
  /*Quaternion Quaternion::create_from_axis_angle(const double &xx, const double &yy, const double &zz, const double &a)
  {
    // Here we calculate the sin( theta / 2) once for optimization
    double factor = sin(a / 2.0);

    // Calculate the x, y and z of the quaternion
    double x = xx * factor;
    double y = yy * factor;
    double z = zz * factor;

    // Calcualte the w value by cos( theta / 2 )
    double w = cos(a / 2.0);

    return Quaternion(x, y, z, w).normalize();
  }*/

  mitk::Quaternion finalQuaternionToSet;

  if (m_LastTrackerIdentifiedInRefSpace && !m_ReferenceNotSet)
  {
    finalQuaternionToSet[3] = Quarternions[0];
    finalQuaternionToSet[0] = Quarternions[1];
    finalQuaternionToSet[1] = Quarternions[2];
    finalQuaternionToSet[2] = Quarternions[3];
  }
  else
  {
    // Here we have to make a -90°-turn around the Y-axis because of a bug of the
    // MTC-library.
    double finalRes[4];
    rotateYAxisNinetyDegree(Quarternions, finalRes);
    finalQuaternionToSet[3] = finalRes[0];
    finalQuaternionToSet[0] = finalRes[1];
    finalQuaternionToSet[1] = finalRes[2];
    finalQuaternionToSet[2] = finalRes[3];
  }

  returnValue.push_back(finalQuaternionToSet[3]);
  returnValue.push_back(finalQuaternionToSet[0]);
  returnValue.push_back(finalQuaternionToSet[1]);
  returnValue.push_back(finalQuaternionToSet[2]);

  return returnValue;
}

std::string mitk::ClaronInterface::GetName(claronToolHandle c)
{
  MTC(Marker_NameGet(c, &m_ClaronToolNameCache[0], MT_MAX_STRING_LENGTH, 0));
  return m_ClaronToolNameCache;
}

bool mitk::ClaronInterface::IsTracking()
{
  return this->isTracking;
}

bool mitk::ClaronInterface::IsMicronTrackerInstalled()
{
  return true;
}

int mitk::ClaronInterface::GetToolThermalHazard()
{
  return ToolThermalHazard;
}

void mitk::ClaronInterface::SetToolThermalHazard(int thermal_hazard)
{
  ToolThermalHazard = thermal_hazard;
}

bool mitk::ClaronInterface::IsLastTrackerIdentifiedInRefSpace()
{
  return this->m_LastTrackerIdentifiedInRefSpace;
}

bool mitk::ClaronInterface::IsLastTrackerReferenceNotSet()
{
  return this->m_ReferenceNotSet;
}

void mitk::ClaronInterface::SetTrackerlessNavEnabledTracker(mtHandle trackerHandle, bool enable)
{
  if (0 == trackerHandle)
    return;

  if (enable)
  {
    if (m_HandleForTrackerless.find(trackerHandle) == m_HandleForTrackerless.end())
      m_HandleForTrackerless.insert(trackerHandle);     // insert only is not already present.
  }
  else
  {
    if (m_HandleForTrackerless.find(trackerHandle) != m_HandleForTrackerless.end())
      m_HandleForTrackerless.erase(trackerHandle);
  }
}
// HRS_NAVIGATION_MODIFICATION ends
