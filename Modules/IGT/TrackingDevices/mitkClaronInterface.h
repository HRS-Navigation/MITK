/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef MITKCLARONINTERFACE_H_HEADER_INCLUDED_
#define MITKCLARONINTERFACE_H_HEADER_INCLUDED_

// HRS_NAVIGATION_MODIFICATION starts

#define MTC(func) {int r = func; if (r!=mtOK) MITK_ERROR << "MTC error:" << MTLastErrorString();};

#include <vector>
#include <string>

#include <MitkIGTExports.h>
#include "mitkCommon.h"

#include <itkObject.h>
#include <itkObjectFactory.h>

#include <set>
#include <map>
#include <utility>

#ifdef _WIN64 // Defined for applications for Win64.
typedef long long mtHandle;
#else
typedef int mtHandle;
#endif

namespace mitk
{
  typedef mtHandle claronToolHandle;

  /** Documentation:
  *   \brief An object of this class represents the interface to the MicronTracker. The methods of this class
  *          are calling the c-functions which are provided by the MTC-library. If the MicronTracker is not in
  *          use, which means the CMake-variable "MITK_USE_MICRON_TRACKER" is set to OFF, this class is replaced
  *          by a stub class called "ClaronInterfaceStub".
  *   \ingroup IGT
  */
  class MITKIGT_EXPORT ClaronInterface : public itk::Object
  {
  public:

    mitkClassMacroItkParent(ClaronInterface,itk::Object);
    itkFactorylessNewMacro(Self);
    itkCloneMacro(Self)
    /**
    * \brief Initialization of claroninterface.
    * \param calibrationDir   The directory where the device can find the camera calibration file.
    * \param toolFilesDir     The directory for the tool files.
    */
    void Initialize(std::string calibrationDir, std::string toolFilesDir);

    /**
    * \brief Opens the connection to the device and makes it ready to track tools.
    * \return Returns true if there is a connection to the device and the device is ready to track tools, false if not.
    */
    bool StartTracking();

    /**
    * \brief Clears all resources. After this method have been called the system isn't ready to track any longer.
    * \return Returns true if the operation was succesful, false if not.
    */
    bool StopTracking();

    /**
    * \return Returns all tools which have been detected at the last frame grab.
    */
    std::vector<claronToolHandle> GetAllActiveTools();

    /**
    * \return Returns the position of the tooltip. If no tooltip is defined the Method returns the position of the tool.
    */
    std::vector<double> GetTipPosition(claronToolHandle c);

    /**
     * \Returns the Thermal hazard associated with the tool position calculation.
     */
    int GetToolThermalHazard();

    /**
    * \return Returns the quarternions of the tooltip. If no tooltip is defined the Method returns the quarternions of the tool.
    */
    std::vector<double> GetTipQuaternions(claronToolHandle c);

    /**
    * \return Returns the position of the tool
    */
    std::vector<double> GetPosition(claronToolHandle c);

    /**
    * \return Returns the quaternion of the tool.
    */
    std::vector<double> GetQuaternions(claronToolHandle c);

    /**
    * \return Returns the name of the tool. This name is given by the calibration file.
    * \param c The handle of the tool, which name should be given back.
    */
    std::string GetName(claronToolHandle c);

    /**
    * \brief Grabs a frame from the camera.
    */
    void GrabFrame();

    /**
    * \return Returns wether the tracking device is tracking or not.
    */
    bool IsTracking();

    /**
    * \return   Returns wether the MicronTracker is installed (means wether the C-Make-Variable "MITK_USE_MICRON_TRACKER" is set ON),
    *           so returns true in this case. This is because the class mitkClaronInterfaceStub, in which the same Method returns false
    *            is used otherways.
    */
    bool IsMicronTrackerInstalled();

  	bool IsLastTrackerIdentifiedInRefSpace();
    bool IsLastTrackerReferenceNotSet();
    void SetTrackerlessNavEnabledTracker(mtHandle trackerHandle, bool enable);

    // Added by AmitRungta on 25-11-2021
    // This function will set if we want to process in background or normal after tracking starts.
    void fnSetProcessingInBackgroundThread(bool baProcessInBackgroundThread){ m_bDesiredProcessingInBackgroundThread = baProcessInBackgroundThread; };
    // This function will get the current state of the tracking and not the one actual desired. As there can be a case that we may have
    // desired to change the state but its already tracking with an older state.
    bool fnGetProcessingInBackgroundThread() const { return m_bCurrentProcessingInBackgroundThread; };


  protected:
    /**
    * \brief standard constructor
    */
    ClaronInterface();
    /**
    * \brief standard destructor
    */
    ~ClaronInterface() override;


    /** \brief Variable is true if the device is tracking at the moment, false if not.*/
    bool isTracking;

    /** \brief Variable which holds the directory which should contain the file BumbleBee_6400420.calib. This directory is needed by the MTC library.*/
    char calibrationDir[512];
    /** \brief Variable which holds a directory with some tool files in it. All this tools are trackable when the path is given to the MTC library.*/
    char markerDir[512];

	// Set the Thermal Hazard in tool tip calculation
    void mitk::ClaronInterface::SetToolThermalHazard(int thermal_hazard);

    // Some handles to communicate with the MTC library.
    mtHandle IdentifiedMarkers;
    mtHandle PoseXf;
    mtHandle CurrCamera;
    mtHandle IdentifyingCamera;
    int ToolThermalHazard;
    bool m_LastTrackerIdentifiedInRefSpace;
    bool m_ReferenceNotSet;

    std::set<mtHandle> m_HandleForTrackerless;
    std::map<mtHandle, std::pair<std::vector<double>, std::vector<double>>> m_TrackerlessHandleLastValidData;
    //------------------------------------------------

    void rotateYAxisNinetyDegree(double quaternionWithFirstTheta[4], double *outQuaternionWithFirstTheta);

  private:
    std::string m_ClaronToolNameCache;


protected:
    // Added by AmitRungta on 25-11-2021 
    bool m_bCurrentProcessingInBackgroundThread;
    bool m_bDesiredProcessingInBackgroundThread;
  };
}//mitk


// HRS_NAVIGATION_MODIFICATION ends
#endif
