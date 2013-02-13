/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/
#ifndef mitkPythonService_h
#define mitkPythonService_h

#include "mitkIPythonService.h"
#include <itkLightObject.h>
#include <ctkAbstractPythonManager.h>
#include "mitkSurface.h"
#include "mitkPythonExports.h"

namespace mitk
{
  ///
  /// implementation of the IPythonService using ctkabstractpythonmanager
  /// \see IPythonService
  class MITK_PYTHON_EXPORT PythonService: public itk::LightObject, public mitk::IPythonService
  {
  public:
      ///
      /// instantiate python manager here
      PythonService();
      ///
      /// empty implementation...
      ~PythonService();
      ///
      /// \see IPythonService::Execute()
      std::string Execute( const std::string& pythonCommand, int commandType = SINGLE_LINE_COMMAND );
      ///
      /// \see IPythonService::GetVariableStack()
      std::vector<PythonVariable> GetVariableStack() const;
      ///
      /// \see IPythonService::AddPythonCommandObserver()
      void AddPythonCommandObserver( PythonCommandObserver* observer );
      ///
      /// \see IPythonService::RemovePythonCommandObserver()
      void RemovePythonCommandObserver( PythonCommandObserver* observer );
      ///
      /// \see IPythonService::NotifyObserver()
      void NotifyObserver( const std::string& command );
      ///
      /// \see IPythonService::IsItkPythonWrappingAvailable()
      bool IsItkPythonWrappingAvailable();
      ///
      /// \see IPythonService::CopyToPythonAsItkImage()
      bool CopyToPythonAsItkImage( mitk::Image* image, const std::string& varName );
      ///
      /// \see IPythonService::CopyItkImageFromPython()
      mitk::Image::Pointer CopyItkImageFromPython( const std::string& varName );
      ///
      /// \see IPythonService::IsOpenCvPythonWrappingAvailable()
      bool IsOpenCvPythonWrappingAvailable();
      ///
      /// \see IPythonService::CopyToPythonAsCvImage()
      bool CopyToPythonAsCvImage( mitk::Image* image, const std::string& varName );
      ///
      /// \see IPythonService::CopyCvImageFromPython()
      mitk::Image::Pointer CopyCvImageFromPython( const std::string& varName );
      ///
      /// \see IPythonService::IsVtkPythonWrappingAvailable()
      bool IsVtkPythonWrappingAvailable();
      ///
      /// \see IPythonService::CopyToPythonAsVtkPolyData()
      bool CopyToPythonAsVtkPolyData( mitk::Surface* surface, const std::string& varName );
      ///
      /// \see IPythonService::CopyVtkPolyDataFromPython()
      mitk::Surface::Pointer CopyVtkPolyDataFromPython( const std::string& varName );
      ///
      /// \return the ctk abstract python manager instance
      ctkAbstractPythonManager* GetPythonManager();
  protected:
      QString GetTempImageName(const std::string &ext) const;
  private:
      QList<PythonCommandObserver*> m_Observer;
      ctkAbstractPythonManager m_PythonManager;
      static const QString m_TmpImageName;
      bool m_ItkWrappingAvailable;
      bool m_OpenCVWrappingAvailable;
      bool m_VtkWrappingAvailable;
  };
}
#endif
