/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "mitkPythonService.h"
#include <mitkIOUtil.h>

#ifdef _DEBUG
  #undef _DEBUG
  #include <python.h>
  #define _DEBUG
#else
  #include <python.h>
#endif

#include "PythonPath.h"
#include <vtkPolyData.h>
#include <mitkRenderingManager.h>
#include <mitkImageReadAccessor.h>
#include <mitkImageWriteAccessor.h>
#include <itksys/SystemTools.hxx>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <mitkExceptionMacro.h>

#ifndef WIN32
#include <dlfcn.h>
#endif

#include"swigpyrun.h"

typedef itksys::SystemTools ist;

mitk::PythonService::PythonService()
  : m_ItkWrappingAvailable(true),
    m_ErrorOccured(false)
{
  if (!Py_IsInitialized())
  {
    Py_Initialize();
  }
  PyGILState_STATE gState = PyGILState_Ensure();
  std::string programPath = mitk::IOUtil::GetProgramPath();
  std::replace(programPath.begin(), programPath.end(), '\\', '/');
  programPath.append("/");
  MITK_INFO << programPath;
  std::string pythonCommand;

  pythonCommand.append("import SimpleITK as sitk\n");
  pythonCommand.append("import SimpleITK._SimpleITK as _SimpleITK\n");
  pythonCommand.append("import numpy\n");

  pythonCommand.append("import site, sys\n");
  pythonCommand.append("import os\n");
  pythonCommand.append("sys.path.append('')\n");
  pythonCommand.append("sys.path.append('" + programPath + "')\n");
  pythonCommand.append("sys.path.append('" + std::string(SWIG_MITK_WRAPPING) + "')\n");
  pythonCommand.append("sys.path.append('" +std::string(EXTERNAL_DIST_PACKAGES) + "')\n");
  pythonCommand.append("\nsite.addsitedir('"+std::string(EXTERNAL_SITE_PACKAGES)+"')\n");
  std::string searchForDll = "if sys.version_info[1] > 7:\n"
                             "  for root, dirs, files in os.walk('" + std::string(SWIG_PYTHON_BUILD_OUTPUT) +"'):\n"
                             "      for file in files:\n"
                             "          if file.endswith('.dll'):\n"
                             "              os.add_dll_directory(root)\n"
                             "              break\n";
  pythonCommand.append(searchForDll);
  
  if (PyRun_SimpleString(pythonCommand.c_str()) == -1)
  {
    MITK_ERROR << "Something went wrong in setting the path in Python";
  }
  PyObject *main = PyImport_AddModule("__main__");
  m_GlobalDictionary = PyModule_GetDict(main);
  m_LocalDictionary = m_GlobalDictionary;
  m_ThreadState = PyEval_SaveThread();
}

mitk::PythonService::~PythonService()
{
}

void mitk::PythonService::AddRelativeSearchDirs(std::vector< std::string > dirs)
{
  for (auto dir : dirs)
  {
    try
    {
      std::string path = std::string(MITK_ROOT) + dir;
      std::string pythonCommand = "import sys\nsys.path.append('" + path + "')\n";
      this->Execute(pythonCommand.c_str());
    }
    catch (const mitk::Exception)
    {
      mitkThrow() << "An error occured setting the relative project path";
    }
  }
}

void mitk::PythonService::AddAbsoluteSearchDirs(std::vector< std::string > dirs)
{
  for (auto dir : dirs)
  {
    try
    {
      std::string pythonCommand = "import sys\nsys.path.append('" + dir + "')\n";
      this->Execute(pythonCommand.c_str());
    }
    catch (const mitk::Exception)
    {
      mitkThrow() << "An error occured setting the absolute project path";
    }
  }
}

std::string mitk::PythonService::Execute(const std::string &stdpythonCommand, int commandType)
{
  if (!Py_IsInitialized())
  {
    Py_Initialize();
  }
  std::string result = "";

  PyGILState_STATE gState = PyGILState_Ensure();
  try
  {
    switch (commandType)
    {
      case IPythonService::SINGLE_LINE_COMMAND:
        commandType = Py_single_input;
        break;
      case IPythonService::MULTI_LINE_COMMAND:
        commandType = Py_file_input;
        break;
      case IPythonService::EVAL_COMMAND:
        commandType = Py_eval_input;
        break;
      default:
        commandType = Py_file_input;
    }
    PyObject* executionResult = PyRun_String(stdpythonCommand.c_str(), commandType, m_GlobalDictionary, m_LocalDictionary);
    this->NotifyObserver(stdpythonCommand);
    if (executionResult)
    {
      PyObject *objectsRepresentation = PyObject_Repr(executionResult);
      const char *resultChar = PyUnicode_AsUTF8(objectsRepresentation);
      result = std::string(resultChar);
      m_ThreadState = PyEval_SaveThread();
      m_ErrorOccured = false;
    }
    else
    {
      m_ErrorOccured = true;
      mitkThrow() << "An error occured while running the Python code";
    }
  }
  catch (const mitk::Exception) 
  {
    PyErr_Print();
    m_ThreadState = PyEval_SaveThread();
    throw;
  }
  return result;
}


void mitk::PythonService::ExecuteScript(const std::string &pythonScript)
{
  std::ifstream t(pythonScript.c_str());
  std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  t.close();
  try
  {
    this->Execute(str.c_str(), MULTI_LINE_COMMAND);
  }
  catch (const mitk::Exception)
  {
    throw;
  }
}

std::vector<mitk::PythonVariable> mitk::PythonService::GetVariableStack()
{
  std::vector<mitk::PythonVariable> list;
  PyGILState_STATE gState = PyGILState_Ensure();
  try
  {
    PyObject *dict = PyImport_GetModuleDict();
    PyObject *object = PyDict_GetItemString(dict, "__main__");
    if (!object)
    {
      mitkThrow() << "An error occured getting the Dictionary";
    }
    PyObject *dirMain = PyObject_Dir(object);
    PyObject *tempObject = nullptr;

    if (dirMain)
    {
      std::string name, attrValue, attrType;

      for (int i = 0; i < PyList_Size(dirMain); i++)
      {
        tempObject = PyList_GetItem(dirMain, i);
        if (!tempObject)
        {
          mitkThrow() << "An error occured getting an item from the dictionary";
        }
        PyObject *objectsRepresentation = PyObject_Repr(tempObject);
        const char *objectChar = PyUnicode_AsUTF8(objectsRepresentation);
        std::string name = std::string(objectChar);
        name = name.substr(1, name.size() - 2);
        tempObject = PyObject_GetAttrString(object, name.c_str());
        if (!tempObject)
        {
          mitkThrow() << "Could not get the attribute to determine type";
        }
        attrType = tempObject->ob_type->tp_name;

        PyObject *valueStringRepresentation = PyObject_Repr(tempObject);
        const char *valueChar = PyUnicode_AsUTF8(valueStringRepresentation);
        std::string attrValue = std::string(valueChar);

        mitk::PythonVariable var;
        var.m_Name = name;
        var.m_Value = attrValue;
        var.m_Type = attrType;
        list.push_back(var);
      }
    }
    m_ThreadState = PyEval_SaveThread();
  }
  catch (const mitk::Exception)
  {
    m_ThreadState = PyEval_SaveThread();
    throw;
  }
  return list;
}

std::string mitk::PythonService::GetVariable(const std::string& name)
{
  std::vector<mitk::PythonVariable> allVars;
  try
  {
    allVars = this->GetVariableStack();
  }
  catch (const mitk::Exception)
  {
    mitkThrow() << "Error getting the variable stack";
  }
  for (unsigned int i = 0; i < allVars.size(); i++)
  {
    if (allVars.at(i).m_Name == name)
      return allVars.at(i).m_Value;
  }

  return "";
}

bool mitk::PythonService::DoesVariableExist(const std::string& name)
{
  bool varExists = false;
  std::vector<mitk::PythonVariable> allVars;
  try
  {
    allVars = this->GetVariableStack();
  }
  catch (const mitk::Exception)
  {
    mitkThrow() << "Error getting the variable stack";
  }
 
  for (unsigned int i = 0; i < allVars.size(); i++)
  {
    if (allVars.at(i).m_Name == name)
    {
      varExists = true;
      break;
    }
  }

  return varExists;
}

void mitk::PythonService::AddPythonCommandObserver(mitk::PythonCommandObserver *observer)
{
  //only add observer if it isn't already in list of observers
  if (!(std::find(m_Observer.begin(), m_Observer.end(), observer) != m_Observer.end()))
  {
    m_Observer.push_back(observer);
  }
}

void mitk::PythonService::RemovePythonCommandObserver(mitk::PythonCommandObserver *observer)
{
  for (std::vector<mitk::PythonCommandObserver *>::iterator iter = m_Observer.begin(); iter != m_Observer.end(); ++iter)
  {
    if (*iter == observer)
    {
      m_Observer.erase(iter);
      break;
    }
  }
}

void mitk::PythonService::NotifyObserver(const std::string &command)
{
  for (int i = 0; i < m_Observer.size(); ++i)
  {
    m_Observer.at(i)->CommandExecuted(command);
  }
}


int mitk::PythonService::GetNumberOfObserver() 
{
  return m_Observer.size();
}

bool mitk::PythonService::IsSimpleItkPythonWrappingAvailable()
{
  return m_ItkWrappingAvailable;
}

bool mitk::PythonService::CopyToPythonAsSimpleItkImage(mitk::Image::Pointer image, const std::string &stdvarName)
{
  std::string transferToPython = "import pyMITK\n"
                                 "import SimpleITK as sitk\n"
                                 "mitk_image = None\n"
                                 "geometry = None\n"
                                 +stdvarName+" = None\n"
                                 "\n"
                                 "def setup(image_from_cxx):\n"
                                 "    print ('setup called with', image_from_cxx)\n"
                                 "    global mitk_image\n"
                                 "    mitk_image = image_from_cxx\n"
                                 "\n"
                                 "def convert_to_sitk(spacing, origin):\n"
                                 "    np = pyMITK.GetArrayViewFromImage(mitk_image)\n"
                                 "    global "+stdvarName+"\n"
                                 "    "+stdvarName+" = sitk.GetImageFromArray(np)\n"
                                 "    npSpacingArray = numpy.array(spacing , dtype=float)\n"
                                 "    "+stdvarName+".SetSpacing(npSpacingArray)\n"
                                 "    npOriginArray = numpy.array(origin , dtype=float)\n"
                                 "    " +stdvarName +".SetOrigin(npOriginArray)\n";

  this->Execute(transferToPython);

  mitk::Image::Pointer *img = &image;
  PyGILState_STATE gState = PyGILState_Ensure();
  //necessary for transfer array from C++ to Python
  import_array();
  PyObject *main = PyImport_ImportModule("__main__");
  if (main == NULL)
  {
    mitkThrow() << "Something went wrong getting the main module";
  }

  swig_type_info *pTypeInfo = nullptr;
  pTypeInfo = SWIG_TypeQuery("_p_itk__SmartPointerT_mitk__Image_t");
  int owned = 0;
  PyObject *pInstance = SWIG_NewPointerObj(reinterpret_cast<void *>(img), pTypeInfo, owned);
  if (pInstance == NULL)
  {
    mitkThrow() << "Something went wrong creating the Python instance of the image";
  }
  //transfer correct image spacing to Python
  int numberOfImageDimension = image->GetDimension();
  auto spacing = image->GetGeometry()->GetSpacing();
  auto *spacingptr = &spacing;
  int nd = 1;
  npy_intp dims[] = {numberOfImageDimension};
  PyObject *spacingArray = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, (void *)spacingptr);

  //transfer correct image origin to Python
  auto origin = image->GetGeometry()->GetOrigin();
  auto *originptr = &origin;
  PyObject *originArray = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, (void *)originptr);

  PyObject *setup = PyObject_GetAttrString(main, "setup");
  PyObject *result = PyObject_CallFunctionObjArgs(setup, pInstance, NULL);
  if (result == NULL)
  {
    mitkThrow() << "Something went wrong setting the MITK image in Python";
  }
  PyObject *convert = PyObject_GetAttrString(main, "convert_to_sitk");
  result = PyObject_CallFunctionObjArgs(convert,spacingArray, originArray, NULL);
  if (result == NULL)
  {
    mitkThrow() << "Something went wrong converting the MITK image to a SimpleITK image";
  }
  m_ThreadState = PyEval_SaveThread();
  return true;
}

mitk::Image::Pointer mitk::PythonService::CopySimpleItkImageFromPython(const std::string &stdvarName)
{
  mitk::Image::Pointer mitkImage;
  std::string convertToMITKImage = "import SimpleITK as sitk\n"
                                   "import pyMITK\n"
                                   "numpy_array = sitk.GetArrayViewFromImage("+stdvarName+")\n"
                                   "mitk_image = pyMITK.GetImageFromArray(numpy_array)\n"
                                   "spacing = "+stdvarName+".GetSpacing()\n"
                                   "origin = "+stdvarName +".GetOrigin()\n";
  this->Execute(convertToMITKImage);

  PyGILState_STATE gState = PyGILState_Ensure();

  PyObject *main = PyImport_AddModule("__main__");
  PyObject *globals = PyModule_GetDict(main);
  PyObject *pyImage = PyDict_GetItemString(globals, "mitk_image");
  if (pyImage==NULL)
  {
    mitkThrow() << "Could not get image from Python";
  }

  int res = 0;
  void *voidImage;
  swig_type_info *pTypeInfo = nullptr;
  pTypeInfo = SWIG_TypeQuery("_p_itk__SmartPointerT_mitk__Image_t");
  res = SWIG_ConvertPtr(pyImage, &voidImage, pTypeInfo, 0);
  if (!SWIG_IsOK(res))
  {
    mitkThrow() << "Could not cast image to C++ type";
  }

  mitkImage = *(reinterpret_cast<mitk::Image::Pointer *>(voidImage));

  PyObject *spacing = PyDict_GetItemString(globals, "spacing");
  mitk::Vector3D spacingMITK;
  for (int i = 0; i < PyTuple_GET_SIZE(spacing); i++)
  {
    PyObject *spacingPy = PyTuple_GetItem(spacing, i);
    double elem = PyFloat_AsDouble(spacingPy);
    spacingMITK[i] = elem;
  }

  PyObject *origin = PyDict_GetItemString(globals, "origin");
  mitk::Point3D originMITK;
  for (int i = 0; i < PyTuple_GET_SIZE(origin); i++)
  {
    PyObject *originPy = PyTuple_GetItem(origin, i);
    double elem = PyFloat_AsDouble(originPy);
    originMITK[i] = elem;
  }

  m_ThreadState = PyEval_SaveThread();
  mitkImage->GetGeometry()->SetSpacing(spacingMITK);
  mitkImage->GetGeometry()->SetOrigin(originMITK);
  return mitkImage;
}

bool mitk::PythonService::CopyMITKImageToPython(mitk::Image::Pointer &image, const std::string &stdvarName)
{
  std::string transferToPython = "import pyMITK\n"
                                 + stdvarName +" = None\n"
                                 "\n"
                                 "def setup(image_from_cxx):\n"
                                 "    print ('setup called with', image_from_cxx)\n"
                                 "    global "+stdvarName+"\n"
                                 "    "+stdvarName+" = image_from_cxx\n";

  this->Execute(transferToPython);

  mitk::Image::Pointer *img = &image;
  PyGILState_STATE gState = PyGILState_Ensure();
  PyObject *main = PyImport_ImportModule("__main__");
  if (main == NULL)
  {
    mitkThrow() << "Something went wrong getting the main module";
  }

  swig_type_info *pTypeInfo = nullptr;
  pTypeInfo = SWIG_TypeQuery("_p_itk__SmartPointerT_mitk__Image_t");
  int owned = 0;
  PyObject *pInstance = SWIG_NewPointerObj(reinterpret_cast<void *>(img), pTypeInfo, owned);
  if (pInstance == NULL)
  {
    mitkThrow() << "Something went wrong creating the Python instance of the image";
  }

  PyObject *setup = PyObject_GetAttrString(main, "setup");
  PyObject *result = PyObject_CallFunctionObjArgs(setup, pInstance, NULL);
  if (result == NULL)
  {
    mitkThrow() << "Something went wrong setting the MITK image in Python";
  }

  m_PythonMITKImages.push_back(image);
  m_ThreadState = PyEval_SaveThread();
  return true;
}

mitk::Image::Pointer GetImageFromPyObject(PyObject *pyImage) 
{
  mitk::Image::Pointer mitkImage;

  int res = 0;
  void *voidImage;
  swig_type_info *pTypeInfo = nullptr;
  pTypeInfo = SWIG_TypeQuery("_p_itk__SmartPointerT_mitk__Image_t");
  res = SWIG_ConvertPtr(pyImage, &voidImage, pTypeInfo, 0);
  if (!SWIG_IsOK(res))
  {
    mitkThrow() << "Could not cast image to C++ type";
  }

  mitkImage = *(reinterpret_cast<mitk::Image::Pointer *>(voidImage));

  return mitkImage;
}

mitk::Image::Pointer mitk::PythonService::CopyMITKImageFromPython(const std::string &stdvarName)
{
  mitk::Image::Pointer mitkImage;

  PyGILState_STATE gState = PyGILState_Ensure();

  PyObject *main = PyImport_AddModule("__main__");
  PyObject *globals = PyModule_GetDict(main);
  PyObject *pyImage = PyDict_GetItemString(globals, stdvarName.c_str());
  if (pyImage==NULL)
  {
    mitkThrow() << "Could not get image from Python";
  }

  mitkImage = GetImageFromPyObject(pyImage);

  m_ThreadState = PyEval_SaveThread();
  return mitkImage;
}

std::vector<mitk::Image::Pointer> mitk::PythonService::CopyListOfMITKImagesFromPython(const std::string &listVarName) 
{
  std::vector<mitk::Image::Pointer> mitkImages;

  PyGILState_STATE gState = PyGILState_Ensure();

  PyObject *main = PyImport_AddModule("__main__");
  PyObject *globals = PyModule_GetDict(main);
  PyObject *pyImageList = PyDict_GetItemString(globals, listVarName.c_str());
  if (pyImageList == NULL)
  {
    mitkThrow() << "Could not get image list from Python";
  }

  for (int i = 0; i < PyList_GET_SIZE(pyImageList);i++)
  {
    PyObject *pyImage = PyList_GetItem(pyImageList, i);
    mitk::Image::Pointer img = GetImageFromPyObject(pyImage);
    mitkImages.push_back(img);
  }

  m_ThreadState = PyEval_SaveThread();
  return mitkImages;
}

bool mitk::PythonService::PythonErrorOccured() const
{
  return m_ErrorOccured;
}

