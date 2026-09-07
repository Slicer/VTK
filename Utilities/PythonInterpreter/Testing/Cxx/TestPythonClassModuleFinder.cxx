// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

// Check that a C++ object whose class has not been wrapped yet (because the
// Python module that wraps it has not been imported) is wrapped as its actual
// class once the vtkmodules class module table lists it, and that a
// class that was returned as its nearest base class is wrapped as itself once
// its module gets imported.

#include "vtkPythonInterpreter.h"
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPythonUtil.h>
#include <vtkSmartPyObject.h>

#include <iostream>

namespace
{
// Check that the object is wrapped as the given class (or as the Python
// class that overrides it, see vtkmodules.util.data_model).
bool CheckClass(vtkObjectBase* object, const char* expectedClassName, const char* description)
{
  vtkSmartPyObject pyObject;
  pyObject.TakeReference(vtkPythonUtil::GetObjectFromPointer(object));
  PyTypeObject* expectedType = vtkPythonUtil::FindClassTypeObject(expectedClassName);
  if (!pyObject || expectedType == nullptr || Py_TYPE(pyObject.GetPointer()) != expectedType)
  {
    std::cerr << description << ": expected class " << expectedClassName << ", got "
              << (pyObject ? vtkPythonUtil::StripModuleFromObject(pyObject) : "(null)") << std::endl;
    return false;
  }
  return true;
}

bool RunScript(const char* script)
{
  if (vtkPythonInterpreter::RunSimpleString(script) != 0)
  {
    std::cerr << "Script failed:\n" << script << std::endl;
    return false;
  }
  return true;
}
}

int TestPythonClassModuleFinder(int, char*[])
{
  vtkPythonInterpreter::Initialize();
  vtkPythonScopeGilEnsurer gilEnsurer(true, true);

  // Only import the module that provides vtkObject: vtkPolyData (from
  // vtkCommonDataModel) is not wrapped yet.
  if (!RunScript("import sys\n"
                 "import vtkmodules.vtkCommonCore\n"
                 "assert 'vtkmodules.vtkCommonDataModel' not in sys.modules\n"))
  {
    return EXIT_FAILURE;
  }

  // While the class is not listed in the class module table, the object is
  // wrapped as its nearest wrapped base class...
  vtkNew<vtkPolyData> polyData1;
  if (!CheckClass(polyData1, "vtkObject", "Without table entry"))
  {
    return EXIT_FAILURE;
  }
  // ...and the module that provides it is left alone.
  if (!RunScript("assert 'vtkmodules.vtkCommonDataModel' not in sys.modules\n"))
  {
    return EXIT_FAILURE;
  }

  // Declare which module provides the class (registering does not import it).
  if (!RunScript("import vtkmodules\n"
                 "vtkmodules.register_class_modules({'vtkPolyData': 'vtkCommonDataModel'})\n"
                 "assert 'vtkmodules.vtkCommonDataModel' not in sys.modules\n"))
  {
    return EXIT_FAILURE;
  }

  // Now the module gets imported and the object is wrapped as its actual class,
  // even though vtkPolyData was previously associated with vtkObject.
  vtkNew<vtkPolyData> polyData2;
  if (!CheckClass(polyData2, "vtkPolyData", "With table entry"))
  {
    return EXIT_FAILURE;
  }
  if (!RunScript("assert 'vtkmodules.vtkCommonDataModel' in sys.modules\n"
                 "from vtkmodules.vtkCommonDataModel import vtkPolyData\n"
                 "assert vtkPolyData.__name__ == 'vtkPolyData', vtkPolyData.__name__\n"
                 "assert vtkPolyData().GetClassName() == 'vtkPolyData'\n"
                 "assert vtkPolyData().GetNumberOfPolys() == 0\n"))
  {
    return EXIT_FAILURE;
  }

  // The object that was wrapped as its base class before (and whose Python
  // object has been released since) is now wrapped as its actual class too.
  if (!CheckClass(polyData1, "vtkPolyData", "Object previously wrapped as its base class"))
  {
    return EXIT_FAILURE;
  }

  vtkPythonInterpreter::Finalize();
  return EXIT_SUCCESS;
}
