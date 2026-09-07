#!/usr/bin/env python

# This tests that objects whose class is provided by a module that has not
# been imported yet are wrapped as their actual class once vtkmodules can tell
# which module provides the class (vtkmodules.register_class_modules), and
# that a class that was meanwhile wrapped as its nearest base class is
# wrapped as itself once its module gets imported.

import ctypes
import importlib.util
import sys

import vtkmodules
from vtkmodules.vtkRenderingCore import vtkRenderer
from vtkmodules.test import Testing


class TestClassModuleFinder(Testing.vtkTest):
    def testClassModuleFinder(self):
        self.assertNotIn("vtkmodules.vtkRenderingOpenGL2", sys.modules)

        # Load the library that provides the OpenGL object factory overrides
        # without importing the Python module that wraps its classes: this is
        # the situation of an application that imports VTK modules lazily and
        # receives objects of not yet wrapped classes from C++.
        spec = importlib.util.find_spec("vtkmodules.vtkRenderingOpenGL2")
        self.assertIsNotNone(spec)
        library = ctypes.CDLL(spec.origin)
        self.assertNotIn("vtkmodules.vtkRenderingOpenGL2", sys.modules)

        # Without a table entry, objects are wrapped as their nearest wrapped base class
        renderer = vtkRenderer()
        self.assertEqual(renderer.GetClassName(), "vtkOpenGLRenderer")
        self.assertEqual(type(renderer).__name__, "vtkRenderer")
        camera = renderer.GetActiveCamera()
        self.assertEqual(camera.GetClassName(), "vtkOpenGLCamera")
        self.assertEqual(type(camera).__name__, "vtkCamera")
        self.assertNotIn("vtkmodules.vtkRenderingOpenGL2", sys.modules)

        # Declare which module provides the classes (registering does not import)
        vtkmodules.register_class_modules({
            "vtkOpenGLRenderer": "vtkmodules.vtkRenderingOpenGL2",
            "vtkOpenGLCamera": "vtkRenderingOpenGL2",
        })
        self.assertNotIn("vtkmodules.vtkRenderingOpenGL2", sys.modules)

        # Objects created from Python (object factory override)...
        renderer2 = vtkRenderer()
        self.assertEqual(type(renderer2).__name__, "vtkOpenGLRenderer")
        self.assertEqual(type(renderer2).__module__, "vtkmodules.vtkRenderingOpenGL2")
        self.assertIn("vtkmodules.vtkRenderingOpenGL2", sys.modules)
        # ...and objects returned by C++ methods are wrapped as their actual class,
        # even for a class that was previously wrapped as its base class
        camera2 = renderer2.GetActiveCamera()
        self.assertEqual(type(camera2).__name__, "vtkOpenGLCamera")
        from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLCamera, vtkOpenGLRenderer
        self.assertIs(type(camera2), vtkOpenGLCamera)
        self.assertIs(type(renderer2), vtkOpenGLRenderer)
        self.assertEqual(vtkOpenGLCamera.__name__, "vtkOpenGLCamera")
        self.assertEqual(type(vtkOpenGLCamera()).__name__, "vtkOpenGLCamera")

        # Objects wrapped before their module was imported keep their type
        self.assertEqual(type(renderer).__name__, "vtkRenderer")
        self.assertEqual(type(camera).__name__, "vtkCamera")
        self.assertIsNotNone(library)


if __name__ == "__main__":
    Testing.main([(TestClassModuleFinder, "test")])
