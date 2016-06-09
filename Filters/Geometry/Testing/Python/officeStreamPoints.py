#!/usr/bin/env python
import vtk
from vtk.test import Testing
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

ren1 = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren1)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

# read data
#
reader = vtk.vtkStructuredGridReader()
reader.SetFileName(VTK_DATA_ROOT + "/Data/office.binary.vtk")
reader.Update()

length = reader.GetOutput().GetLength()
maxVelocity = reader.GetOutput().GetPointData().GetVectors().GetMaxNorm()
maxTime = 35.0 * length / maxVelocity

table1 = vtk.vtkStructuredGridGeometryFilter()
table1.SetInputConnection(reader.GetOutputPort())
table1.SetExtent(11, 15, 7, 9, 8, 8)
mapTable1 = vtk.vtkPolyDataMapper()
mapTable1.SetInputConnection(table1.GetOutputPort())
mapTable1.ScalarVisibilityOff()
table1Actor = vtk.vtkActor()
table1Actor.SetMapper(mapTable1)
table1Actor.GetProperty().SetColor(.59, .427, .392)

table2 = vtk.vtkStructuredGridGeometryFilter()
table2.SetInputConnection(reader.GetOutputPort())
table2.SetExtent(11, 15, 10, 12, 8, 8)
mapTable2 = vtk.vtkPolyDataMapper()
mapTable2.SetInputConnection(table2.GetOutputPort())
mapTable2.ScalarVisibilityOff()
table2Actor = vtk.vtkActor()
table2Actor.SetMapper(mapTable2)
table2Actor.GetProperty().SetColor(.59, .427, .392)

FilingCabinet1 = vtk.vtkStructuredGridGeometryFilter()
FilingCabinet1.SetInputConnection(reader.GetOutputPort())
FilingCabinet1.SetExtent(15, 15, 7, 9, 0, 8)
mapFilingCabinet1 = vtk.vtkPolyDataMapper()
mapFilingCabinet1.SetInputConnection(FilingCabinet1.GetOutputPort())
mapFilingCabinet1.ScalarVisibilityOff()
FilingCabinet1Actor = vtk.vtkActor()
FilingCabinet1Actor.SetMapper(mapFilingCabinet1)
FilingCabinet1Actor.GetProperty().SetColor(.8, .8, .6)

FilingCabinet2 = vtk.vtkStructuredGridGeometryFilter()
FilingCabinet2.SetInputConnection(reader.GetOutputPort())
FilingCabinet2.SetExtent(15, 15, 10, 12, 0, 8)
mapFilingCabinet2 = vtk.vtkPolyDataMapper()
mapFilingCabinet2.SetInputConnection(FilingCabinet2.GetOutputPort())
mapFilingCabinet2.ScalarVisibilityOff()
FilingCabinet2Actor = vtk.vtkActor()
FilingCabinet2Actor.SetMapper(mapFilingCabinet2)
FilingCabinet2Actor.GetProperty().SetColor(.8, .8, .6)

bookshelf1Top = vtk.vtkStructuredGridGeometryFilter()
bookshelf1Top.SetInputConnection(reader.GetOutputPort())
bookshelf1Top.SetExtent(13, 13, 0, 4, 0, 11)
mapBookshelf1Top = vtk.vtkPolyDataMapper()
mapBookshelf1Top.SetInputConnection(bookshelf1Top.GetOutputPort())
mapBookshelf1Top.ScalarVisibilityOff()
bookshelf1TopActor = vtk.vtkActor()
bookshelf1TopActor.SetMapper(mapBookshelf1Top)
bookshelf1TopActor.GetProperty().SetColor(.8, .8, .6)
bookshelf1Bottom = vtk.vtkStructuredGridGeometryFilter()
bookshelf1Bottom.SetInputConnection(reader.GetOutputPort())
bookshelf1Bottom.SetExtent(20, 20, 0, 4, 0, 11)
mapBookshelf1Bottom = vtk.vtkPolyDataMapper()
mapBookshelf1Bottom.SetInputConnection(bookshelf1Bottom.GetOutputPort())
mapBookshelf1Bottom.ScalarVisibilityOff()
bookshelf1BottomActor = vtk.vtkActor()
bookshelf1BottomActor.SetMapper(mapBookshelf1Bottom)
bookshelf1BottomActor.GetProperty().SetColor(.8, .8, .6)
bookshelf1Front = vtk.vtkStructuredGridGeometryFilter()
bookshelf1Front.SetInputConnection(reader.GetOutputPort())
bookshelf1Front.SetExtent(13, 20, 0, 0, 0, 11)
mapBookshelf1Front = vtk.vtkPolyDataMapper()
mapBookshelf1Front.SetInputConnection(bookshelf1Front.GetOutputPort())
mapBookshelf1Front.ScalarVisibilityOff()
bookshelf1FrontActor = vtk.vtkActor()
bookshelf1FrontActor.SetMapper(mapBookshelf1Front)
bookshelf1FrontActor.GetProperty().SetColor(.8, .8, .6)
bookshelf1Back = vtk.vtkStructuredGridGeometryFilter()
bookshelf1Back.SetInputConnection(reader.GetOutputPort())
bookshelf1Back.SetExtent(13, 20, 4, 4, 0, 11)
mapBookshelf1Back = vtk.vtkPolyDataMapper()
mapBookshelf1Back.SetInputConnection(bookshelf1Back.GetOutputPort())
mapBookshelf1Back.ScalarVisibilityOff()
bookshelf1BackActor = vtk.vtkActor()
bookshelf1BackActor.SetMapper(mapBookshelf1Back)
bookshelf1BackActor.GetProperty().SetColor(.8, .8, .6)
bookshelf1LHS = vtk.vtkStructuredGridGeometryFilter()
bookshelf1LHS.SetInputConnection(reader.GetOutputPort())
bookshelf1LHS.SetExtent(13, 20, 0, 4, 0, 0)
mapBookshelf1LHS = vtk.vtkPolyDataMapper()
mapBookshelf1LHS.SetInputConnection(bookshelf1LHS.GetOutputPort())
mapBookshelf1LHS.ScalarVisibilityOff()
bookshelf1LHSActor = vtk.vtkActor()
bookshelf1LHSActor.SetMapper(mapBookshelf1LHS)
bookshelf1LHSActor.GetProperty().SetColor(.8, .8, .6)
bookshelf1RHS = vtk.vtkStructuredGridGeometryFilter()
bookshelf1RHS.SetInputConnection(reader.GetOutputPort())
bookshelf1RHS.SetExtent(13, 20, 0, 4, 11, 11)
mapBookshelf1RHS = vtk.vtkPolyDataMapper()
mapBookshelf1RHS.SetInputConnection(bookshelf1RHS.GetOutputPort())
mapBookshelf1RHS.ScalarVisibilityOff()
bookshelf1RHSActor = vtk.vtkActor()
bookshelf1RHSActor.SetMapper(mapBookshelf1RHS)
bookshelf1RHSActor.GetProperty().SetColor(.8, .8, .6)

bookshelf2Top = vtk.vtkStructuredGridGeometryFilter()
bookshelf2Top.SetInputConnection(reader.GetOutputPort())
bookshelf2Top.SetExtent(13, 13, 15, 19, 0, 11)
mapBookshelf2Top = vtk.vtkPolyDataMapper()
mapBookshelf2Top.SetInputConnection(bookshelf2Top.GetOutputPort())
mapBookshelf2Top.ScalarVisibilityOff()
bookshelf2TopActor = vtk.vtkActor()
bookshelf2TopActor.SetMapper(mapBookshelf2Top)
bookshelf2TopActor.GetProperty().SetColor(.8, .8, .6)
bookshelf2Bottom = vtk.vtkStructuredGridGeometryFilter()
bookshelf2Bottom.SetInputConnection(reader.GetOutputPort())
bookshelf2Bottom.SetExtent(20, 20, 15, 19, 0, 11)
mapBookshelf2Bottom = vtk.vtkPolyDataMapper()
mapBookshelf2Bottom.SetInputConnection(bookshelf2Bottom.GetOutputPort())
mapBookshelf2Bottom.ScalarVisibilityOff()
bookshelf2BottomActor = vtk.vtkActor()
bookshelf2BottomActor.SetMapper(mapBookshelf2Bottom)
bookshelf2BottomActor.GetProperty().SetColor(.8, .8, .6)
bookshelf2Front = vtk.vtkStructuredGridGeometryFilter()
bookshelf2Front.SetInputConnection(reader.GetOutputPort())
bookshelf2Front.SetExtent(13, 20, 15, 15, 0, 11)
mapBookshelf2Front = vtk.vtkPolyDataMapper()
mapBookshelf2Front.SetInputConnection(bookshelf2Front.GetOutputPort())
mapBookshelf2Front.ScalarVisibilityOff()
bookshelf2FrontActor = vtk.vtkActor()
bookshelf2FrontActor.SetMapper(mapBookshelf2Front)
bookshelf2FrontActor.GetProperty().SetColor(.8, .8, .6)
bookshelf2Back = vtk.vtkStructuredGridGeometryFilter()
bookshelf2Back.SetInputConnection(reader.GetOutputPort())
bookshelf2Back.SetExtent(13, 20, 19, 19, 0, 11)
mapBookshelf2Back = vtk.vtkPolyDataMapper()
mapBookshelf2Back.SetInputConnection(bookshelf2Back.GetOutputPort())
mapBookshelf2Back.ScalarVisibilityOff()
bookshelf2BackActor = vtk.vtkActor()
bookshelf2BackActor.SetMapper(mapBookshelf2Back)
bookshelf2BackActor.GetProperty().SetColor(.8, .8, .6)
bookshelf2LHS = vtk.vtkStructuredGridGeometryFilter()
bookshelf2LHS.SetInputConnection(reader.GetOutputPort())
bookshelf2LHS.SetExtent(13, 20, 15, 19, 0, 0)
mapBookshelf2LHS = vtk.vtkPolyDataMapper()
mapBookshelf2LHS.SetInputConnection(bookshelf2LHS.GetOutputPort())
mapBookshelf2LHS.ScalarVisibilityOff()
bookshelf2LHSActor = vtk.vtkActor()
bookshelf2LHSActor.SetMapper(mapBookshelf2LHS)
bookshelf2LHSActor.GetProperty().SetColor(.8, .8, .6)
bookshelf2RHS = vtk.vtkStructuredGridGeometryFilter()
bookshelf2RHS.SetInputConnection(reader.GetOutputPort())
bookshelf2RHS.SetExtent(13, 20, 15, 19, 11, 11)
mapBookshelf2RHS = vtk.vtkPolyDataMapper()
mapBookshelf2RHS.SetInputConnection(bookshelf2RHS.GetOutputPort())
mapBookshelf2RHS.ScalarVisibilityOff()
bookshelf2RHSActor = vtk.vtkActor()
bookshelf2RHSActor.SetMapper(mapBookshelf2RHS)
bookshelf2RHSActor.GetProperty().SetColor(.8, .8, .6)

window = vtk.vtkStructuredGridGeometryFilter()
window.SetInputConnection(reader.GetOutputPort())
window.SetExtent(20, 20, 6, 13, 10, 13)
mapWindow = vtk.vtkPolyDataMapper()
mapWindow.SetInputConnection(window.GetOutputPort())
mapWindow.ScalarVisibilityOff()
windowActor = vtk.vtkActor()
windowActor.SetMapper(mapWindow)
windowActor.GetProperty().SetColor(.3, .3, .5)

outlet = vtk.vtkStructuredGridGeometryFilter()
outlet.SetInputConnection(reader.GetOutputPort())
outlet.SetExtent(0, 0, 9, 10, 14, 16)
mapOutlet = vtk.vtkPolyDataMapper()
mapOutlet.SetInputConnection(outlet.GetOutputPort())
mapOutlet.ScalarVisibilityOff()
outletActor = vtk.vtkActor()
outletActor.SetMapper(mapOutlet)
outletActor.GetProperty().SetColor(0, 0, 0)

inlet = vtk.vtkStructuredGridGeometryFilter()
inlet.SetInputConnection(reader.GetOutputPort())
inlet.SetExtent(0, 0, 9, 10, 0, 6)
mapInlet = vtk.vtkPolyDataMapper()
mapInlet.SetInputConnection(inlet.GetOutputPort())
mapInlet.ScalarVisibilityOff()
inletActor = vtk.vtkActor()
inletActor.SetMapper(mapInlet)
inletActor.GetProperty().SetColor(0, 0, 0)

outline = vtk.vtkStructuredGridOutlineFilter()
outline.SetInputConnection(reader.GetOutputPort())
mapOutline = vtk.vtkPolyDataMapper()
mapOutline.SetInputConnection(outline.GetOutputPort())
outlineActor = vtk.vtkActor()
outlineActor.SetMapper(mapOutline)
outlineActor.GetProperty().SetColor(0, 0, 0)

# Create source for streamtubes
streamer = vtk.vtkStreamPoints()
streamer.SetInputConnection(reader.GetOutputPort())
streamer.SetStartPosition(0.1, 2.1, 0.5)
streamer.SetMaximumPropagationTime(500)
streamer.SetTimeIncrement(0.5)
streamer.SetIntegrationDirectionToForward()

cone = vtk.vtkConeSource()
cone.SetResolution(8)
cones = vtk.vtkGlyph3D()
cones.SetInputConnection(streamer.GetOutputPort())
cones.SetSourceConnection(cone.GetOutputPort())
cones.SetScaleFactor(0.5)
cones.SetScaleModeToScaleByVector()

mapCones = vtk.vtkPolyDataMapper()
mapCones.SetInputConnection(cones.GetOutputPort())
mapCones.SetScalarRange(reader.GetOutput().GetScalarRange())

conesActor = vtk.vtkActor()
conesActor.SetMapper(mapCones)

ren1.AddActor(table1Actor)
ren1.AddActor(table2Actor)
ren1.AddActor(FilingCabinet1Actor)
ren1.AddActor(FilingCabinet2Actor)
ren1.AddActor(bookshelf1TopActor)
ren1.AddActor(bookshelf1BottomActor)
ren1.AddActor(bookshelf1FrontActor)
ren1.AddActor(bookshelf1BackActor)
ren1.AddActor(bookshelf1LHSActor)
ren1.AddActor(bookshelf1RHSActor)
ren1.AddActor(bookshelf2TopActor)
ren1.AddActor(bookshelf2BottomActor)
ren1.AddActor(bookshelf2FrontActor)
ren1.AddActor(bookshelf2BackActor)
ren1.AddActor(bookshelf2LHSActor)
ren1.AddActor(bookshelf2RHSActor)
ren1.AddActor(windowActor)
ren1.AddActor(outletActor)
ren1.AddActor(inletActor)
ren1.AddActor(outlineActor)
ren1.AddActor(conesActor)

ren1.SetBackground(0.4, 0.4, 0.5)

aCamera = vtk.vtkCamera()
aCamera.SetClippingRange(0.7724, 39)
aCamera.SetFocalPoint(1.14798, 3.08416, 2.47187)
aCamera.SetPosition(-2.64683, -3.55525, 3.55848)
aCamera.SetViewUp(0.0511273, 0.132773, 0.989827)
aCamera.SetViewAngle(15.5033)

ren1.SetActiveCamera(aCamera)

renWin.SetSize(500, 300)

iren.Initialize()
#iren.Start()
