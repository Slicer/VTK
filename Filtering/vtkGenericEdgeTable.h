/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkGenericEdgeTable.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkGenericEdgeTable - keep track of edges (defined by pair of integer id's)
// .SECTION Description
// vtkGenericEdgeTable is used to indicate the existance of and hold
// information about edges. Similar to vtkEdgeTable, this class is
// more sophisticated in that it uses reference counting to keep track
// of when information about an edge should be deleted.
//
// vtkGenericEdgeTable is a helper class used in the adaptor framework.  It
// is used during the tessellation process to hold information about the
// error metric on each edge. This avoids recomputing the error metric each
// time the same edge is visited.

#ifndef __vtkGenericEdgeTable_h
#define __vtkGenericEdgeTable_h

#include "vtkObject.h"

class vtkEdgeTableEdge;
class vtkEdgeTablePoints;

class VTK_FILTERING_EXPORT vtkGenericEdgeTable : public vtkObject
{
public:
  // Description:
  // Instantiate an empty edge table.
  static vtkGenericEdgeTable *New();
  
  // Description:
  // Standard VTK type and print macros.
  vtkTypeRevisionMacro(vtkGenericEdgeTable,vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Split the edge with the indicated point id.
  void InsertEdge(vtkIdType e1, vtkIdType e2, vtkIdType cellId, 
                  int ref, vtkIdType &ptId );

  // Description:
  // Split the edge with the indicated point id.
  void InsertEdge(vtkIdType e1, vtkIdType e2, vtkIdType cellId, 
                  int ref, int toSplit, vtkIdType &ptId );

  // Description:
  // Insert an edge but do not split it.
  void InsertEdge(vtkIdType e1, vtkIdType e2, vtkIdType cellId, int ref = 1 );

  // Description:
  // Method to remove an edge from the table. The method returns the
  // current reference count.
  int RemoveEdge(vtkIdType e1, vtkIdType e2);

  // Description:
  // Method to determine whether an edge is in the table.
  // It returns if the edge was split, and the point id exists.
  int CheckEdge(vtkIdType e1, vtkIdType e2, vtkIdType &ptId);
  
  // Description:
  // Method that increments the referencecount and returns it.
  int IncrementEdgeReferenceCount(vtkIdType e1, vtkIdType e2, 
                                  vtkIdType cellId);
  
  // Description:
  // Return the edge reference count.
  int CheckEdgeReferenceCount(vtkIdType e1, vtkIdType e2);

  // Description:
  // To specify the starting point id.
  void Initialize(vtkIdType start);
  
  // Description:
  // Return the last point id inserted.
  vtkIdType GetLastPointId();
  
  // Description:
  // Increment the last point id.
  void IncrementLastPointId();
  
  // Description:
  // Return the total number of components for the point-centered attributes.
  // \post positive_result: result>0
  int GetNumberOfComponents();
  
  // Description:
  // Set the total number of components for the point-centered attributes.
  // \pre positive_count: count>0
  void SetNumberOfComponents(int count);
  
  // Description:
  // Check if a point is already in the point table.
  int CheckPoint(vtkIdType ptId);

  // Description:
  // Check for the existence of a point and return its coordinate value.
  // \pre scalar_size: sizeof(scalar)==this->GetNumberOfComponents()
  int CheckPoint(vtkIdType ptId, double point[3], double *scalar);

  // Description:
  // Insert point associated with an edge.
  void InsertPoint(vtkIdType ptId, double point[3]);
  // \pre: sizeof(s)==GetNumberOfComponents()
  void InsertPointAndScalar(vtkIdType ptId, double pt[3], double *s);

  // Description:
  // Remove a point from the point table.
  void RemovePoint(vtkIdType ptId);
  
  // Description:
  // Increment the reference count for the indicated point.
  void IncrementPointReferenceCount(vtkIdType ptId );

//BTX
class PointEntry
{
public:
  vtkIdType PointId;
  double Coord[3];
  double *Scalar;  // point data: all point-centered attributes at this point
  int numberOfComponents;
  
  int Reference;  //signed char
  
  // Description:
  // Constructor with a scalar field of `size' doubles.
  // \pre positive_number_of_components: size>0
  PointEntry(int size);
  
  ~PointEntry()
    {
      delete[] this->Scalar;
    }

  PointEntry(const PointEntry &other)
    {
    this->PointId  = other.PointId;

    memcpy(this->Coord,other.Coord,sizeof(double)*3);

    int c=other.numberOfComponents;
    this->numberOfComponents=c;
    this->Scalar=new double[c];
    memcpy(this->Scalar,other.Scalar,sizeof(double)*c);
    this->Reference = other.Reference;
    }

  void operator=(const PointEntry &other) 
    {
    if(this != &other)
      {
      this->PointId  = other.PointId;
      
      memcpy(this->Coord,other.Coord,sizeof(double)*3);
      
      int c=other.numberOfComponents;
      
      if(this->numberOfComponents!=c)
        {
        delete[] this->Scalar;
        this->Scalar=new double[c];
        this->numberOfComponents=c;
        }
      memcpy(this->Scalar,other.Scalar,sizeof(double)*c);
      this->Reference = other.Reference;
      }
    }
};

class EdgeEntry
{
public:
  vtkIdType E1;
  vtkIdType E2;

  int Reference;  //signed char
  int ToSplit;  //signed char
  vtkIdType PtId;
  vtkIdType CellId; //CellId the edge refer to at a step in tesselation
  
  EdgeEntry() 
    { 
    this->Reference = 0; 
    this->CellId = -1;
    }
  ~EdgeEntry() {}

  EdgeEntry(const EdgeEntry& copy)
    {
    this->E1 = copy.E1;
    this->E2 = copy.E2;

    this->Reference = copy.Reference;
    this->ToSplit = copy.ToSplit;
    this->PtId = copy.PtId;
    this->CellId = copy.CellId;
    }
  
  void operator=(const EdgeEntry& entry) 
    {
    if(this == &entry)
      {
      return;
      }
    this->E1 = entry.E1;
    this->E2 = entry.E2;
    this->Reference = entry.Reference;
    this->ToSplit = entry.ToSplit;
    this->PtId = entry.PtId;
    this->CellId = entry.CellId;
    }
};
//ETX

protected:
  vtkGenericEdgeTable();
  ~vtkGenericEdgeTable();

  // Description:
  // For debuggin purposes.
  void DumpTable();
  void LoadFactor();
  
  //Hash table that contiain entry based on edges:
  vtkEdgeTableEdge   *EdgeTable;

  //At end of process we should be able to retrieve points coord based on pointid
  vtkEdgeTablePoints *HashPoints;

  // Main hash functions
  //For edge table:
  vtkIdType HashFunction(vtkIdType e1, vtkIdType e2);

  //For point table:
  vtkIdType HashFunction(vtkIdType ptId);

  // Keep track of the last point id we inserted, increment it each time:
  vtkIdType LastPointId;  //static
//  vtkGetMacro(LastPointId, vtkIdType);
  //Use only once !
//  vtkSetMacro(LastPointId, vtkIdType);

  vtkIdType NumberOfComponents;
  
private:
  vtkGenericEdgeTable(const vtkGenericEdgeTable&);  // Not implemented.
  void operator=(const vtkGenericEdgeTable&);  // Not implemented.

};

#endif

