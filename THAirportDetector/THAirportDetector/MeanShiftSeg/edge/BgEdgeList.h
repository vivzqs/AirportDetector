/////////////////////////////////////////////////////////////////////////////

// Name:        BgEdgeList.h

// Purpose:     BgEdgeList class

// Author:      Bogdan Georgescu

// Modified by:

// Created:     06/22/2000

// Copyright:   (c) Bogdan Georgescu

// Version:     v0.1

/////////////////////////////////////////////////////////////////////////////



class BgEdgeList

{
public:
	BgEdgeList();

	~BgEdgeList();

   int nEdges_;

   BgEdge* edgelist_;

   BgEdge* crtedge_;

   void FreeEdgeList();

   void AddEdge(float*, int);

   void AddEdge(int*, int nPoints);

   void RemoveShortEdges(int);

   void SetBinImage(BgImage*);

   bool SaveEdgeList(char*);

   void SetGradient(float*, float*, float*, int);

   void SetNoMark(void);

   void GetAllEdgePoints(int*, int*, int*);

   //shenzf 2009.12.18
   void GetAllEdgePoints(int *nLineNum,int **pLinePoint,int *point_x,int *point_y);
};

