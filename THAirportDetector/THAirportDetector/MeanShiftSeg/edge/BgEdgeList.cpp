/////////////////////////////////////////////////////////////////////////////

// Name:        BgEdgeList.cpp

// Purpose:     BgEdgeList class functions

// Author:      Bogdan Georgescu

// Modified by:

// Created:     06/22/2000

// Copyright:   (c) Bogdan Georgescu

// Version:     v0.1

/////////////////////////////////////////////////////////////////////////////


#include "stdafx.h"

#include <stdio.h>

#include <stdlib.h>

#include <math.h>

#include "BgDefaults.h"

#include "BgImage.h"

#include "BgEdge.h"

#include "BgEdgeList.h"



BgEdgeList::BgEdgeList()

{
   nEdges_ = 0;

   edgelist_ = 0;

   crtedge_ = 0;
}



BgEdgeList::~BgEdgeList()

{
	FreeEdgeList();
}


void BgEdgeList::FreeEdgeList()
{
	if (nEdges_>0)
	{
		BgEdge* edge;
		for (int i=0; i<nEdges_; i++)
		{
			edge = edgelist_->next_;
			delete edgelist_;
			edgelist_=edge;
		}
	}

	 edgelist_ = 0;

	return;
}


void BgEdgeList::AddEdge(float* edge, int nPoints)

{
   BgEdge* tedge;

   tedge = new BgEdge();

   tedge->SetPoints(edge, nPoints);

   if (nEdges_==0)

   {
      nEdges_ = 1;

      edgelist_ = tedge;

      crtedge_ = tedge;
   }

   else

   {

      nEdges_++;

      crtedge_->next_ = tedge;

      crtedge_ = tedge;

   }

}



void BgEdgeList::AddEdge(int* edge, int nPoints)

{

   BgEdge* tedge;

   tedge = new BgEdge();

   tedge->SetPoints(edge, nPoints);

   if (nEdges_==0)

   {

      nEdges_ = 1;

      edgelist_ = tedge;

      crtedge_ = tedge;

   }

   else

   {

      nEdges_++;

      crtedge_->next_ = tedge;

      crtedge_ = tedge;

   }

}



void BgEdgeList::SetGradient(float* grx, float* gry, float* mark, int ncol)

{
   BgEdge* it;

   int i;

   it = edgelist_;

   for (i=0; i<nEdges_; i++)
   {
      it->SetGradient(grx, gry, mark, ncol);

      it = it->next_;
   }

   return;
}



void BgEdgeList::RemoveShortEdges(int minp)

{
   if (nEdges_==0)
      return;

   int nEdges=nEdges_;

   BgEdge* it1;

   BgEdge* it2;

   it1 = edgelist_;

   it2 = it1->next_;

   for (int i=1; i<nEdges_; i++)
   {
      if (it2->nPoints_ < minp)
      {
         it1->next_ = it2->next_;

         delete it2;

         it2 = it1->next_;

         nEdges--;

      }
      else
      {
         it1 = it2;

         it2 = it1->next_;
      }
   }

   if (edgelist_->nPoints_ < minp)
   {

      it1 = edgelist_;

      edgelist_ = edgelist_->next_;

      delete it1;

      nEdges--;

   }

   nEdges_=nEdges;

   return;
}



void BgEdgeList::SetBinImage(BgImage* image)
{

   int i, j;

   int ix, iy;

   int x, y;

   

   x = image->x_;

   y = image->y_;

   unsigned char* im=image->im_;

   

   for (i=0; i<x; i++)

   {

      for (j=0;j<y;j++)

      {

         *(im++) = 0;

      }

   }

   

   im = image->im_;

   int* ite;

   crtedge_=edgelist_;

   for (i=0; i<nEdges_; i++)

   {

      ite = crtedge_->edge_;

      for (j=0; j<crtedge_->nPoints_; j++)

      {

         ix = *(ite++);

         iy = *(ite++);

         *(im+iy*x+ix) = 255;

      }

      crtedge_=crtedge_->next_;

   }

}



bool BgEdgeList::SaveEdgeList(char* edgeFile)

{

   int length;

   int i,j;

   BgEdge *crtedge;

   

   FILE* fp;

   fopen_s(&fp, edgeFile,"wb");

   crtedge = edgelist_;

   for (i=0; i<nEdges_; i++)

   {

      length = crtedge->nPoints_;

      for (j=0; j<length; j++)

      {

         fprintf(fp, "%d %d %d\n", *((crtedge->edge_)+2*j), *((crtedge->edge_)+2*j+1), i);

      }

      crtedge = crtedge->next_;

   }

   fclose(fp);

   return true;

}



void BgEdgeList::GetAllEdgePoints(int* x, int* y, int* n)

{

   int length;

   int i,j;

   BgEdge *crtedge;

   int *edgep;

   

   crtedge = edgelist_;

   *n = 0;

   for (i=0; i<nEdges_; i++)

   {

      length = crtedge->nPoints_;

      edgep = crtedge->edge_;

      for (j=0; j<length; j++)

      {

         x[*n] = edgep[2*j];

         y[*n] = edgep[2*j+1];

         (*n)++;

      }

      crtedge = crtedge->next_;

   }

}

void BgEdgeList::GetAllEdgePoints(int *nLineNum,int **pLinePoint,int *point_x,int *point_y)

{
	*nLineNum = nEdges_;

	if(*pLinePoint)
	{
		delete *pLinePoint;
		*pLinePoint = NULL;
	}

	*pLinePoint = new int [*nLineNum];

	int length;

	int i,j;

	BgEdge *crtedge;

	int *edgep;

	crtedge = edgelist_;

	int numTMP = 0;

	for (i=0; i<nEdges_; i++)
	{
		length = crtedge->nPoints_;

		(*pLinePoint)[i] = length;

		edgep = crtedge->edge_;

		for (j=0; j<length; j++)

		{
			point_x[numTMP] = edgep[2*j];

			point_y[numTMP] = edgep[2*j+1];

			numTMP++;

		}

		crtedge = crtedge->next_;
	}

	return;
}


void BgEdgeList::SetNoMark(void)

{

   int length;

   int i,j;

   BgEdge* crtedge;

   unsigned char* mark;

   crtedge = edgelist_;

   for (i=0; i<nEdges_; i++)

   {

      length = crtedge->nPoints_;

      mark = crtedge->mark_ = new unsigned char[length];

      crtedge->isMarkSet_ = true;

      for (j=0; j<length; j++)

      {

         *(mark+j) = 0;

      }

      crtedge = crtedge->next_;

   }

}

