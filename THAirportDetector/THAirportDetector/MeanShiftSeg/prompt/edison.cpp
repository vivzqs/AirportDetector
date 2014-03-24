////////////////////////////////////////////////////////
// Name     : edison.cpp
// Purpose  : Wrapper class used for segmenation and
//            edge detection.
// Author   : Chris M. Christoudias
// Modified by
// Created  : 03/20/2002
// Copyright: (c) Chris M. Christoudias
// Version  : v0.1
////////////////////////////////////////////////////////

#include "stdafx.h"
#include "defs.h"
#include "flags.h"
#include "edison.h"
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "io.h"
#include "crtdbg.h"
#include	<assert.h>
#include	<stdlib.h>

#define TRUE 1
#define FALSE 0

////////////////////////////////////////////////////////
//Constructor/Destructor
////////////////////////////////////////////////////////

EDISON::EDISON( void )
{
	height_            = -1;
	width_             = -1;
	numEdges_          = -1;
	numBoundaries_     = -1;
	inputImage_        = (unsigned char *) NULL;
	filtImage_         = (unsigned char *) NULL;
	segmImage_         = (unsigned char *) NULL;
	gradMap_           = (float *) NULL;
	confMap_           = (float *) NULL;
	weightMap_         = (float *) NULL;
	custMap_           = (float *) NULL;
	edges_             = (int *)   NULL;
	boundaries_        = (int *)   NULL;
	parameterList_     = (void **) NULL;
	inputDefined_      = false;
	gradMapDefined_    = false;
	confMapDefined_    = false;
	weightMapDefined_  = false;
	custMapDefined_    = false;
	filtImageDefined_  = false;
	segmImageDefined_  = false;
	edgesDefined_      = false;
	boundariesDefined_ = false;
	piProc			   = NULL;	
	m_bHaveSaved	   = false;

	m_pEdgeList		   = NULL;

	m_wait_begin       = 0;
	m_wait_end         = 100;

	old_img_size       = -1;
}

EDISON::~EDISON( void )
{
	//shenzf 2009.11.20 在SegByMS.cpp中释放内存
	//if(inputImage_) {delete [] inputImage_;inputImage_ = NULL;}
	if(filtImage_)  {delete [] filtImage_;filtImage_ = NULL;}
	if(segmImage_)  {delete [] segmImage_;segmImage_ = NULL;}
	if(gradMap_)    {delete [] gradMap_;gradMap_ = NULL;}
	if(confMap_)    {delete [] confMap_;confMap_ = NULL;}
	if(weightMap_)  {delete [] weightMap_;weightMap_ = NULL;}
	if(custMap_)    {delete [] custMap_;custMap_ = NULL;}
	if(edges_)      {delete [] edges_;edges_ = NULL;}
	if(boundaries_) {delete [] boundaries_;boundaries_ = NULL;}
	if(piProc)		{delete piProc;piProc = NULL;}
	if(m_pEdgeList) {delete m_pEdgeList;m_pEdgeList = NULL;}
}


////////////////////////////////////////////////////////
//Input/Output
////////////////////////////////////////////////////////

int EDISON::Save(char *filename, int filetype, int outputtype)
{
	if(!filename) return EXE_NULL_PTR;
	int error;
	switch(outputtype)
	{
	case OUTPUT_SEGM_BOUNDARIES:
		if(!boundariesDefined_) return EXE_OUTPUT_UNDEFINED;
		error = writeImage(filename, (unsigned char *) NULL, boundaries_, numBoundaries_, filetype);
		break;
	case OUTPUT_SEGM_IMAGE:
		if(!segmImageDefined_) return EXE_OUTPUT_UNDEFINED;
		error = writeImage(filename, segmImage_, (int *) NULL, -1, filetype);
		break;
	case OUTPUT_SEGM_IMAGE_BOUNDARIES:
		if(!(boundariesDefined_ && segmImageDefined_)) return EXE_OUTPUT_UNDEFINED;
		error = writeImage(filename, segmImage_, boundaries_, numBoundaries_, filetype);
		break;
	case OUTPUT_FILT_IMAGE:
		if(!filtImageDefined_) return EXE_OUTPUT_UNDEFINED;
		error = writeImage(filename, filtImage_, (int *) NULL, -1, filetype);
		break;
	case OUTPUT_FILT_IMAGE_BOUNDARIES:
		if(!(boundariesDefined_ && filtImageDefined_)) return EXE_OUTPUT_UNDEFINED;      
		error = writeImage(filename, filtImage_, boundaries_, numBoundaries_, filetype);
		break;
	case OUTPUT_GRADIENT_MAP:
		if(!gradMapDefined_) return EXE_OUTPUT_UNDEFINED;
		error = saveData(filename, gradMap_, filetype);
		break;
	case OUTPUT_CONFIDENCE_MAP:
		if(!confMapDefined_) return EXE_OUTPUT_UNDEFINED;
		error = saveData(filename, confMap_, filetype);
		break;
	case OUTPUT_WEIGHT_MAP:
		if(!weightMapDefined_) return EXE_OUTPUT_UNDEFINED;
		error = saveData(filename, weightMap_, filetype);
		break;
	case OUTPUT_EDGES:
		if(!edgesDefined_) return EXE_OUTPUT_UNDEFINED;
		error = writeImage(filename, (unsigned char *) NULL, edges_, numEdges_, filetype);
		break;
	}
	return error;
}

int EDISON::Load(char *filename, int inputtype)
{
	if(!filename) return EXE_NULL_PTR;
	int error;
	switch(inputtype)
	{
	case INPUT_IMAGE:
		error = loadImage(filename);
		if(!error) Refresh();
		break;
	case INPUT_MAP:
		error = loadMap(filename);
		break;
	}
	return error;
}

int EDISON::UseResult(int outputtype)
{
	switch(outputtype)
	{
	case OUTPUT_SEGM_IMAGE:
		if(!segmImageDefined_) return EXE_OUTPUT_UNDEFINED;
		memcpy(inputImage_, segmImage_, height_*width_*dim_*sizeof(unsigned char));
		break;
	case OUTPUT_FILT_IMAGE:
		if(!filtImageDefined_) return EXE_OUTPUT_UNDEFINED;
		memcpy(inputImage_, filtImage_, height_*width_*dim_*sizeof(unsigned char));
		break;
	default:
		break;
	}
	Refresh();
	return NO_ERRORS;
}

int EDISON::writeImage(char *filename, unsigned char *image, int *dataPoints, int n, int filetype)
{
	unsigned char *data = new unsigned char [height_ * width_ * dim_];
	if(!data) return EXE_OUT_OF_MEMORY;
	memset(data, 0, height_*width_*dim_*sizeof(unsigned char));
	if(image) memcpy(data, image, height_*width_*dim_*sizeof(unsigned char));
	if(dataPoints) 
	{
		for(int i = 0; i < n; i++) 
		{
			for(int j = 0; j < dim_; j++) 
			{
				data[dim_*dataPoints[i]+j] = 255;
			}
		}
	}
	int error = CmCWriteImage(filename, data, height_, width_, dim_, filetype);
	delete [] data;
	return error;
}

int EDISON::saveData(char *filename, float *data, int filetype)
{
	if(filetype != FILE_MATLAB_ASCII) {
		unsigned char *imData = new unsigned char[height_*width_];
		if(!imData) return EXE_OUT_OF_MEMORY;
		int i;
		for(i = 0; i < height_*width_; i++) {
			imData[i] = (int)(255*data[i] + 0.5);
		}
		int error = CmCWriteImage(filename, imData, height_, width_, 1, filetype);
		return error;
	}
	int error = CmCWriteMFile(filename, data, height_, width_, dim_);
	return error;
}

int EDISON::loadImage(char *filename)
{  
	int error = CmCReadImage(filename, &inputImage_, height_, width_, dim_);
	if(!error) inputDefined_ = true;
	return error;
}

int EDISON::loadMap(char *filename)
{
	if(!inputDefined_) return EXE_INPUT_UNDEFINED;
	int error = CmCReadMFile(filename, &custMap_, height_, width_);
	if(!error) custMapDefined_ = true;
	return error;
}   

void EDISON::Refresh( void )
{
	//de-allocate all memory
	if(filtImage_)  delete [] filtImage_;
	if(segmImage_)  delete [] segmImage_;
	if(gradMap_)    delete [] gradMap_;
	if(confMap_)    delete [] confMap_;
	if(weightMap_)  delete [] weightMap_;
	if(custMap_)    delete [] custMap_;
	if(edges_)      delete [] edges_;
	if(boundaries_) delete [] boundaries_;boundaries_ = NULL;

	//reset flags
	gradMapDefined_    = false;
	confMapDefined_    = false;
	weightMapDefined_  = false;
	custMapDefined_    = false;
	filtImageDefined_  = false;
	segmImageDefined_  = false;
	edgesDefined_      = false;
	boundariesDefined_ = false;
}

////////////////////////////////////////////////////////
//Set Parameters
////////////////////////////////////////////////////////

void EDISON::SetParameters(void **parameterList)
{
	parameterList_ = parameterList;
}

////////////////////////////////////////////////////////
//Edge Detection
////////////////////////////////////////////////////////

int EDISON::EdgeDetect( void )
{

	//make sure an input image was defined
	if(!inputDefined_) return EXE_INPUT_UNDEFINED;

	//prompt the user
	CmCPrompt("\n-------------------------------------------------------------------------\n");
	CmCPrompt("Performing EDGE DETECTION:\n\n");
	CmCPrompt("\tUsing Parameters:\n\n");  

	//make sure all necessary parameters are available
	int i;
	if(!parameterList_[PARAM_GRADIENT_WINDOW_RADIUS]) return EXE_MISSING_PARAM;
	for(i = 7; i < PARAM_NUM; i++) {
		if(!parameterList_[i]) return EXE_MISSING_PARAM;
	}

	//obtain parameters
	int gradWindRad     = *((int *)(parameterList_[PARAM_GRADIENT_WINDOW_RADIUS]));
	int minLength       = *((int *)(parameterList_[PARAM_MINIMUM_LENGTH]));
	float nmxRank       = *((float *)(parameterList_[PARAM_NMX_RANK]));
	float nmxConf       = *((float *)(parameterList_[PARAM_NMX_CONF]));
	int   nmxType       = *((int *)(parameterList_[PARAM_NMX_TYPE]));
	float hystHighRank  = *((float *)(parameterList_[PARAM_HYSTERISIS_HIGH_RANK]));
	float hystHighConf  = *((float *)(parameterList_[PARAM_HYSTERISIS_HIGH_CONF]));
	int   hystHighType  = *((int *)(parameterList_[PARAM_HYSTERISIS_HIGH_TYPE]));
	float hystLowRank   = *((float *)(parameterList_[PARAM_HYSTERISIS_LOW_RANK]));
	float hystLowConf   = *((float *)(parameterList_[PARAM_HYSTERISIS_LOW_CONF]));
	int   hystLowType   = *((int *)(parameterList_[PARAM_HYSTERISIS_LOW_TYPE]));

	//prompt the user
	CmCPrompt("\t\tGradient Window Radius\t= %6d\n\t\tMinimum Length\t\t= %6d\n\n", gradWindRad, minLength);
	CmCPrompt("\t\tNmx. Rank\t\t= %6.4f\n\t\tNmx. Conf\t\t= %6.4f\n\t\tNmx. Curve Type\t\t= %s\n\n",
		nmxRank, nmxConf, CURVETYPE_LIST[nmxType]);
	CmCPrompt("\t\tHyst. High Rank\t\t= %6.4f\n\t\tHyst. High Conf\t\t= %6.4f\n\t\tHyst. High Curve Type\t= %s",
		hystHighRank, hystHighConf, CURVETYPE_LIST[hystHighType]);
	CmCPrompt("\n\n");
	CmCPrompt("\t\tHyst. Low Rank\t\t= %6.4f\n\t\tHyst. Low Conf\t\t= %6.4f\n\t\tHyst. Low Curve Type\t= %s",
		hystLowRank, hystLowConf, CURVETYPE_LIST[hystLowType]);
	CmCPrompt("\n\n");  

	//convert the input image to grayscale if necessary...
	BgImage inputImage;
	if(dim_ == 3) {
		unsigned char *data = CmCConvertToGrayscale(inputImage_, height_, width_);
		inputImage.SetImage(data, width_, height_, false);
		delete [] data;
	} else
		inputImage.SetImage(inputImage_, width_, height_, false);

	//if there are any custom curves attempt to read in there point lists
	int   n;
	float *pts;
	double *ptsX, *ptsY;
	BgEdgeDetect edgeDetector(gradWindRad);
	if(hystHighType == CURVE_CUSTOM) {
		pts = (float *)(parameterList_[PARAM_NUM + 2*CUST_CURVE_HYST_HIGH]);
		if(!pts) return EXE_POINT_LIST_HIGH;
		n = *((int *)(parameterList_[PARAM_NUM + 2*CUST_CURVE_HYST_HIGH + 1]));
		ptsX = new double [n];
		ptsY = new double [n];
		for(i = 0; i < n; i++) {
			ptsX[i] = (double) pts[2*i];
			ptsY[i] = (double) pts[2*i+1];
		}
		edgeDetector.SetCustomHigh(ptsX, ptsY, n);
		delete [] ptsX;
		delete [] ptsY;
	}
	if(hystLowType == CURVE_CUSTOM) {
		pts = (float *)(parameterList_[PARAM_NUM + 2*CUST_CURVE_HYST_LOW]);
		if(!pts) return EXE_POINT_LIST_LOW;
		n = *((int *)(parameterList_[PARAM_NUM + 2*CUST_CURVE_HYST_LOW + 1]));
		ptsX = new double [n];
		ptsY = new double [n];
		for(i = 0; i < n; i++) {
			ptsX[i] = (double) pts[2*i];
			ptsY[i] = (double) pts[2*i+1];
		}
		edgeDetector.SetCustomLow(ptsX, ptsY, n);
		delete [] ptsX;
		delete [] ptsY;
	}

	//edge detect the input image
	BgEdgeList   edgeList;
	edgeDetector.DoEdgeDetect(&inputImage, &edgeList, nmxRank, nmxConf, hystHighRank, hystHighConf,
		hystLowRank, hystLowConf, minLength, nmxType, hystHighType, hystLowType);  
	if(edgesDefined_) delete [] edges_;
	edges_ = new int [height_ * width_];
	int *edgex, *edgey;
	edgex = new int [height_ * width_];
	edgey = new int [height_ * width_];
	edgeList.GetAllEdgePoints(edgex, edgey, &numEdges_);
	for(i = 0; i < numEdges_; i++) {
		edges_[i] = edgey[i]*width_ + edgex[i];
	}
	edgesDefined_ = true;
	delete [] edgex;
	delete [] edgey;

	//obtain rank and confidence maps
	if(gradMapDefined_) delete [] gradMap_;
	if(confMapDefined_) delete [] confMap_;
	gradMap_ = new float [height_ * width_];
	confMap_ = new float [height_ * width_];
	if(!gradMap_ || !confMap_) return EXE_OUT_OF_MEMORY;
	memcpy(gradMap_, edgeDetector.permRank_, height_ * width_ * sizeof(float));
	memcpy(confMap_, edgeDetector.permConf_, height_ * width_ * sizeof(float));
	gradMapDefined_ = true;
	confMapDefined_ = true;

	//prompt the user
	CmCPrompt("-------------------------------------------------------------------------\n");

	//done.
	return NO_ERRORS;

} 


BOOL EDISON::DoEdgeDetect(unsigned char* inBIPImageData,
						  int DIM,
						  int WIDTH,
						  int HEIGHT,
						  int GRAD_WIN/* = 2*/, 
						  int MIN_LENGTH /*= 5*/,
						  int NON_SUP_TYPE /*= 0*/, 
						  float NON_SUP_RANK /*= 0.5*/, 
						  float NON_SUP_CONF /*= 0.5*/,
						  int HIGH_TYPE /*= 0*/, 
						  float HIGH_RANK /*= 0.93*/, 
						  float HIGH_CONF /*= 0.96*/,
						  int LOW_TYPE /*= 0*/, 
						  float LOW_RANK /*= 0.99*/, 
						  float LOW_CONF /*= 0.91*/)
{
	inputImage_ = inBIPImageData;
	dim_			= DIM;
	width_			= WIDTH;
	height_			= HEIGHT;

	int gradWindRad     = GRAD_WIN;
	int minLength       = MIN_LENGTH;
	float nmxRank       = NON_SUP_RANK;
	float nmxConf       = NON_SUP_CONF;
	int   nmxType       = NON_SUP_TYPE;
	float hystHighRank  = HIGH_RANK;
	float hystHighConf  = HIGH_CONF;
	int   hystHighType  = HIGH_TYPE;
	float hystLowRank   = LOW_RANK;
	float hystLowConf   = LOW_CONF;
	int   hystLowType   = LOW_TYPE;


	//convert the input image to grayscale if necessary...
	BgImage inputImage;
	if(dim_ == 3) 
	{
		unsigned char *data = CmCConvertToGrayscale(inputImage_, height_, width_);
		inputImage.SetImage(data, width_, height_, false);
		delete [] data;
	} 
	else
		inputImage.SetImage(inputImage_, width_, height_, false);

	//if there are any custom curves attempt to read in there point lists
	//int   n;
	BgEdgeDetect edgeDetector(gradWindRad);

	//edge detect the input image
	if(m_pEdgeList)
	{
		delete m_pEdgeList;
		m_pEdgeList = NULL;
	}
	
	m_pEdgeList = new BgEdgeList;
	
	//BgEdgeList   edgeList;
	edgeDetector.DoEdgeDetect(&inputImage, 
							  m_pEdgeList, 
							  nmxRank, 
							  nmxConf, 
							  hystHighRank, 
							  hystHighConf,
							  hystLowRank, 
							  hystLowConf, 
							  minLength, 
							  nmxType, 
							  hystHighType, 
							  hystLowType);  

	return TRUE;
}
BOOL EDISON::GetEdgeDetectResults(int **x,int **y, int* nNum)
{
	//if(edgesDefined_) delete [] edges_;
	//edges_ = new int [height_ * width_];
	//int *edgex, *edgey;
	if(*x){delete *x;*x = NULL;}
	if(*y){delete *y;*y = NULL;}
	*x = new int [height_ * width_];
	*y = new int [height_ * width_];
	m_pEdgeList->GetAllEdgePoints(*x, *y, nNum);

	//FILE*ff;
	//fopen_s(&ff,"d:\\ff1.txt","w");
	//fprintf(ff,"%d\n",numEdges_);
	//for (int i=0;i<numEdges_;i++)
	//{
	//	fprintf(ff,"%d,%d\n",edgex[i],edgey[i]);
	//}
	//fclose(ff);
	
	//for(int i = 0; i < numEdges_; i++) {
	//	edges_[i] = edgey[i]*width_ + edgex[i];
	//}
	//edgesDefined_ = true;
	//delete [] edgex;
	//delete [] edgey;

	return TRUE;
}
BOOL EDISON::GetEdgeDetectResults(int *nLineNum,int **pLinePoint,int **point_x,int **point_y)
{
	if(*point_x){delete *point_x;*point_x = NULL;}
	if(*point_y){delete *point_y;*point_y = NULL;}
	*point_x = new int [height_ * width_];
	*point_y = new int [height_ * width_];

	m_pEdgeList->GetAllEdgePoints(nLineNum,pLinePoint,*point_x,*point_y);

	return TRUE;
}

////////////////////////////////////////////////////////
//Image Segmenation
////////////////////////////////////////////////////////

int EDISON::Filter( void )
{
	return meanShift(EDISON_FILTER);
}

int EDISON::Fuse( void )
{
	return meanShift(EDISON_FUSE);
}

int EDISON::Segment( void )
{
	return meanShift(EDISON_SEGMENT);
}

int EDISON::meanShift(int action)
{

	//make sure input has been defined
	if(!inputDefined_) return EXE_INPUT_UNDEFINED;

	//obtain parameters...
	int   sigmaS, minRegion;
	float sigmaR;
	char action_str[80];
	action_str[0] = 0; //initialize string
	if(CmCSynergistic) strcpy_s(action_str, 80, "SYNERGISTIC ");
	if(!parameterList_[PARAM_SPEEDUP]) return EXE_MISSING_PARAM;
	switch(action) 
	{
	case EDISON_FILTER:
		strcat_s(action_str,80, "IMAGE FILTERING");
		if(parameterList_[PARAM_SPATIAL_BANDWIDTH]) 
		{
			sigmaS = *((int *)(parameterList_[PARAM_SPATIAL_BANDWIDTH]));
			if(parameterList_[PARAM_RANGE_BANDWIDTH]) 
			{
				sigmaR = *((float *)(parameterList_[PARAM_RANGE_BANDWIDTH]));
				break;
			}
		}
		return EXE_MISSING_PARAM;
	case EDISON_FUSE:
		strcat_s(action_str,80, "IMAGE REGION FUSION");
		if(parameterList_[PARAM_RANGE_BANDWIDTH]) 
		{
			sigmaR = *((float *)(parameterList_[PARAM_RANGE_BANDWIDTH]));
			if(parameterList_[PARAM_MINIMUM_REGION_AREA]) 
			{
				minRegion = *((int *)(parameterList_[PARAM_MINIMUM_REGION_AREA]));
				break;
			}
		}
		return EXE_MISSING_PARAM;
	case EDISON_SEGMENT:
		strcat_s(action_str,80, "IMAGE SEGMENTATION");
		if(parameterList_[PARAM_SPATIAL_BANDWIDTH]) 
		{
			sigmaS = *((int *)(parameterList_[PARAM_SPATIAL_BANDWIDTH]));
			if(parameterList_[PARAM_RANGE_BANDWIDTH]) 
			{
				sigmaR = *((float *)(parameterList_[PARAM_RANGE_BANDWIDTH]));
				if(parameterList_[PARAM_MINIMUM_REGION_AREA]) 
				{
					minRegion = *((int *)(parameterList_[PARAM_MINIMUM_REGION_AREA]));
					break;
				}
			}
		}
		return EXE_MISSING_PARAM;
	}

	//check for synergistic parameters
	if(CmCSynergistic) {
		int i;
		for(i = PARAM_GRADIENT_WINDOW_RADIUS; i <= PARAM_EDGE_STRENGTH_THRESHOLD; i++) {
			if(!parameterList_[i]) return EXE_MISSING_PARAM;
		}
	}

	//prompt the user
	CmCPrompt("\n-------------------------------------------------------------------------\n");
	CmCPrompt("Performing %s:\n\n", action_str);
	CmCPrompt("\tUsing Parameters:\n\n");
	CmCPrompt("\t\tSpatial Bandwidth\t= %6d\n\t\tRange Bandwidth\t\t= %6.4f\n\t\tMinimum Region Area\t= %6d",
		sigmaS, sigmaR, minRegion);
	if(CmCSynergistic) 
	{
		int   gradWindRad = *((int *)(parameterList_[PARAM_GRADIENT_WINDOW_RADIUS]));
		float threshold   = *((float *)(parameterList_[PARAM_EDGE_STRENGTH_THRESHOLD]));
		float mixture     = *((float *)(parameterList_[PARAM_MIXTURE_PARAMETER]));
		CmCPrompt("\n\n\t\tGradient Window Radius\t= %6d\n\t\tEdge Strength Threshold\t= %6.4f\n\t\t",
			gradWindRad, threshold);
		CmCPrompt("Mixture Parameter\t= %6.4f\n\n", mixture);
	}

	//create image processing object
	msImageProcessor iProc(m_bWaitBar);

	//define the image to be processed as the input image
	if(dim_ == 3)
		iProc.DefineImage(fpSaveFile, inputImage_, COLOR, height_, width_);
	else
		iProc.DefineImage(fpSaveFile, inputImage_, GRAYSCALE, height_, width_);
	if(iProc.ErrorStatus) {
		return EXE_ERROR;
	}

	//compute and set weight map if synergistic segmentation is requested
	if(CmCSynergistic) 
	{
		if(CmCUseCustomWeightMap) 
		{
			if(!custMapDefined_) return EXE_INPUT_UNDEFINED;
			iProc.SetWeightMap(custMap_, *((float *)(parameterList_[PARAM_EDGE_STRENGTH_THRESHOLD])));
		} else {    
			int error = ComputeWeightMap();
			if(error) return error;
			iProc.SetWeightMap(weightMap_, *((float *)(parameterList_[PARAM_EDGE_STRENGTH_THRESHOLD])));
		}
		if(iProc.ErrorStatus) 
		{
			return EXE_ERROR;
		}
	}

	switch(action) {
  case EDISON_FILTER:
	  //filter the input image
	  iProc.Filter(sigmaS, sigmaR, (SpeedUpLevel)(*(int *)(parameterList_[PARAM_SPEEDUP])));
	  if(iProc.ErrorStatus) {
		  return EXE_ERROR;
	  }

	  //obtain the output
	  if(filtImageDefined_) delete [] filtImage_;
	  filtImage_ = new unsigned char [height_ * width_ * dim_];
	  iProc.GetResults(filtImage_);
	  if(iProc.ErrorStatus) {
		  return EXE_ERROR;
	  }
	  filtImageDefined_ = true;
	  break;
  case EDISON_FUSE:
	  //re-define input image if filtered image is defined
	  if(filtImageDefined_) {
		  if(dim_ == 3)
			  iProc.DefineImage(fpSaveFile, filtImage_, COLOR, height_, width_);
		  else
			  iProc.DefineImage(fpSaveFile, filtImage_, GRAYSCALE, height_, width_);
	  } 

	  //fuse the regions of input image
	  iProc.FuseRegions(sigmaR, minRegion);
	  if(iProc.ErrorStatus) {
		  return EXE_ERROR;
	  }

	  //obtain the output
	  if(segmImageDefined_) delete [] segmImage_;
	  segmImage_ = new unsigned char [height_ * width_ * dim_];
	  iProc.GetResults(segmImage_);
	  if(iProc.ErrorStatus) {
		  return EXE_ERROR;
	  }
	  segmImageDefined_ = true;
	  break;
  case EDISON_SEGMENT:
	  //filter the image
	  iProc.Filter(sigmaS, sigmaR, (SpeedUpLevel)(*(int *)(parameterList_[PARAM_SPEEDUP])));
	  if(iProc.ErrorStatus) 
	  {
		  return EXE_ERROR;
	  }

	  //obtain the filtered image
	  if(filtImageDefined_) 
		  delete [] filtImage_;
	  filtImage_ = new unsigned char [height_ * width_ * dim_];
	  iProc.GetResults(filtImage_);
	  if(iProc.ErrorStatus) 
	  {
		  return EXE_ERROR;
	  }
	  filtImageDefined_ = true;

	  //fuse regions
	  iProc.FuseRegions(sigmaR, minRegion);
	  if(iProc.ErrorStatus) 
	  {
		  return EXE_ERROR;
	  }

	  //obtain the segmented image
	  if(segmImageDefined_) 
		  delete [] segmImage_;
	  segmImage_ = new unsigned char [height_ * width_ * dim_];
	  iProc.GetResults(segmImage_);
	  if(iProc.ErrorStatus) 
	  {
		  return EXE_ERROR;
	  }
	  segmImageDefined_ = true;
	  break; 
	}

	//define the boundaries
	RegionList *regionList        = iProc.GetBoundaries();
	int        *regionIndeces     = regionList->GetRegionIndeces(0);
	int        numRegions         = regionList->GetNumRegions();
	numBoundaries_ = 0;
	int i;
	for(i = 0; i < numRegions; i++) 
	{
		numBoundaries_ += regionList->GetRegionCount(i);
	}
	if(boundariesDefined_) 
	{
		delete [] boundaries_;
		boundaries_ = NULL;
	}
	boundaries_ = new int [numBoundaries_];
	for(i = 0; i < numBoundaries_; i++) 
	{
		boundaries_[i] = regionIndeces[i];
	}
	boundariesDefined_ = true;

	//prompt the user
	CmCPrompt("-------------------------------------------------------------------------\n\n");

	//done.
	return NO_ERRORS;

}


BOOL EDISON::SetSegParams(unsigned char* inBIPImageData,
						  int DIM,
						  int WIDTH,
						  int HEIGHT,
						  int SPATIAL_BANDWIDTH /*= 7*/,
						  float RANGE_BANDWIDTH /*= 6.5*/,
						  int SPEEDUP  /*= 1*/,
						  float subSpeedUp /*= 0.1*/,
						  bool bUseWeightMap /*= true*/,
						  int GRADIENT_WINDOW_RADIUS /*= 2*/,
						  float EDGE_STRENGTH_THRESHOLD /*= 0.3*/,
						  float MIXTURE_PARAMETER  /*= 0.3*/,
						  bool bAllowCache/* = true*/,
						  bool bWaitBar)
{
	//obtain parameters...
	
	char action_str[99];
	action_str[0] = 0; //initialize string

	//初始化参数
	dim_ = DIM;
	width_ = WIDTH;
	height_ = HEIGHT;
	m_bWaitBar = bWaitBar;

	//inputImage_ = new unsigned char[width_ * height_ * dim_];
	//memcpy(inputImage_,inBSQImageData,sizeof(unsigned char) * width_ * height_ * dim_);
	
	//注：在程序外面内存分配，但不需要外面释放，这里把指针在模块内释放
	inputImage_ = inBIPImageData;
	inputDefined_ = true;
	m_sigmaS = SPATIAL_BANDWIDTH;
	m_sigmaR  = RANGE_BANDWIDTH;
	m_bUseWeightMap = bUseWeightMap;
	m_SPEEDUP = SPEEDUP;
	m_subSpeedUp = subSpeedUp;
	m_bAllowCache = bAllowCache;

	if(m_bUseWeightMap) 
	{
		m_gradWindRad = GRADIENT_WINDOW_RADIUS;
		m_threshold   = EDGE_STRENGTH_THRESHOLD;
		m_mixture     = MIXTURE_PARAMETER;
	}

	//create image processing object
	if(piProc)
	{
		delete piProc;
		piProc = NULL;
	}

	piProc = new msImageProcessor(m_bWaitBar);

	if(!m_bWaitBar)
		piProc->SetRange(0,0);
	else
		piProc->SetRange(m_wait_begin,m_wait_end);

	//define the image to be processed as the input image
	//沈占锋2009.11.23增加最后一个参数，允许这个函数更新最后的参数m_bHaveSaved，用以指示
	//当前读取的已经存储的影像数据与当前正在处理的影像数据是否一致，一致则为true
	BOOL ret = FALSE;

	if(dim_ == 3)
		ret = piProc->DefineImage(fpSaveFile, inputImage_, COLOR, height_, width_,bAllowCache,&m_bHaveSaved);
	else
		ret = piProc->DefineImage(fpSaveFile, inputImage_, GRAYSCALE, height_, width_,bAllowCache,&m_bHaveSaved);

	if(piProc->ErrorStatus) return FALSE;

	return ret;
}


void EDISON::SetRange(int begin, int end)
{
	m_wait_begin = begin; 
	m_wait_end = end;

	if(piProc)piProc->SetRange(begin,end);

	return;
}


BOOL EDISON::DoRSSegByMS()
{
	//compute and set weight map if synergistic segmentation is requested
	//沈占锋，2009.11.23判断是否已经存储过
	int begin_pos = m_wait_begin;
	int end_pos   = m_wait_end;

	SetRange(begin_pos, (end_pos-begin_pos)/3+begin_pos);
	while(m_bAllowCache && m_bHaveSaved)
	{
		//FILE *fpSaveFile;
		//\\CString strDBName = AfxGetApp()->GetProfileString("SINCE2008","CurrnetDBFile","");
		//char strDBName[MAX_PATH];

		//DWORD dw = GetProfileString((LPCTSTR)"SINCE2008",(LPCTSTR)"CurrnetDBFile",(LPCTSTR)"",(LPSTR)strDBName,255);
		//TCHAR appName[MAX_PATH],keyName[MAX_PATH],defaultName[1],stringName[MAX_PATH]  ;//(LPCTSTR)strTmpDBName;
		//mbstowcs(appName,"SINCE2008",MAX_PATH);
		//mbstowcs(keyName,"CurrnetDBFile",MAX_PATH);
		//mbstowcs(defaultName,"",1);
		//DWORD dw = GetProfileString(appName,keyName,defaultName,stringName,MAX_PATH);
		//wcstombs(strDBName,stringName,MAX_PATH);

		//ASSERT( _access(strDBName,0) == -1);
		//fopen_s(&fpSaveFile,strDBName,"rb");
		
		//跳过若干字节，直接读取confMap与rank
		//需要跳过：3个int，3*width*dim个uchar（1、中、最后行），width*height*dim个float（LUV Image）
		//fseek(fpSaveFile,sizeof(int)*3,SEEK_CUR);
		////\\filep.Seek(sizeof(int)*3,CFile::current);
		//fseek(fpSaveFile,sizeof(unsigned char)*3*width_*dim_,SEEK_CUR);
		////\\filep.Seek(sizeof(unsigned char)*3*width_*dim_,CFile::current);
		//fseek(fpSaveFile,sizeof(float)*width_*height_*dim_,SEEK_CUR);
		//\\filep.Seek(sizeof(float)*width_*height_*dim_,CFile::current);
		//\\int dsd = ftell(fp);
		//\\int dsd = filep.GetPosition();

		//读取四个参数
		bool bSavedUserWeightMap;
		int nSavedGradRadius;
		
		//float fSavedEdgeThre;
		float fSavedMixture;
		int resu = (int)fread(&bSavedUserWeightMap,sizeof(bool),1,fpSaveFile);
		//fscanf(fpSaveFile,"%d",&bSavedUserWeightMap);
		//\\UINT resu = filep.Read(&bSavedUserWeightMap,sizeof(bool));
		if(resu != 1 || !(bSavedUserWeightMap && m_bUseWeightMap))
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		
		resu = (int)fread(&nSavedGradRadius,sizeof(int),1,fpSaveFile);
		//fscanf(fpSaveFile,"%d",&nSavedGradRadius);
		//\\resu = filep.Read(&nSavedGradRadius,sizeof(int));
		if(resu != 1 || nSavedGradRadius != m_gradWindRad)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		
		////resu = fread(&fSavedEdgeThre,sizeof(float),1,fp);
		////if(resu != 1 || fabs(fSavedEdgeThre-m_threshold) > 1e-5)
		////{
		////	m_bHaveSaved = false;
		////	fclose(fp);
		////	break;
		////}
		resu = (int)fread(&fSavedMixture,sizeof(float),1,fpSaveFile);
		//\\resu = filep.Read(&fSavedMixture,sizeof(float));
		if(resu != 1 || fabs(fSavedMixture-m_mixture) > 1e-5)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		
		//运行至此证明存储的与正在运行的一致
		if(weightMapDefined_) 
		delete [] weightMap_;

		weightMap_ = new float [height_ * width_];
		if(!weightMap_) return EXE_OUT_OF_MEMORY;
		
		//读取weightMap_
		resu = (int)fread(weightMap_,sizeof(float)*width_*height_,1,fpSaveFile);
		//\\resu = filep.Read(weightMap_,sizeof(float)*width_*height_);
		if(resu != 1)
		{
			delete weightMap_;
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		//indicate that maps are now defined
		gradMapDefined_   = true;
		confMapDefined_   = true;
		weightMapDefined_ = true;

		//indicate that the changed parameters have been accounted for
		CmCGradWinChanged = false;
		CmCMixtureChanged = false;
		piProc->SetWeightMap(weightMap_,m_threshold);//same as line 739

		//fclose(fpSaveFile);
		//\\filep.Close();
		break;//不允许循环
	}

	if( !m_bHaveSaved)
	{
		if(m_bUseWeightMap) 
		{
			if(CmCUseCustomWeightMap) //不会运行此处
			{
				if(!custMapDefined_) 
					return EXE_INPUT_UNDEFINED;
				piProc->SetWeightMap(custMap_, m_threshold);
			} 
			else 
			{    
				int error = ComputeWeightMap(m_gradWindRad,m_mixture);
				if(error) 
					return error;
				piProc->SetWeightMap(weightMap_,m_threshold);

				//存储weightMap_
				//先读出原来的，再重新写入一个新的
				if (m_bAllowCache)
				{
					fwrite(&m_bUseWeightMap,sizeof(bool),1,fpSaveFile);
					fwrite(&m_gradWindRad,sizeof(int),1,fpSaveFile);
					fwrite(&m_mixture,sizeof(float),1,fpSaveFile);
					fwrite(weightMap_,sizeof(float)*width_*height_,1,fpSaveFile);
				}

				//if(weightMap_)  {delete [] weightMap_;weightMap_ = NULL;}
			}
			if(piProc->ErrorStatus) 
			{
				return EXE_ERROR;
			}
		}
	}

	while(m_bAllowCache && m_bHaveSaved)
	{
		//FILE *fpSaveFile;
		//\\CFile filep;
		//\\CString strDBName = AfxGetApp()->GetProfileString("SINCE2008","CurrnetDBFile","");
		//char strDBName[MAX_PATH];
		//DWORD dw = GetProfileString((LPCTSTR)"SINCE2008",(LPCTSTR)"CurrnetDBFile",(LPCTSTR)"",(LPSTR)strDBName,255);
		//TCHAR appName[MAX_PATH],keyName[MAX_PATH],defaultName[1],stringName[MAX_PATH]  ;//(LPCTSTR)strTmpDBName;
		//mbstowcs(appName,"SINCE2008",MAX_PATH);
		//mbstowcs(keyName,"CurrnetDBFile",MAX_PATH);
		//mbstowcs(defaultName,"",1);
		//DWORD dw = GetProfileString(appName,keyName,defaultName,stringName,MAX_PATH);
		//wcstombs(strDBName,stringName,MAX_PATH);

		//ASSERT( _access(strDBName,0) == -1);
		//fopen_s(&fpSaveFile,strDBName,"rb");
		//\\filep.Open(strDBName,CFile::typeBinary|CFile::modeRead);

		//跳过若干字节，直接读取confMap与rank
		//需要跳过：3个int，3*width*dim个uchar（1、中、最后行），width*height*dim个float（LUV Image）
		//fseek(fp,sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_
		//	  +sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_,SEEK_CUR);
		//long Lsize = sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_
		//	+sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_;
		//\\filep.Seek(Lsize,CFile::current);
		//fseek(fpSaveFile,0,SEEK_END);
		//\\long Length = filep.GetLength();
		//long Length = ftell(fpSaveFile);
		//fseek(fpSaveFile,0,SEEK_SET);
		//fseek(fpSaveFile,Lsize,SEEK_CUR);
		//long dsd = ftell(fpSaveFile);
		//\\long dsd = filep.GetPosition();
		//if (dsd +4 >= Length)
		//{
		//	m_bHaveSaved = false;
		//	fclose(fpSaveFile);
		//	//\\filep.Close();
		//	break;
		//}
		//读取四个参数
		int nSavedSigmaS;
		float fSavedSigmaR;
		int nSavedSpeed;
		float fSavedSubSpeed;
		int resu = (int)fread(&nSavedSigmaS,sizeof(int),1,fpSaveFile);
		//\\UINT resu = filep.Read(&nSavedSigmaS,sizeof(int));
		if(resu != 1 || nSavedSigmaS != m_sigmaS)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		resu = (int)fread(&fSavedSigmaR,sizeof(float),1,fpSaveFile);
		//\\resu = filep.Read(&fSavedSigmaR,sizeof(float));
		if(resu != 1 || fabs(fSavedSigmaR - m_sigmaR) >1e-5)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		resu = (int)fread(&nSavedSpeed,sizeof(int),1,fpSaveFile);
		//\\resu = filep.Read(&nSavedSpeed,sizeof(int));
		if(resu != 1 || nSavedSpeed != m_SPEEDUP)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}
		resu = (int)fread(&fSavedSubSpeed,sizeof(float),1,fpSaveFile);
		//\\resu = filep.Read(&fSavedSubSpeed,sizeof(float));
		if(resu != 1 || nSavedSpeed == 2 && fabs(fSavedSubSpeed-m_subSpeedUp) > 1e-5)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}

		//threshold
		float fSavedEdgeThre;
		resu = (int)fread(&fSavedEdgeThre,sizeof(float),1,fpSaveFile);
		//\\resu = filep.Read(&fSavedEdgeThre,sizeof(float));
		if(resu != 1 || fabs(fSavedEdgeThre-m_threshold) > 1e-5)
		{
			m_bHaveSaved = false;
			fclose(fpSaveFile);
			//\\filep.Close();
			break;
		}

		//运行至此证明存储的与正在运行的一致

		//读取
		// 2. regionCount	int		1
		// 1. labels		int		w*h
		// 3. modes			float	regionCount*dim_
		// 4. modePointCounts	int	regionCount

		resu = (int)fread(&piProc->regionCount,sizeof(int),1,fpSaveFile);
		//\\resu = filep.Read(&piProc->regionCount,sizeof(int));
		_ASSERT(resu == 1);
		_ASSERT(piProc->regionCount>0);
		if(piProc->labels)delete piProc->labels;
		piProc->labels = new int[width_*height_];
		resu = (int)fread(piProc->labels,sizeof(int)*width_*height_,1,fpSaveFile);
		//\\resu = filep.Read(piProc->labels,sizeof(int)*width_*height_);
		_ASSERT(resu == 1);
		if(piProc->modes)delete piProc->modes;
		piProc->modes = new float[piProc->regionCount*dim_];
		resu = (int)fread(piProc->modes,sizeof(float)*piProc->regionCount*dim_,1,fpSaveFile);
		//\\resu = filep.Read(piProc->modes,sizeof(float)*piProc->regionCount*dim_);
		_ASSERT(resu == 1);
		if(piProc->modePointCounts)delete piProc->modePointCounts;
		piProc->modePointCounts = new int[piProc->regionCount];
		resu = (int)fread(piProc->modePointCounts,sizeof(int)*piProc->regionCount,1,fpSaveFile);
		//\\resu = filep.Read(piProc->modePointCounts,sizeof(int)*piProc->regionCount);
		_ASSERT(resu == 1);
		piProc->SetClassStateOutputDefined(true);
	
		if(inputImage_) {delete [] inputImage_;inputImage_ = NULL;}

		//fclose(fpSaveFile);
		//\\filep.Close();
		break;//不允许循环
	}

	SetRange((end_pos-begin_pos)/3+begin_pos, end_pos);

	if(!m_bHaveSaved)
	{
		//存储以下几个参数
		//m_sigmaS,m_sigmaR,m_SPEEDUP,m_subSpeedUp

		piProc->SetSpeedThreshold(m_subSpeedUp);
		piProc->Filter(m_sigmaS, m_sigmaR, (SpeedUpLevel)m_SPEEDUP);
		if(piProc->ErrorStatus) 
		{
			return EXE_ERROR;
		}

		if (m_bAllowCache)
		{
#ifdef SHOW_PROGRESS
			CWaitCursor wait;
#endif

			//存储weightMap_
			//先读出原来的，再重新写入一个新的
			//char *strtmp = new char[sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_
			//						+sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_];

			//FILE *fpSaveFile;
			//\\CFile filep;
			//\\CString strDBName = AfxGetApp()->GetProfileString("SINCE2008","CurrnetDBFile","");
		//	char strDBName[MAX_PATH];
		//DWORD dw = GetProfileString((LPCTSTR)"SINCE2008",(LPCTSTR)"CurrnetDBFile",(LPCTSTR)"",(LPSTR)strDBName,255);
			//TCHAR appName[MAX_PATH],keyName[MAX_PATH],defaultName[1],stringName[MAX_PATH]  ;//(LPCTSTR)strTmpDBName;
			//mbstowcs(appName,"SINCE2008",MAX_PATH);
			//mbstowcs(keyName,"CurrnetDBFile",MAX_PATH);
			//mbstowcs(defaultName,"",1);
			//DWORD dw = GetProfileString(appName,keyName,defaultName,stringName,MAX_PATH);
			//wcstombs(strDBName,stringName,MAX_PATH);

			//ASSERT( _access(strDBName,0) == -1);
			//fopen_s(&fpSaveFile,strDBName,"ab");
			//\\filep.Open(strDBName,CFile::typeBinary|CFile::modeReadWrite);

			//ASSERT(fp);
			//int resu = (int)fread(strtmp,sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_
			//				   	  +sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_,1,fp);
			//ASSERT(resu == 1);
			//fclose(fp);

			//fopen_s(&fp,strDBName,"wb");
			//resu = (int)fwrite(strtmp,sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_
			//	+sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_,1,fp);
			//ASSERT(resu == 1);
			//delete strtmp;

//			fseek(fp,sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_ \
				+sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_,SEEK_SET);
			//\\filep.Seek(sizeof(int)*3+sizeof(unsigned char)*3*width_*dim_+sizeof(float)*width_*height_*dim_ \
				+sizeof(bool)+sizeof(int)+sizeof(float)+sizeof(float)*width_*height_,CFile::begin);
			//fseek(fpSaveFile,0,SEEK_END);

			size_t resu = (int)fwrite(&m_sigmaS,sizeof(int),1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(&m_sigmaS,sizeof(int));
			resu = (int)fwrite(&m_sigmaR,sizeof(float),1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(&m_sigmaR,sizeof(float));
			resu = (int)fwrite(&m_SPEEDUP,sizeof(int),1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(&m_SPEEDUP,sizeof(int));
			resu = (int)fwrite(&m_subSpeedUp,sizeof(float),1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(&m_subSpeedUp,sizeof(float));
			resu = (int)fwrite(&m_threshold,sizeof(float),1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(&m_threshold,sizeof(float));

			//存储
			// 2. regionCount	int		1
			// 1. labels		int		w*h
			// 3. modes			float	regionCount*dim_
			// 4. modePointCounts	int	regionCount
			resu = (int)fwrite(&piProc->regionCount,sizeof(int),1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(&piProc->regionCount,sizeof(int));
			resu = (int)fwrite(piProc->labels,sizeof(int)*width_*height_,1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(piProc->labels,sizeof(int)*width_*height_);
			resu = (int)fwrite(piProc->modes,sizeof(float)*piProc->regionCount*dim_,1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(piProc->modes,sizeof(float)*piProc->regionCount*dim_);
			resu = (int)fwrite(piProc->modePointCounts,sizeof(int)*piProc->regionCount,1,fpSaveFile);
			//ASSERT(resu == 1);
			//\\filep.Write(piProc->modePointCounts,sizeof(int)*piProc->regionCount);
//可以临时删除，再读取			
			//delete piProc->labels;
			//piProc->labels = NULL;
			//delete piProc->modes;
			//piProc->modes = NULL;
			//delete piProc->modePointCounts;
			//piProc->modePointCounts = NULL;
			//fclose(fpSaveFile);//存储完毕
			//\\filep.Close();
		}
	}


	//注意，需要初始化下面的变量，但不需要从已经存储的数据库中读取其值，也不需要存储其值
	if(piProc->msRawData)delete piProc->msRawData;
	piProc->msRawData = new float[width_*height_*dim_];
	////resu = fread(piProc->msRawData,sizeof(float)*width_*height_*dim_,1,fp);
	////ASSERT(resu == 1);

	m_wait_begin = begin_pos;
	m_wait_end   = end_pos;
	if(fpSaveFile)
		fclose(fpSaveFile);
	return TRUE;
}



BOOL EDISON::GetSegmentResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA)
{
	if (*OUT_BIP_IMAGE_DATA)	delete *OUT_BIP_IMAGE_DATA;
	unsigned char* outData = new unsigned char[width_*height_*dim_];;
	//开辟数据，共width*height*3*SCALE_NUM大小
	if(!outData)return FALSE;

	//循环每个尺度SCALES[0,1,...SCALE_NUM-1]进行合并
	piProc->FuseRegions(m_sigmaS,curSCALE);
	if(piProc->ErrorStatus) 
	{
		return FALSE;
	}
	piProc->GetResults(outData);
	if(piProc->ErrorStatus) 
	{
		return FALSE;
	}
	//////////////////////////////////////////////////////////////////////////
	//TODO:    沈占锋      需要在此将相关参数及分割结果存储，以备后用
	//////////////////////////////////////////////////////////////////////////
		
	//for (int ii=0;ii<NUM_SCALES_MINIMUM_REGION_AREA;ii++)
	//{
	//	//strcpy(out,"d:\\data\\segmentation\\OUTPUT_SEGM_IMAGE_");
	//	//sprintf(strtmp,"%d",ii+1);
	//	//strcat(out,strtmp);
	//	//strcat(out,".ppm");

	//	//fuse regions
	//	int minRegion_tmp = SCALES_ARRAY_MINIMUM_REGION_AREA[ii];
	//	iProc.FuseRegions(m_sigmaR, minRegion_tmp);
	//	if(iProc.ErrorStatus) 
	//	{
	//		return EXE_ERROR;
	//	}

	//	//obtain the segmented image
	//	if(segmImageDefined_) 
	//		delete [] segmImage_;
	//	segmImage_ = new unsigned char [height_ * width_ * dim_];
	//	iProc.GetResults(segmImage_);
	//	if(iProc.ErrorStatus) 
	//	{
	//		return EXE_ERROR;
	//	}
	//	segmImageDefined_ = true;


	//	//define the boundaries
	//	RegionList *regionList        = iProc.GetBoundaries();
	//	int        *regionIndeces     = regionList->GetRegionIndeces(0);
	//	int        numRegions         = regionList->GetNumRegions();
	//	numBoundaries_ = 0;
	//	int i;
	//	for(i = 0; i < numRegions; i++) 
	//	{
	//		numBoundaries_ += regionList->GetRegionCount(i);
	//	}
	//	if(boundariesDefined_) 
	//		delete [] boundaries_;
	//	boundaries_ = new int [numBoundaries_];
	//	for(i = 0; i < numBoundaries_; i++) 
	//	{
	//		boundaries_[i] = regionIndeces[i];
	//	}
	//	boundariesDefined_ = true;

	//	////prompt the user
	//	//CmCPrompt("-------------------------------------------------------------------------\n\n");

	//	//Save(out,FILE_PGM,OUTPUT_SEGM_IMAGE);
	//	Output(&(outSingleBand_ResultData[ii*WIDTH*HEIGHT]));
	//}
	*OUT_BIP_IMAGE_DATA = outData;
	return TRUE;
}
RegionList*	EDISON::GetSegmentBoundaries(int curSCALE)
{
	if (!piProc)
	{
		return NULL;
	}
	piProc->FuseRegions(m_sigmaR,curSCALE);
	if(piProc->ErrorStatus) 
	{
		return FALSE;
	}
	return piProc->GetBoundaries();
}
int	EDISON::GetResultRegions(int curSCALE,int** OUT_LABELS,unsigned char** OUT_MODES,int**OUT_MODE_POINT_COUNTS)
{
	if (!piProc)
	{
		return NULL;
	}
	piProc->FuseRegions(m_sigmaR, curSCALE);
	if(piProc->ErrorStatus) 
	{
		return EXE_ERROR;
	}

	////define the boundaries
	//RegionList *regionList        = piProc.GetBoundaries();
	//int        *regionIndeces     = regionList->GetRegionIndeces(0);
	//int        numRegions         = regionList->GetNumRegions();
	//numBoundaries_ = 0;
	//int i;
	//for(i = 0; i < numRegions; i++) 
	//{
	//	numBoundaries_ += regionList->GetRegionCount(i);
	//}
	//if(boundariesDefined_) 
	//	delete [] boundaries_;
	//boundaries_ = new int [numBoundaries_];
	//for(i = 0; i < numBoundaries_; i++) 
	//{
	//	boundaries_[i] = regionIndeces[i];
	//}
	//boundariesDefined_ = true;
	return piProc->GetRegions(OUT_LABELS, OUT_MODES,OUT_MODE_POINT_COUNTS);

}
void		EDISON::SaveSegInformationDB(char* strFileName)
{

}

int EDISON::Output(unsigned char *out)
{
	//if(!out) return EXE_OUT_OF_MEMORY;
	//memset(out, 0, height_*width_*sizeof(unsigned char));

	//if(boundaries_) 
	//{
	//	for(int i = 0; i < numBoundaries_; i++) 
	//	{
	//		out[boundaries_[i]] = 255;
	//	}
	//}

	memset(out, 0, height_*width_*sizeof(unsigned char));
	if (dim_ == 1)
	{
		memcpy(out,segmImage_,sizeof(unsigned char)*width_*height_);
	}
	else
	{
		for (int i=0;i<height_*width_;i++)
		{
			out[i] = segmImage_[i*dim_];
		}
	}
	return NO_ERRORS;
}

int EDISON::ComputeWeightMap( void )
{

	//do not do un-necessary computation
	if(weightMapDefined_ && !CmCGradWinChanged && !CmCMixtureChanged) return NO_ERRORS;

	//attain necessary parameters...
	int   gradWindowRadius = *((int *)(parameterList_[PARAM_GRADIENT_WINDOW_RADIUS]));
	float mixtureParam     = *((float *)(parameterList_[PARAM_MIXTURE_PARAMETER]));

	//compute gradient and confidence maps
	if(!gradMapDefined_ || !confMapDefined_ || CmCGradWinChanged) 
	{
		if(gradMapDefined_) delete [] gradMap_;
		if(confMapDefined_) delete [] confMap_;
		gradMap_ = new float [height_ * width_];
		confMap_ = new float [height_ * width_];
		if(!gradMap_ || !confMap_) return EXE_OUT_OF_MEMORY;
		BgEdgeDetect edgeDetector(gradWindowRadius);
		BgImage inputImage;
		
		if(dim_ == 3)
			inputImage.SetImage(inputImage_, width_, height_, true);
		else
			inputImage.SetImage(inputImage_, width_, height_, false);

		edgeDetector.ComputeEdgeInfo(&inputImage, confMap_, gradMap_,m_wait_begin, m_wait_end);
	}

	//compute weight map
	if(!weightMapDefined_ || !gradMapDefined_ || !confMapDefined_ || CmCGradWinChanged || CmCMixtureChanged) 
	{
		CmCPrompt("Computing weight map...");
		if(weightMapDefined_)
		{
			delete [] weightMap_;
			weightMap_ = NULL;
		}

		weightMap_ = new float [height_ * width_];
		if(!weightMap_) return EXE_OUT_OF_MEMORY;
		int i;
		for(i = 0; i < width_*height_; i++) 
		{
			if(gradMap_[i] > 0.02)
			{
				weightMap_[i] = mixtureParam*gradMap_[i] + (1 - mixtureParam)*confMap_[i];
			} 
			else 
			{
				weightMap_[i] = 0;
			}
		}
		CmCPrompt("done.\n");
	}

	//indicate that maps are now defined
	gradMapDefined_   = true;
	confMapDefined_   = true;
	weightMapDefined_ = true;

	//indicate that the changed parameters have been accounted for
	CmCGradWinChanged = false;
	CmCMixtureChanged = false;

	//done.
	return NO_ERRORS;
}


int EDISON::ComputeWeightMap( int GRADIENT_WINDOW_RADIUS, float MIXTURE_PARAMETER)
{
	//do not do un-necessary computation
	//if(weightMapDefined_ && !CmCGradWinChanged && !CmCMixtureChanged) return NO_ERRORS;

	//attain necessary parameters...
	int   gradWindowRadius = GRADIENT_WINDOW_RADIUS;
	float mixtureParam     = MIXTURE_PARAMETER;

	//compute gradient and confidence maps
	/*if(!gradMapDefined_ || !confMapDefined_ || CmCGradWinChanged)*/ 
	{
		if(gradMapDefined_) delete [] gradMap_;
		if(confMapDefined_) delete [] confMap_;
		gradMap_ = new float [height_ * width_];
		confMap_ = new float [height_ * width_];
		if(!gradMap_ || !confMap_) return EXE_OUT_OF_MEMORY;
		BgEdgeDetect edgeDetector(gradWindowRadius);
		BgImage inputImage;
		if(dim_ == 3)
			inputImage.SetImage(inputImage_, width_, height_, true);
		else
			inputImage.SetImage(inputImage_, width_, height_, false);
		
		delete inputImage_;
		inputImage_ = NULL;
		

		if(!m_bWaitBar)
		{
			m_wait_begin = 0;
			m_wait_end   = 0;
		}

		edgeDetector.ComputeEdgeInfo(&inputImage, confMap_, gradMap_,m_wait_begin, m_wait_end);	
	}

	long new_img_size = width_*height_;

	//compute weight map
	if(!weightMapDefined_ || !gradMapDefined_ || !confMapDefined_ || CmCGradWinChanged || CmCMixtureChanged || CmCSplitWinChanged || new_img_size != old_img_size) 
	{
		CmCPrompt("Computing weight map...");
		if(weightMap_)
		{
			delete [] weightMap_;
			weightMap_ = NULL;
		}

		weightMap_ = new float [height_ * width_];
		if(!weightMap_) return EXE_OUT_OF_MEMORY;
		int i;
		for(i = 0; i < width_*height_; i++) 
		{
			if(gradMap_[i] > 0.02) 
			{
				weightMap_[i] = mixtureParam*gradMap_[i] + (1 - mixtureParam)*confMap_[i];
			} 
			else 
			{
				weightMap_[i] = 0;
			}
		}
		CmCPrompt("done.\n");

		old_img_size = new_img_size;
	}

	//indicate that maps are now defined
	gradMapDefined_   = true;
	confMapDefined_   = true;
	weightMapDefined_ = true;

	//indicate that the changed parameters have been accounted for
	CmCGradWinChanged = false;
	CmCMixtureChanged = false;
////shenzf 20100303
	if(gradMap_)    {delete [] gradMap_;gradMap_ = NULL;}
	if(confMap_)    {delete [] confMap_;confMap_ = NULL;}
////end
	//done.
	return NO_ERRORS;
}

