/****************************************************************
*
* Project:  地理信息处理工具集
* Purpose:  各种层的定义类
* Author:   沈占锋, shenzf@irsa.ac.cn, 2011年-4月-8日
*
****************************************************************
* Copyright (c) All rights reserved.
* 版权所有  (c) 保留所有权利
****************************************************************/

#include "stdafx.h"
#include "MeanShiftSeg.h"
#include ".\segm\msImageProcessor.h"
#include "math.h"
#include <io.h>
//#include "polygonize.h"
//#include "gdal.h"
//#include "gdal_priv.h"
//#include "ogrsf_frmts.h"
//#include "gdal_alg.h"
//#include "..\RSL15LakeExt\WaterExtract\polygonize.h"
//#include "gt_datasource.h"
//#include "gt_rasterreader.h"
//#include "gt_datadriver.h"
//#include "gt_warp.h"

//#include "../../Include/gDosModel/gDOSToolsIOAPI.h"

//#include "DataManageNCP.h"
//#include "gDOSToolsIOAPI.h"

RasterSegByMS::RasterSegByMS(void)
{
	GDALAllRegister();
	m_bHaveSegmented = false;
	m_IN_BIP_IMAGE_DATA = NULL;
	m_bAllowCache = true;
	m_strFileName = "";
	freeScaleInformation();
}



RasterSegByMS::~RasterSegByMS(void)
{
	if (strlen(m_strFileName) > 0)
	{
		free(m_strFileName);
	}
	freeScaleInformation();
}


void RasterSegByMS::freeScaleInformation()
{
	map<int,ScaleInformation>::iterator it;

	for(it = m_scaleInformation.begin();it != m_scaleInformation.end();it++)
	{
		ScaleInformation si = m_scaleInformation[(*it).first];
		/*if (si.boundaryPointList)
		{
		delete si.boundaryPointList;
		si.boundaryPointList = NULL;
		}*/
		if (si.objectAverageRGB)
		{
			delete si.objectAverageRGB;
			si.objectAverageRGB = NULL;
		}
		if (si.pixelInObjectLabel)
		{
			delete si.pixelInObjectLabel;
			si.pixelInObjectLabel = NULL;
		}
		if (si.pointCountInObject)
		{
			delete si.pointCountInObject;
			si.pointCountInObject = NULL;
		}
		if (si.segRGBResult)
		{
			delete si.segRGBResult;
			si.segRGBResult = NULL;
		}
	}

	m_scaleInformation.clear();
	return;
}


char*		RasterSegByMS::getFuncDescription()
{
	char des[5500];
	strcpy_s(des,5500,"\r\n\
					  Improved MeanShift Algorithm for (High-Resolution) Remote Sensing Images Segmentation\r\n\
					  CopyRight by IRSA http://www.irsa.ac.cn E-mail:shenzf@irsa.ac.cn Tele-Phone:010-64869130\r\n\
					  USage(Sequence):\r\n\
					  You can call these functions by sequence or call directly startSegByMS(...) if you need not set parameters\r\n\
					  getFuncDescription()\r\n\
					  setSegParams(int SPATIAL_BANDWIDTH = 7,float COLOR_RANGE_BANDWIDTH = 6.5, int SPEEDUP  = NO_SPEEDUP,bool USE_WEIGHT_MAP = true,\r\n\
					  int GRADIENT_WINDOW_RADIUS = 2,float EDGE_STRENGTH_THRESHOLD = 0.3,float MIXTURE_PARAMETER  = 0.3,bool bAllowCache = true)\r\n\
					  Parameters:\r\n\
					  SPATIAL_BANDWIDTH:sigmaS\r\n\
					  COLOR_RANGE_BANDWIDTH:sigmaR\r\n\
					  SPEEDUP:the speed of algorithms,you can choose NO_SPEEDUP(or 0) MED_SPEEDUP(or 1) HIGH_SPEEDUP(or 2)\r\n\
					  if you have chose HIGH_SPEEDUP(or 2), you should call the function _2_SetSpeedThreshold(float SPEED_THRESHOLD between 0 and 1),\r\n\
					  or else you neednot call it. \r\n\
					  SPEED_THRESHOLD: 0.1 default. the algorithm speed will increase with the increase of SPEED_THRESHOLD,only useful for HIGH_SPEEDUP \r\n\
					  USE_WEIGHT_MAP: true or false indicate wether or not use weight map\r\n\
					  GRADIENT_WINDOW_RADIUS: 2 default\r\n\
					  EDGE_STRENGTH_THRESHOLD: 0.3 default\r\n\
					  MIXTURE_PARAMETER: 0.3 default\r\n\
					  bAllowCache:true default\r\n\
					  startSegByMS(unsigned char* IN_BIP_IMAGE_DATA,int DIM,int WIDTH,int HEIGHT)\r\n\
					  Parameters:\r\n\
					  IN_BIP_IMAGE_DATA: unsigned char* image data stored in BIP(RGB) mode, NEED NOT delete memory by user\r\n\
					  DIM: the band count of IN_BIP_IMAGE_DATA\r\n\
					  WIDTH: the width of IN_BIP_IMAGE_DATA\r\n\
					  HEIGHT: the height of IN_BIP_IMAGE_DATA\r\n\
					  getSegImageResults(int SCALE,unsigned char** OUT_BIP_IMAGE_DATA)\r\n\
					  Parameters:\r\n\
					  SCALE: the merge scale\r\n\
					  OUT_BIP_IMAGE_DATA is the segmented results of unsigned char data type with DIM (see startSegByMS()) \r\n\
					  and this ptr NEED you to delete so as to free memory.\r\n\
					  getSegBoundariesPointList(int SCALE)\r\n\
					  This Function return the data type of RegionList*, and this class gives the boundries of different objects\r\n\
					  getResultRegionsInfo(int** OUT_LABELS,float** OUT_MODES,int**OUT_MODE_POINT_COUNTS)\r\n\
					  Returns An integer regionCount that specifies the number of regions contained in the processed image\r\n\
					  Parameters:\r\n\
					  OUT_LABELS: An integer array of length (height*width) which contains at every pixel location (x,y) a label\r\n\
					  relating that pixel to a region whose mode is specified by modes and whose area is specified by modePointCounts\r\n\
					  OUT_MODES: A floating point array of length regionCount*N containing the feature space component of each\r\n\
					  region (e.g. LUV), and indexed by region label\r\n\
					  OUT_MODE_POINT_COUNTS: An integer array of length regionCount and indexed by region label, that specifies the region\r\n\
					  area (in pixels) for each segmented image region. (e.g. Area of region having label specifified by l, has area modePointCounts[l] (pixels)\r\n\
					  ");
	return des;
}



bool		RasterSegByMS::setSegParams(unsigned char* IN_BIP_IMAGE_DATA,
	int BANDCOUNT,
	int WIDTH,
	int HEIGHT,
	int SPATIAL_BANDWIDTH /*= 7*/,
	float COLOR_RANGE_BANDWIDTH /*= 6.5*/,
	int SPEEDUP  /*= NO_SPEEDUP*/,
	float SPEED_THRESHOLD /*= 0.1*/,
	bool USE_WEIGHT_MAP /*= true*/,
	int GRADIENT_WINDOW_RADIUS /*= 2*/,
	float EDGE_STRENGTH_THRESHOLD /*= 0.3*/,
	float MIXTURE_PARAMETER  /*= 0.3*/,
	bool bAllowCache/* = true*/,
	bool bWaitBar)
{
	if( m_IN_BIP_IMAGE_DATA          == IN_BIP_IMAGE_DATA &&
		m_DIM                        == BANDCOUNT &&
		m_WIDTH                      == WIDTH &&
		m_HEIGHT                     == HEIGHT &&
		m_SPATIAL_BANDWIDTH          ==	SPATIAL_BANDWIDTH &&
		fabs(m_COLOR_RANGE_BANDWIDTH -	COLOR_RANGE_BANDWIDTH) <1e-5 &&
		m_SPEEDUP                    ==	SPEEDUP  &&
		fabs(m_SPEED_THRESHOLD - SPEED_THRESHOLD) <1e-5	&&
		m_USE_WEIGHT_MAP             ==	USE_WEIGHT_MAP &&
		m_GRADIENT_WINDOW_RADIUS     ==	GRADIENT_WINDOW_RADIUS	&&
		fabs(m_EDGE_STRENGTH_THRESHOLD - EDGE_STRENGTH_THRESHOLD)<1e-5 &&
		fabs(m_MIXTURE_PARAMETER - MIXTURE_PARAMETER )<1e-5 && 
		m_bAllowCache == bAllowCache
		)//一致
		return true;

	m_bHaveSegmented = false;
	m_IN_BIP_IMAGE_DATA = IN_BIP_IMAGE_DATA;
	m_DIM = BANDCOUNT;
	m_WIDTH = WIDTH;
	m_HEIGHT = HEIGHT;
	m_SPATIAL_BANDWIDTH			=	SPATIAL_BANDWIDTH; 	    
	m_COLOR_RANGE_BANDWIDTH		=	COLOR_RANGE_BANDWIDTH	;	
	m_SPEEDUP  					=	SPEEDUP  				;	      
	m_SPEED_THRESHOLD			=	SPEED_THRESHOLD		;	    
	m_USE_WEIGHT_MAP 			=	USE_WEIGHT_MAP 		;	    
	m_GRADIENT_WINDOW_RADIUS	=	GRADIENT_WINDOW_RADIUS;	
	m_EDGE_STRENGTH_THRESHOLD	=	EDGE_STRENGTH_THRESHOLD ;
	m_MIXTURE_PARAMETER  	    =	MIXTURE_PARAMETER  	;
	m_bAllowCache				=	bAllowCache;
	m_bWaitBar                  =   bWaitBar;

	return true;
}

bool		RasterSegByMS::setSegParams(const char* IN_FILENAME,
	int BANDCOUNT/* = 0*/,
	int *iBandList/* = NULL*/,
	int SPATIAL_BANDWIDTH /*= 7*/,
	float COLOR_RANGE_BANDWIDTH /*= 6.5*/,
	int SPEEDUP  /*= NO_SPEEDUP*/,
	float SPEED_THRESHOLD /*= 0.1*/,
	bool USE_WEIGHT_MAP /*= true*/,
	int GRADIENT_WINDOW_RADIUS /*= 2*/,
	float EDGE_STRENGTH_THRESHOLD /*= 0.3*/,
	float MIXTURE_PARAMETER  /*= 0.3*/,
	bool bAllowCache/* = true*/,
	bool bWaitBar)
{
	m_strFileName = _strdup(IN_FILENAME);
	GDALDataset* pDS = (GDALDataset *)GDALOpen(m_strFileName,GA_ReadOnly);
	if(!pDS)return false;


	int XS = pDS->GetRasterXSize();
	int YS = pDS->GetRasterYSize();

	int DIM = BANDCOUNT;
	int nRealBandCount = pDS->GetRasterCount();
	int iSegBandList[3];
	if (iBandList)
	{
		int nBandNum = min(3,DIM);
		for (int i=0;i<nBandNum;i++)
		{
			iSegBandList[i] = iBandList[i];
		}
	}
	if (!iBandList)
	{
		if(nRealBandCount >= 3)
		{
			DIM = 3;
			if(!iBandList)
			{
				iSegBandList[0] = 1;
				iSegBandList[1] = 2;
				iSegBandList[2] = 3;
			}
		}
		else if (nRealBandCount == 1)
		{
			DIM = 1;
			if(!iBandList)
			{
				iSegBandList[0] = 1;
			}
		}
	}
	if(DIM != 1 && DIM != 3)
	{
		GDALClose(pDS);
		return false;
	}

	//double dMin=0,dMax=0,dMean=0,dDev=0;

	unsigned char *pData;
	if (DIM == 1)
	{
		pData = new unsigned char[XS*YS];
		GDALRasterBand* pBand = pDS->GetRasterBand(1);
		double dMin,dMax,dMean,dDev;
		//pBand->getGDALRasterBandRef()->GetStatistics(1,1,&dMin,&dMax,&dMean,&dDev);
		//bSuc = pBand->rasterIO(true,0,0,XS,YS,pData,XS,YS,GTD_Byte);
		CPLErr er = pBand->RasterIO(GF_Read,0,0,XS,YS,pData,XS,YS,GDT_Byte,0,0);
		//GTRasterBand::destroyGTRasterBand(pBand);
	}
	else 
	{
		pData = new unsigned char[XS*YS*3];
		unsigned char *b1= new unsigned char[XS*YS];
		unsigned char *b2= new unsigned char[XS*YS];
		unsigned char *b3= new unsigned char[XS*YS];

		GDALRasterBand* pBand = pDS->GetRasterBand(iSegBandList[0]);
		//pBand->GetStatistics(1,1,&dMin,&dMax,&dMean,&dDev);
		//printf("%f,%f,%f,%f",dMin,dMax,dMean,dDev);
		/*bSuc = pBand->getPixels(-1,0,0,XS,YS,b1);*/
		CPLErr er = pBand->RasterIO(GF_Read,0,0,XS,YS,b1,XS,YS,GDT_Byte,0,0);
		if(er != CE_None)
		{
			delete pData;
			GDALClose(pDS);
			return false;
		}

		pBand = pDS->GetRasterBand(iSegBandList[1]);
		/*bSuc = pBand->getPixels(-1,0,0,XS,YS,b2);*/
		er = pBand->RasterIO(GF_Read,0,0,XS,YS,b2,XS,YS,GDT_Byte,0,0);
		if(er != CE_None)
		{
			delete pData;
			GDALClose(pDS);
			return false;
		}

		pBand = pDS->GetRasterBand(iSegBandList[2]);
		/*bSuc = pBand->getPixels(-1,0,0,XS,YS,b3);*/
		er = pBand->RasterIO(GF_Read,0,0,XS,YS,b3,XS,YS,GDT_Byte,0,0);
		if(er != CE_None)
		{
			delete pData;
			GDALClose(pDS);
			return false;
		}

		for (int i=0;i<XS*YS;i++)
		{
			pData[3*i+0] = b1[i];
			pData[3*i+1] = b2[i];
			pData[3*i+2] = b3[i];
		}
		delete b1;
		delete b2;
		delete b3;
	}
	GDALClose(pDS);
	bool bSuc = setSegParams(pData,DIM,XS,YS,SPATIAL_BANDWIDTH,COLOR_RANGE_BANDWIDTH,SPEEDUP,
		SPEED_THRESHOLD,USE_WEIGHT_MAP,GRADIENT_WINDOW_RADIUS,EDGE_STRENGTH_THRESHOLD,MIXTURE_PARAMETER,bAllowCache,bWaitBar);
	if(!bSuc)
	{
		delete pData;
		return false;
	}
	return true;
}

bool RasterSegByMS::startSegByMS()
{
	//if(m_bHaveSegmented)return TRUE;
	/*
	//shenzf破坏
	struct   tm  local;   
	time_t   t;   
	t=time(NULL);   
	localtime_s(&local,&t);
	//localtime_s(local,&t); 

	int afd=local.tm_year;

	char str1[5];
	GetProfileStringA("dirrid","HaveRightReserved","0",(LPSTR)str1,5);
	int num = atoi(str1);
	////2010年允许使用25次
	if (local.tm_year > 109 / *|| local->tm_mon > 7* /)
	num++;

	_itoa_s(num,str1,10);
	WriteProfileStringA("dirrid","HaveRightReserved",str1);
	if(num > 25)
	{
	return FALSE;
	}

	//shenzf破坏结束*/


	freeScaleInformation();

	////调用EDISON 中的 MS算法实现分割，如果成功返回0
	oEdison.SetRange(0,30);
	BOOL bSuccess = oEdison.SetSegParams(m_IN_BIP_IMAGE_DATA,
		m_DIM,
		m_WIDTH,
		m_HEIGHT,
		m_SPATIAL_BANDWIDTH,
		m_COLOR_RANGE_BANDWIDTH,
		m_SPEEDUP,
		m_SPEED_THRESHOLD,
		m_USE_WEIGHT_MAP,
		m_GRADIENT_WINDOW_RADIUS,
		m_EDGE_STRENGTH_THRESHOLD,
		m_MIXTURE_PARAMETER,
		m_bAllowCache,
		m_bWaitBar);

	if(!bSuccess)return FALSE;

	oEdison.SetRange(30,100);
	bSuccess = oEdison.DoRSSegByMS();
	if(!bSuccess)return FALSE;

	//更新分割状态
	m_bHaveSegmented = true;

	//SaveSegInformationDB("d:\\SinSeg.db");
	////分割结束

	return TRUE;
}



bool RasterSegByMS::getSegImageResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA)
{
	if(!m_bHaveSegmented)return FALSE;
	if (m_scaleInformation.find(curSCALE) == m_scaleInformation.end())//没有此信息，增加
		if(!BuildScaleInformation(curSCALE))return NULL;

	ScaleInformation scaleInfo = m_scaleInformation[curSCALE];
	*OUT_BIP_IMAGE_DATA = scaleInfo.segRGBResult;

	return TRUE;
}

bool RasterSegByMS::addSegResultToLayer(int curSCALE,OGRLayerH hLayer)
{
	if (strlen(m_strFileName) == 0 || NULL == hLayer)
		return false;
	if(!m_bHaveSegmented)
		return false;
	if (m_scaleInformation.find(curSCALE) == m_scaleInformation.end())//没有此信息，增加
		if(!BuildScaleInformation(curSCALE))
			return false;
	ScaleInformation scaleInfo = m_scaleInformation[curSCALE];
	unsigned char*OUT_BIP_IMAGE_DATA = scaleInfo.segRGBResult;
	//*
	GDALDataset* pDS = (GDALDataset*)GDALOpen(m_strFileName,GA_ReadOnly);
	if(!pDS)return false;
	int nBandCount = pDS->GetRasterCount();
	int nDstBandCount = 0;
	if (nBandCount >= 3)
		nDstBandCount = 3;
	else if(nBandCount == 1)
		nDstBandCount = 1;
	else
	{
		GDALClose(pDS);
		return false;
	}
	int XS = pDS->GetRasterXSize();
	int YS = pDS->GetRasterYSize();

	double padfGeoTransform[6];
	GDALGetGeoTransform(pDS,padfGeoTransform);
	unsigned char *b1= new unsigned char[XS*YS];

	for (int i=0;i<XS*YS;i++)
	{
		b1[i] = OUT_BIP_IMAGE_DATA[3*i+0];
	}

	//OGRSpatialReference *poDstSR = (OGRSpatialReference*)OGR_L_GetSpatialRef(hLayer);
	//if (poDstSR == NULL)
	//	return false;
	//const char* pszProjection = GDALGetProjectionRef((GDALDatasetH)pDS);
	//OGRSpatialReference *poSrcSR = (OGRSpatialReference*)OSRNewSpatialReference(pszProjection);
	//if(NULL == poSrcSR)
	//	return false;

	//OGRCoordinateTransformation *poCoorTrans = OGRCreateCoordinateTransformation(poSrcSR, poDstSR);
	//if (poCoorTrans == NULL)
	//{
	//	if(poSrcSR)
	//		OSRRelease(poSrcSR);
	//	return false;
	//}

	//CPolygonize poly(b1,XS,YS,(OGRLayer*)hLayer);
	//if(!poly.vectorize(padfGeoTransform,poCoorTrans))
	//	return false;
	if(b1)
		delete[] b1;

	return true;
}

bool RasterSegByMS::getSegImageResults(int curSCALE,const char* OUT_FILENAME)
{
	if (strlen(m_strFileName) == 0)
		return FALSE;
	if(!m_bHaveSegmented)
		return FALSE;
	if (m_scaleInformation.find(curSCALE) == m_scaleInformation.end())//没有此信息，增加
		if(!BuildScaleInformation(curSCALE))return FALSE;
	bool bShp = false;
	ScaleInformation scaleInfo = m_scaleInformation[curSCALE];
	unsigned char*OUT_BIP_IMAGE_DATA = scaleInfo.segRGBResult;
	//*
	GDALDataset* pDS = (GDALDataset*)GDALOpen(m_strFileName,GA_ReadOnly);
	if(!pDS)return false;
	int nBandCount = pDS->GetRasterCount();
	int nDstBandCount = 0;
	if (nBandCount >= 3)
		nDstBandCount = 3;
	else if(nBandCount == 1)
		nDstBandCount = 1;
	else
	{
		GDALClose(pDS);
		return false;
	}
	int XS = pDS->GetRasterXSize();
	int YS = pDS->GetRasterYSize();

	char* driverName = "";

	std::string strFL = OUT_FILENAME;

	std::string r4 = strupr((char*)strFL.substr(strFL.length()-4).c_str());

	/*判断输出影像类型格式*/
	if (r4 == ".TIF")
		driverName = "GTiff";
	else if (r4 == ".IMG")
		driverName = "HFA";
	else if (r4 == ".BMP")
		driverName = "BMP";
	else if (r4 == ".GIF")
		driverName = "GIF";
	else if (r4 == "JPEG" || r4 == "JPG")
		driverName = "JPEG";
	else if (r4 == "PAux")
		driverName = "PAux";
	else if (r4 == ".PNG")
		driverName = "PNG";
	else if (r4 == ".SHP")
		bShp = true;
	else
		driverName = "ENVI";

	if (strlen(driverName) == 0 && !bShp)
	{
		GDALClose(pDS);
		return false;
	}

	//char** option = NULL;

	//option = CSLSetNameValue(option,"INTERLEAVE","PIXEL");
	if(bShp)
	{
		//OGRSFDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName("ESRI Shapefile");
		//if(poDriver == NULL)
		//{
		//	return false;
		//}
		//remove(OUT_FILENAME);
		//DeleteFile((LPCWSTR)OUT_FILENAME);
		//OGRDataSource* poDstDS = poDriver->CreateDataSource(OUT_FILENAME, NULL);
		//if(poDstDS == NULL)
		//{
		//	return false;
		//}

		//double padfGeoTransform[6];
		//GDALGetGeoTransform(pDS,padfGeoTransform);
		//const char* pszProjection = GDALGetProjectionRef(pDS);
		//OGRSpatialReferenceH hSpatial = OSRNewSpatialReference(pszProjection);
		//OGRLayer* poLayer = poDstDS->CreateLayer("seg", (OGRSpatialReference*)hSpatial, wkbPolygon, NULL);

		//unsigned char *b1= new unsigned char[XS*YS];

		//for (int i=0;i<XS*YS;i++)
		//{
		//	b1[i] = OUT_BIP_IMAGE_DATA[3*i+0];
		//}

		//CPolygonize poly(b1,XS,YS,poLayer);
		//if(!poly.vectorize(padfGeoTransform))
		//	return false;
		//if(b1)
		//	delete[] b1;
		//if(hSpatial)
		//	OSRRelease(hSpatial);
		//OGRDataSource::DestroyDataSource(poDstDS);

	}
	else
	{
		GDALDriverH hDriver = GDALGetDriverByName(driverName);
		GDALDataset* pNew = (GDALDataset*)GDALCreate(hDriver,OUT_FILENAME,XS,YS,nDstBandCount,GDT_Byte,NULL/*,option*/);
		if (!pNew)
		{
			GDALClose(pDS);
			return false;
		}

		if(nDstBandCount == 3)
		{
			//BIP  ==>   BSQ
			unsigned char *b1= new unsigned char[XS*YS];
			unsigned char *b2= new unsigned char[XS*YS];
			unsigned char *b3= new unsigned char[XS*YS];

			for (int i=0;i<XS*YS;i++)
			{
				b1[i] = OUT_BIP_IMAGE_DATA[3*i+0];
				b2[i] = OUT_BIP_IMAGE_DATA[3*i+1];
				b3[i] = OUT_BIP_IMAGE_DATA[3*i+2];
			}

			GDALRasterBand* pBand = pNew->GetRasterBand(1);
			CPLErr er = pBand->RasterIO(GF_Write,0,0,XS,YS,b1,XS,YS,GDT_Byte,0,0);
			_ASSERT(er == CE_None);

			pBand = pNew->GetRasterBand(2);
			er = pBand->RasterIO(GF_Write,0,0,XS,YS,b2,XS,YS,GDT_Byte,0,0);
			_ASSERT(er == CE_None);

			pBand = pNew->GetRasterBand(3);
			er = pBand->RasterIO(GF_Write,0,0,XS,YS,b3,XS,YS,GDT_Byte,0,0);
			_ASSERT(er == CE_None);

			delete b1;
			delete b2;
			delete b3;
		}
		else if (nDstBandCount == 1)
		{
			GDALRasterBand* pBand = pNew->GetRasterBand(1);
			CPLErr er = pBand->RasterIO(GF_Write,0,0,XS,YS,OUT_BIP_IMAGE_DATA,XS,YS,GDT_Byte,0,0);
		}

		double padfT[6];
		CPLErr er = pDS->GetGeoTransform(padfT);
		if(er == CE_None)
		{
			er = pNew->SetGeoTransform(padfT);
			_ASSERT(er == CE_None);
		}

		const char* pDSS_Ref = pDS->GetProjectionRef();
		if (strlen(pDSS_Ref) > 0)
		{
			er = pNew->SetProjection(pDSS_Ref);
			_ASSERT(er == CE_None);
		}

		GDALClose(pNew);
		if(pDS)
			GDALClose(pDS);

	}
	//*/
	return true;
}


RegionList*	RasterSegByMS::getSegBoundariesPointList(int curSCALE)
{
	if(!m_bHaveSegmented)return NULL;
	if (m_scaleInformation.find(curSCALE) == m_scaleInformation.end())//没有此信息，增加
		if(!BuildScaleInformation(curSCALE))return NULL;

	ScaleInformation scaleInfo = m_scaleInformation[curSCALE];

	return scaleInfo.boundaryPointList;
	//return oEdison.GetSegmentBoundaries(curSCALE);
}

int			RasterSegByMS::getResultRegionsInfo(int curSCALE,int** OUT_LABELS,unsigned char** OUT_MODES,int**OUT_MODE_POINT_COUNTS)
{
	if(!m_bHaveSegmented)return 0;
	if (m_scaleInformation.find(curSCALE) == m_scaleInformation.end())//没有此信息，增加
		if(!BuildScaleInformation(curSCALE))return 0;

	ScaleInformation scaleInfo = m_scaleInformation[curSCALE];
	*OUT_LABELS = scaleInfo.pixelInObjectLabel;
	*OUT_MODES = scaleInfo.objectAverageRGB;
	*OUT_MODE_POINT_COUNTS = scaleInfo.pointCountInObject;
	return scaleInfo.numOfObject;
	//return oEdison.GetResultRegions(curSCALE,OUT_LABELS, OUT_MODES,OUT_MODE_POINT_COUNTS);
}


void		RasterSegByMS::buildSegRelationship(int NUM,int* SCALES)
{
	vector<Layer> allLayer;
	for (int i=0;i<(int)allLayer.size();i++)
	{
		Layer layer = allLayer[i];
		int nCount = layer.segCount;
		LayerMap layerMap = layer.segLayerMap;

	}
	//vector<int> GetSegChild(int curScale,int curLabel,int dstScale)
	//{


	//	vector<int> st;
	//	return st;
	//}

}
//////////////////////////////////////////////////////////////////////////
//  Private Functions
//////////////////////////////////////////////////////////////////////////
BOOL	RasterSegByMS::BuildScaleInformation(int scale)
{
	pair<int,ScaleInformation>tmp;
	unsigned char* OUT_BIP_IMAGE_DATA=NULL,*OUT_MODES = NULL;
	RegionList * rList;
	int *OUT_LABELS = NULL,*OUT_MODE_POINT_COUNTS = NULL;
	int num;
	BOOL bSuccess = oEdison.GetSegmentResults(scale,&OUT_BIP_IMAGE_DATA);
	rList = oEdison.GetSegmentBoundaries(scale);
	num = oEdison.GetResultRegions(scale,&OUT_LABELS, &OUT_MODES,&OUT_MODE_POINT_COUNTS);
	ScaleInformation sTmp;
	sTmp.boundaryPointList = rList;
	sTmp.numOfObject = num;
	sTmp.objectAverageRGB = OUT_MODES;
	sTmp.pixelInObjectLabel = OUT_LABELS;
	sTmp.pointCountInObject = OUT_MODE_POINT_COUNTS;
	sTmp.segRGBResult = OUT_BIP_IMAGE_DATA;

	m_scaleInformation.insert(pair<int,ScaleInformation>(scale,sTmp));
	return TRUE;
}



void	RasterSegByMS::SaveSegInformationDB(char *strFileName)
{
	FILE *fp;
	fopen_s(&fp,strFileName,"wb");
	_ASSERT(fp);

	//存储以下几个参数：width,height,dim,第一行，中间行，最后行（用于判断已经存储的数据是不是新的数据，如果是，可以跳过若干个步骤）
	size_t resu = fwrite(&m_WIDTH,(int)sizeof(int),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_HEIGHT,(int)sizeof(int),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_DIM,(int)sizeof(int),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_IN_BIP_IMAGE_DATA,(int)sizeof(unsigned char)*m_WIDTH*m_DIM,1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&(m_IN_BIP_IMAGE_DATA[m_WIDTH*m_HEIGHT*m_DIM/2]),(int)sizeof(unsigned char)*m_WIDTH*m_DIM,1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&(m_IN_BIP_IMAGE_DATA[m_WIDTH*(m_HEIGHT-1)*m_DIM]),(int)sizeof(unsigned char)*m_WIDTH*m_DIM,1,fp);
	_ASSERT(resu == 1);

	char str[11];
	strcpy_s(str,11,"LUV IMAGE ");
	resu = fwrite(str,(int)sizeof(char)*10,1,fp);
	_ASSERT(resu == 1);
	//存储DefineImage(RGBtoLUV)后的结果，大小为sizeof(float)*width*height*dim
	//oEdison.SaveLUVImage(fp);

	resu = fwrite(&m_USE_WEIGHT_MAP,(int)sizeof(bool),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_GRADIENT_WINDOW_RADIUS,(int)sizeof(int),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_EDGE_STRENGTH_THRESHOLD,(int)sizeof(float),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_MIXTURE_PARAMETER,(int)sizeof(float),1,fp);
	_ASSERT(resu == 1);
	//存储ComputeWeightMap的结果

	//存储以下参数：……用于判断用户分割参数是否一致，如果一致可直接获取Filter结果
	resu = fwrite(&m_SPATIAL_BANDWIDTH,(int)sizeof(int),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_COLOR_RANGE_BANDWIDTH,(int)sizeof(float),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_SPEEDUP,(int)sizeof(int),1,fp);
	_ASSERT(resu == 1);
	resu = fwrite(&m_SPEED_THRESHOLD,(int)sizeof(float),1,fp);
	_ASSERT(resu == 1);

	strcpy_s(str,11,"WEIGHT MAP");
	resu = fwrite(str,(int)sizeof(char)*10,1,fp);
	_ASSERT(resu == 1);
	//存储ComputeWeightMap后的结果，这个结果有意义的前提是m_USE_WEIGHT_MAP，m_GRADIENT_WINDOW_RADIUS，m_EDGE_STRENGTH_THRESHOLD
	//m_MIXTURE_PARAMETER几个参数一致

	strcpy_s(str,11,"FILTER MAP");
	resu = fwrite(str,(int)sizeof(char)*10,1,fp);
	_ASSERT(resu == 1);
	//存储FILTER后的结果
	fclose(fp);
}

bool		RasterSegByMS::rasterFileSegmentationByMeanShift(const char* inRasterFilename, 
	const char	*outRasterFilename,
	int			nScale,
	bool		bCreateShpFile /*= true*/,
	int			nBandCount /*= 0*/,
	int			*iBandList /*= NULL*/,
	int			nSpatialBandWidth /*= 7*/,
	float		fColorRangeBandWidth /*= 6.5*/,
	int			nSpeedUp  /*= MED_SPEEDUP*/,
	float		fSpeedThreshold /*= 0.1*/,
	bool		bUseWeightMap /*= true*/,
	int			nGradientWindowRadius /*= 2*/,
	float		fEdgeStrengthThreshold /*= 0.3*/,
	float		fMixtureParameter  /*= 0.3*/,
	bool		bAllowCache /*= true*/,
	const char* dbInfo/*=""*/)
{
	bool bSuc;
	bSuc = setSegParams(inRasterFilename,nBandCount,iBandList,nSpatialBandWidth,fColorRangeBandWidth,nSpeedUp,fSpeedThreshold,bUseWeightMap,nGradientWindowRadius,fEdgeStrengthThreshold,fMixtureParameter,bAllowCache);
	if(!bSuc)return false;
	bSuc = startSegByMS();
	if(!bSuc)return false;
	bSuc = getSegImageResults(nScale,outRasterFilename);
	if(!bSuc)return false;
	//if (bCreateShpFile)
	//{
	//	std::string outShp = outRasterFilename;
	//	int nFind = outShp.find_last_of(".");
	//	if(nFind > 0)
	//		outShp = outShp.substr(0,nFind);
	//	outShp += ".shp";
	//	//bSuc = executePolygonize(outRasterFilename,outShp.c_str());

	//	/*增加访问数据库的代码*/
	//	//CollectResult(dbInfo, outShp);
	//}
	return true;
}
//bool RasterSegByMS::executePolygonize(const char* pszSourceFromRasterFilename, const char* pszSourceToVectorShpFilename, bool bFourConnected/* = true*/, bool bUsedMaskBand/* = false*/)
//{
///*
//	if(strlen(pszSourceFromRasterFilename) == 0 || strlen(pszSourceToVectorShpFilename) == 0)
//		return false;
//	int nStart = -1,nEnd = -1;
//	for (int i=strlen(pszSourceToVectorShpFilename)-1;i>=0;i--)
//	{
//		if (pszSourceToVectorShpFilename[i] == '.')
//		{
//			nEnd = i;
//		}
//		if (pszSourceToVectorShpFilename[i] == '\\')
//		{
//			nStart = i+1;
//			break;
//		}
//	}
//	char name[256];
//	if(nStart > 0)
//		strcpy(name,pszSourceToVectorShpFilename + nStart);
//	if(nEnd > 0)
//		name[nEnd-nStart] = '\0';
//	GTRasterDataset *pDS = new GTRasterDataset;
//	bool bSuc = pDS->openDataSource(pszSourceFromRasterFilename,false);
//	if(!bSuc)return false;
//	std::string filename = "";
//	int XS = pDS->getXSize();
//	int YS = pDS->getYSize();
//	int NB = pDS->getRasterBandCount();
//	if(NB >= 3)
//	{
//		filename = pszSourceFromRasterFilename;
//		int nFind = filename.find_last_of('.');
//		if (nFind > 0)
//		{
//			filename = filename.substr(0,nFind);
//		}
//		filename += "_new.tif";
//		GTEnvelop env;
//		char* srswkt = NULL;
//		
//		pDS->getSpatialExtent(env);
//		if (pDS->getSpatialRefPtr()!=NULL)
//		{
//			pDS->getSpatialRefPtr()->exportToWkt(&srswkt);
//		}
//		
//		GTRasterDataset* pNewDS = GTGDALDataDriver::createDataSource("GTiff",filename.c_str(),pDS->getXResolution(),pDS->getYResolution(),env,srswkt,1,GTD_Float64);
//		GTS_FREE(srswkt);
//
//		if (!pNewDS)
//		{
//			GTRasterDataset::destroyGTRasterDataset(pDS);
//			return false;
//		}
//		_ASSERT(pDS->getRasterBandDataType(1) == GTD_Byte);
//		int nBand = min(pDS->getRasterBandCount(),3);
//		unsigned char *b1 = new unsigned char[XS*YS];
//		unsigned char *b2 = new unsigned char[XS*YS];
//		unsigned char *b3 = new unsigned char[XS*YS];
//		if (!b1 || !b2 || !b3)
//		{
//			GTRasterDataset::destroyGTRasterDataset(pDS);
//			GTRasterDataset::destroyGTRasterDataset(pNewDS);
//			return false;
//		}
//
//		GTRasterBand * pBand;
//		pBand = pDS->getRasterBand(1);
//		bSuc = pBand->getPixels(-1,0,0,XS,YS,b1);
//		_ASSERT(bSuc);
//		GTRasterBand::destroyGTRasterBand(pBand);
//
//		pBand = pDS->getRasterBand(2);
//		bSuc = pBand->getPixels(-1,0,0,XS,YS,b2);
//		_ASSERT(bSuc);
//		GTRasterBand::destroyGTRasterBand(pBand);
//
//		pBand = pDS->getRasterBand(3);
//		bSuc = pBand->getPixels(-1,0,0,XS,YS,b3);
//		_ASSERT(bSuc);
//		GTRasterBand::destroyGTRasterBand(pBand);
//
//		double *bNew = new double[XS*YS];
//		if(!bNew)
//		{
//			delete b1;delete b2;delete b3;
//			GTRasterDataset::destroyGTRasterDataset(pDS);
//			GTRasterDataset::destroyGTRasterDataset(pNewDS);
//			return false;
//		}
//		for (int i=0;i<XS*YS;i++)
//		{
//			bNew[i] = double(b1[i]*256.+b2[i]*1.+b3[i]*1./256);
//		}
//
//		pBand = pNewDS->getRasterBand(1);
//		pBand->setPixels(0,0,XS,YS,bNew);
//		GTRasterBand::destroyGTRasterBand(pBand);
//
//		delete b1;
//		delete b2;
//		delete b3;
//		delete bNew;
//		bSuc = GTRasterDataset::destroyGTRasterDataset(pNewDS);
//		bSuc = GTRasterizeAndPolygonize::executePolygonize(filename.c_str(),1,pszSourceToVectorShpFilename,name);
//	}
//	else
//	{
//		bSuc = GTRasterizeAndPolygonize::executePolygonize(pszSourceFromRasterFilename,1,pszSourceToVectorShpFilename,name);
//	}
//	GTRasterDataset::destroyGTRasterDataset(pDS);
//	enumGTDataSourceErr er;
//	if(filename.length() > 0 && access(filename.c_str(),0) == 0)
//		er = GTGDALDataDriver::deleteDataSource(filename.c_str(),"GTiff");
//	return bSuc;
////*/
//	return false;
//}

//bool RasterSegByMS::polygonize( const char* pszFile, const char* pszShpFile )
//{
//	GDALAllRegister();
//	OGRRegisterAll();
//
//	OGRSFDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName("ESRI Shapefile");
//	if(poDriver == NULL)
//	{
//		return false;
//	}
//	DeleteFile((LPCTSTR)pszShpFile);
//	OGRDataSource* poDstDS = poDriver->CreateDataSource(pszShpFile, NULL);
//	if(poDstDS == NULL)
//	{
//		return false;
//	}
//
//	GDALDatasetH segDS = GDALOpen(pszFile, GA_ReadOnly);
//	GDALRasterBandH band = GDALGetRasterBand(segDS,GDALGetRasterCount(segDS));
//	double padfGeoTransform[6];
//	GDALGetGeoTransform(segDS,padfGeoTransform);
//	const char* pszProjection = GDALGetProjectionRef(segDS);
//	OGRSpatialReferenceH hSpatial = OSRNewSpatialReference(pszProjection);
//	OGRLayer* poLayer = poDstDS->CreateLayer("seg", (OGRSpatialReference*)hSpatial, wkbMultiPolygon, NULL);
//
//	//OGRFieldDefnH   fieldDefn;
//	//fieldDefn = OGR_Fld_Create( "feature1", OFTReal );
//	//OGR_L_CreateField(poLayer, fieldDefn, 0);
//
//	char **papszOptions = NULL;
//	papszOptions = CSLSetNameValue( papszOptions, "8CONNECTED", "1");
//	//if(GDALPolygonize(band,NULL,(OGRLayerH)poLayer,-1,papszOptions,NULL,NULL) != CE_None)
//	//{
//	//	return false;
//	//}
//
//	if(hSpatial)
//		OSRRelease(hSpatial);
//	OGRDataSource::DestroyDataSource(poDstDS);
//	return true;
//}

bool RST_SegmentByMS( const char* pszInputImgDir, const char* pszOutputImgDir, int nScale )
{
	if(access(pszInputImgDir,0) != 0)
	{
		printf("can't find input file: %s\n", pszInputImgDir);
		return false;
	}
	if(nScale <= 0)
	{
		printf("bad scale: %d\n", nScale);
		return false;
	}

	RasterSegByMS ms;
	bool bSuc = ms.rasterFileSegmentationByMeanShift(pszInputImgDir,pszOutputImgDir,nScale);

	//删除分割生成的临时文件
	FILE* pFiledb = fopen("SinSeg001.db","r");
	if (pFiledb)
	{
		fclose(pFiledb);
		pFiledb=NULL;
		remove("SinSeg001.db");
	}
	return bSuc;
}
