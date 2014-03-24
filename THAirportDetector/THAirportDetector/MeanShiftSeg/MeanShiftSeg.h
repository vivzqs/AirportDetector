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

/** @file RasterSegByMS.h
 *  @brief 基于均值漂移算法的影像多尺度分割
 *  @author 沈占锋, shenzf@irsa.ac.cn
 *  @data 2011年-4月-8日
 ****************************************************************
 * Copyright (c) All rights reserved.
 * 版权所有  (c) 保留所有权利
 ****************************************************************/
#ifndef MEANSHIFTSEG_H_INCLUDE
#define MEANSHIFTSEG_H_INCLUDE
#pragma once
//#include ".\segm\tdef.h"
#include ".\segm\rlist.h"
#include ".\prompt\edison.h"
#include <map>
#include <vector>
#include <stack>
#include "gdal_priv.h"
#include "ogr_api.h"
using namespace std;


/*定义用于构建尺度间关系的结构体*/
typedef struct _SegStrcture
{
	int parentLabel;
	vector <int>childLabel;
}SegStructure;

/*某个分割尺度所对应的层内所有块*/
typedef map<int,SegStructure> LayerMap;
typedef struct _Layer
{
	int segCount;
	LayerMap segLayerMap;
}Layer;

typedef struct _ScaleInformation
{
	unsigned char*	segRGBResult;
	RegionList*		boundaryPointList;
	int*			pixelInObjectLabel;
	unsigned char*	objectAverageRGB;
	int*			pointCountInObject;
	int				numOfObject;
}ScaleInformation;

class RasterSegByMS
{
public:
	RasterSegByMS(void);
	~RasterSegByMS(void);

	/*返回函数说明及使用方法，char**/
	char*		getFuncDescription();

	/*设置分割参数，注意为BIP格式，暂时以UCHAR*形式，1或3个波段，
	特别注意第一个参数 IN_BIP_IMAGE_DATA 在调用之前new出，不需要用户释放
	对应的结果用函数	BOOL		getSegImageResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA)获取
	*/
	bool		setSegParams(unsigned char* IN_BIP_IMAGE_DATA,
							 int BANDCOUNT,
							 int WIDTH,
							 int HEIGHT,
							 int SPATIAL_BANDWIDTH = 7,
							 float COLOR_RANGE_BANDWIDTH = 6.5,
							 int SPEEDUP  = MED_SPEEDUP,
							 float SPEED_THRESHOLD = 0.1,
							 bool USE_WEIGHT_MAP = true,
							 int GRADIENT_WINDOW_RADIUS = 2,
							 float EDGE_STRENGTH_THRESHOLD = 0.3,
							 float MIXTURE_PARAMETER  = 0.3,
							 bool bAllowCache = true,
							 bool bWaitBar = false);

	/*设置分割参数，输入为文件名
	对应的结果用函数	BOOL		getSegImageResults(int curSCALE,char* OUT_FILENAME)获取*/
	bool		setSegParams(const char* IN_FILENAME,
							 int BANDCOUNT = 0,
							 int *iBandList = NULL,
							 int SPATIAL_BANDWIDTH = 7,
							 float COLOR_RANGE_BANDWIDTH = 6.5,
							 int SPEEDUP  = MED_SPEEDUP,
							 float SPEED_THRESHOLD = 0.1,
							 bool USE_WEIGHT_MAP = true,
							 int GRADIENT_WINDOW_RADIUS = 2,
							 float EDGE_STRENGTH_THRESHOLD = 0.3,
							 float MIXTURE_PARAMETER  = 0.3,
							 bool bAllowCache = true,
							 bool bWaitBar = false);

	/*开始进行分割*/
	bool		startSegByMS();

	/*获取分割结果，需要注意第二个参数OUT_BIP_IMAGE_DATA是一个3个波段的真彩色影像，BIP格式，
	不需要外部释放，为一个XSize*YSize*3的影像数据块，BIP格式，调用方法为
	unsigned char* out = NULL;
	getSegImageResults(scale,out);
	...	*/
	bool		getSegImageResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA);

	/*获取分割结果，第二个参数为影像名*/
	bool		getSegImageResults(int curSCALE,const char* OUT_FILENAME);

	bool addSegResultToLayer(int curSCALE,OGRLayerH hLayer);
	/*获取分割结果的边界点串*/
	RegionList*	getSegBoundariesPointList(int curSCALE);

	/*获取分割区域，OUT_LABELS为其标记，OUT_MODES为模式，OUT_MODE_POINT_COUNTS为每种模式对应的点数目
	其中，OUT_LABELS为一个XS*YS的int型的数组，指示不同像元位于哪个区域中
	OUT_MODE_POINT_COUNTS为一个整形数组，个数为此函数返回值rValue个，值为该区域的像元个数/面积
	OUT_MODES为一个UCHAR数组，指示每个区域的RGB值，个数为此函数返回值rValue*3个*/
	int			getResultRegionsInfo(int curSCALE,int** OUT_LABELS,unsigned char** OUT_MODES,int**OUT_MODE_POINT_COUNTS);

	/*构建相应的尺度间递进关系*/
	void		buildSegRelationship(int NUM,int* SCALES);

	/*释放相应的不同尺度所占内存空间*/
	void        freeScaleInformation();

	/*直接进行影像分割的函数，实际上是以上几个函数的联合*/
	bool		rasterFileSegmentationByMeanShift(const char* inRasterFilename, 
		const char	*outRasterFilename,
		int			nScale,
		bool		bCreateShpFile = true,
		int			nBandCount = 0,
		int			*iBandList = NULL,
		int			nSpatialBandWidth = 7,
		float		fColorRangeBandWidth = 6.5,
		int			nSpeedUp  = MED_SPEEDUP,
		float		fSpeedThreshold = 0.1,
		bool		bUseWeightMap = true,
		int			nGradientWindowRadius = 2,
		float		fEdgeStrengthThreshold = 0.3,
		float		fMixtureParameter  = 0.3,
		bool		bAllowCache = true,
		const char* dbInfo="");

private:
	//私有变量，用户可通过公共函数进行设置
	char*	m_strFileName;
	bool	m_bHaveSegmented;

	unsigned char* m_IN_BIP_IMAGE_DATA;
	int		m_DIM;
	int		m_WIDTH;
	int		m_HEIGHT;

	int		m_SPATIAL_BANDWIDTH;
	float	m_COLOR_RANGE_BANDWIDTH ;
	int		m_SPEEDUP  ;
	float	m_SPEED_THRESHOLD ;
	bool	m_USE_WEIGHT_MAP ;
	int		m_GRADIENT_WINDOW_RADIUS ;
	float	m_EDGE_STRENGTH_THRESHOLD ;
	float	m_MIXTURE_PARAMETER  ;
	bool	m_bAllowCache;
	bool    m_bWaitBar;

	EDISON	oEdison;

	/*存储用户指定的尺度信息，如果已有相应信息，则从中取，如果没有，则增加*/
	map<int,ScaleInformation> m_scaleInformation;

	BOOL	BuildScaleInformation(int scale);
	void	SaveSegInformationDB(char *strFileName);
	/*执行矢量化*/
	//bool executePolygonize(const char* pszSourceFromRasterFilename, const char* pszSourceToVectorShpFilename, bool bFourConnected = true, bool bUsedMaskBand = false);
	//bool polygonize(const char* pszFile, const char* pszShpFile);
};

extern "C" __declspec(dllexport) bool RST_SegmentByMS( const char* pszInputImgDir,
													   const char* pszOutputImgDir,
													   int nScale);

#endif