/****************************************************************
*
* Project:  ������Ϣ�����߼�
* Purpose:  ���ֲ�Ķ�����
* Author:   ��ռ��, shenzf@irsa.ac.cn, 2011��-4��-8��
*
****************************************************************
* Copyright (c) All rights reserved.
* ��Ȩ����  (c) ��������Ȩ��
****************************************************************/

/** @file RasterSegByMS.h
 *  @brief ���ھ�ֵƯ���㷨��Ӱ���߶ȷָ�
 *  @author ��ռ��, shenzf@irsa.ac.cn
 *  @data 2011��-4��-8��
 ****************************************************************
 * Copyright (c) All rights reserved.
 * ��Ȩ����  (c) ��������Ȩ��
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


/*�������ڹ����߶ȼ��ϵ�Ľṹ��*/
typedef struct _SegStrcture
{
	int parentLabel;
	vector <int>childLabel;
}SegStructure;

/*ĳ���ָ�߶�����Ӧ�Ĳ������п�*/
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

	/*���غ���˵����ʹ�÷�����char**/
	char*		getFuncDescription();

	/*���÷ָ������ע��ΪBIP��ʽ����ʱ��UCHAR*��ʽ��1��3�����Σ�
	�ر�ע���һ������ IN_BIP_IMAGE_DATA �ڵ���֮ǰnew��������Ҫ�û��ͷ�
	��Ӧ�Ľ���ú���	BOOL		getSegImageResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA)��ȡ
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

	/*���÷ָ����������Ϊ�ļ���
	��Ӧ�Ľ���ú���	BOOL		getSegImageResults(int curSCALE,char* OUT_FILENAME)��ȡ*/
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

	/*��ʼ���зָ�*/
	bool		startSegByMS();

	/*��ȡ�ָ�������Ҫע��ڶ�������OUT_BIP_IMAGE_DATA��һ��3�����ε����ɫӰ��BIP��ʽ��
	����Ҫ�ⲿ�ͷţ�Ϊһ��XSize*YSize*3��Ӱ�����ݿ飬BIP��ʽ�����÷���Ϊ
	unsigned char* out = NULL;
	getSegImageResults(scale,out);
	...	*/
	bool		getSegImageResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA);

	/*��ȡ�ָ������ڶ�������ΪӰ����*/
	bool		getSegImageResults(int curSCALE,const char* OUT_FILENAME);

	bool addSegResultToLayer(int curSCALE,OGRLayerH hLayer);
	/*��ȡ�ָ����ı߽�㴮*/
	RegionList*	getSegBoundariesPointList(int curSCALE);

	/*��ȡ�ָ�����OUT_LABELSΪ���ǣ�OUT_MODESΪģʽ��OUT_MODE_POINT_COUNTSΪÿ��ģʽ��Ӧ�ĵ���Ŀ
	���У�OUT_LABELSΪһ��XS*YS��int�͵����飬ָʾ��ͬ��Ԫλ���ĸ�������
	OUT_MODE_POINT_COUNTSΪһ���������飬����Ϊ�˺�������ֵrValue����ֵΪ���������Ԫ����/���
	OUT_MODESΪһ��UCHAR���飬ָʾÿ�������RGBֵ������Ϊ�˺�������ֵrValue*3��*/
	int			getResultRegionsInfo(int curSCALE,int** OUT_LABELS,unsigned char** OUT_MODES,int**OUT_MODE_POINT_COUNTS);

	/*������Ӧ�ĳ߶ȼ�ݽ���ϵ*/
	void		buildSegRelationship(int NUM,int* SCALES);

	/*�ͷ���Ӧ�Ĳ�ͬ�߶���ռ�ڴ�ռ�*/
	void        freeScaleInformation();

	/*ֱ�ӽ���Ӱ��ָ�ĺ�����ʵ���������ϼ�������������*/
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
	//˽�б������û���ͨ������������������
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

	/*�洢�û�ָ���ĳ߶���Ϣ�����������Ӧ��Ϣ�������ȡ�����û�У�������*/
	map<int,ScaleInformation> m_scaleInformation;

	BOOL	BuildScaleInformation(int scale);
	void	SaveSegInformationDB(char *strFileName);
	/*ִ��ʸ����*/
	//bool executePolygonize(const char* pszSourceFromRasterFilename, const char* pszSourceToVectorShpFilename, bool bFourConnected = true, bool bUsedMaskBand = false);
	//bool polygonize(const char* pszFile, const char* pszShpFile);
};

extern "C" __declspec(dllexport) bool RST_SegmentByMS( const char* pszInputImgDir,
													   const char* pszOutputImgDir,
													   int nScale);

#endif