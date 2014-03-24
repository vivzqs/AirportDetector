////////////////////////////////////////////////////////
// Name     : edison.h
// Purpose  : Wrapper class used for segmenation and
//            edge detection.
// Author   : Chris M. Christoudias
// Modified by
// Created  : 03/20/2002
// Copyright: (c) Chris M. Christoudias
// Version  : v0.1
////////////////////////////////////////////////////////

#ifndef EDISON_H
#define EDISON_H

#include "../segm/msImageProcessor.h"
#include "../edge/BgImage.h"
#include "../edge/BgEdge.h"
#include "../edge/BgEdgeList.h"
#include "../edge/BgEdgeDetect.h"
#include "../edge/BgDefaults.h"

//define mean shift actions
enum 
{
	EDISON_FILTER,
	EDISON_FUSE,
	EDISON_SEGMENT
};

class EDISON 
{
public:

	//class constructor/destructor
	 EDISON( void );
	~EDISON( void );

	// *** Input/Output ***
	int Save(char *filename, int filetype, int outputtype);
	int Load(char *filename, int inputtype);
	int UseResult(int outputtype);

	// *** Set Parameters ***
	void SetParameters(void **parameterList);

	// *** Image Segmentation ***
	int Filter( void );
	int Fuse( void );
	int Segment( void );

	// *** Edge Detection ***
	int EdgeDetect( void );
	
	//***  ��ռ�� 2009.12.13����  **************************************************************************  
	//***  ���� BIP(RGB) ��ʽ���� DIM �����ε� unsigned char* ���� inBIPImageData,   ******************
	BOOL DoEdgeDetect(unsigned char* inBIPImageData,int DIM,int WIDTH,int HEIGHT,
						int GRAD_WIN = 2, int MIN_LENGTH = 5,
						int NON_SUP_TYPE = 0, float NON_SUP_RANK = 0.5, float NON_SUP_CONF = 0.5,
						int HIGH_TYPE = 0, float HIGH_RANK = 0.93, float HIGH_CONF = 0.96,
						int LOW_TYPE = 0, float LOW_RANK = 0.99, float LOW_CONF = 0.91);

	BOOL GetEdgeDetectResults(int **x,int **y, int* nNum);
	BOOL GetEdgeDetectResults(int *nLineNum,int **pLinePoint,int **point_x,int **point_y);

	//***  ��ռ�� 2009.11.20����  **************************************************************************  
	//***  ���� BIP(RGB) ��ʽ���� DIM �����ε� unsigned char* ���� inBIPImageData,   ******************
	//***  ���� NUM_SCALES_MINIMUM_REGION_AREA �� WIDTH*HEIGHT �ķָ�Ӱ������,   **********************
	//***  ���������� outSingleBand_ResultData ��(��Ҫ���ȷ����ڴ沢�Լ��ͷ�)    **********************
	//***  ����������ѡ������ SPEEDUP ��ȡ 2���ٶȿ쾫�ȵͣ���1���ٶ��о����У���1���ٶ������ȸߣ�   **
	//***  ��������ش�����룬���򷵻�FALSE    *********************************************************
	BOOL SetSegParams(unsigned char* inBIPImageData,
					  int DIM,
					  int WIDTH,
					  int HEIGHT,
					  int SPATIAL_BANDWIDTH = 7,
					  float RANGE_BANDWIDTH = 6.5,
					  int SPEEDUP  = 1,
					  float subSpeedUp = 0.1,
					  bool bUseWeightMap = true,
					  int GRADIENT_WINDOW_RADIUS = 2,
					  float EDGE_STRENGTH_THRESHOLD = 0.3,
					  float MIXTURE_PARAMETER  = 0.3,
					  bool bAllowCache = true,
					  bool bWaitBar = true);
	//***  ��ռ�� 2009.11.20����***************************************************************************
	//***  ��ʼ�ָ�************************************************************************************
	BOOL DoRSSegByMS();
	//***  ��ȡ�ָ���************************************************************************************
	BOOL GetSegmentResults(int curSCALE,unsigned char** OUT_BIP_IMAGE_DATA);
	//***  ��ȡ�ָ�߽�************************************************************************************
	RegionList*	GetSegmentBoundaries(int curSCALE);
	int 	GetResultRegions(int curSCALE,int** OUT_LABELS,unsigned char** OUT_MODES,int**OUT_MODE_POINT_COUNTS);
	void	SaveSegInformationDB(char* strFileName);

	void SetRange(int begin, int end);

private:

	//***  ��ռ�� 2009.11.20����  **************************************************************************  
	unsigned char* m_outResultData;
	msImageProcessor *piProc;
	float m_sigmaR;
	int   m_gradWindRad;
	float m_threshold   ;
	float m_mixture     ;
	bool  m_bUseWeightMap;
	int   m_sigmaS;
	float m_subSpeedUp;
	int  m_SPEEDUP;
	bool m_bHaveSaved;
	bool m_bAllowCache;//�Ƿ��������û���

	BgEdgeList   *m_pEdgeList;

	//store input image
	bool inputDefined_;
	int height_, width_, dim_;
	unsigned char *inputImage_;

	//store weight maps
	bool gradMapDefined_;
	bool confMapDefined_;
	bool weightMapDefined_;
	bool custMapDefined_;
	float *gradMap_;
	float *confMap_;
	float *weightMap_;
	float *custMap_;

	//store the output images
	bool filtImageDefined_;
	bool segmImageDefined_;
	unsigned char *filtImage_;
	unsigned char *segmImage_;

	//store the output edges and boundaries
	bool edgesDefined_;
	bool boundariesDefined_;
	int *edges_, numEdges_;
	int *boundaries_, numBoundaries_;

	//store the parameters
	void **parameterList_;
	FILE* fpSaveFile;

	int  m_wait_begin, m_wait_end;
	bool m_bWaitBar;

	long old_img_size;

	int writeImage(char *filename, unsigned char *image, int *dataPoints, int n, int filetype);
	int saveData(char *filename, float *data, int filetype);
	int loadImage(char *filename);
	int loadMap(char *filename);
	int ComputeWeightMap( void );
	//***   ��ռ��  2008.5.28   ***//
	//***   �����µķ�������Ȩ�� **//
	int ComputeWeightMap( int GRADIENT_WINDOW_RADIUS, float MIXTURE_PARAMETER );
	int meanShift(int action);
	void Refresh( void );
	int Output(unsigned char* out);
};

#endif
