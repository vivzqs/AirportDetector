#include "StdAfx.h"
#include "afx.h"
#include <fstream>
#include "AirportDetector.h"
#include "opencv/cv.h"  
#include "opencv/highgui.h"  
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "gdal.h"
#include "gdal_priv.h"
#include "MeanShiftSeg/MeanShiftSeg.h"
#include "MeanShiftSeg/segm/msImageProcessor.h"
#include "SalientRegionDetector/Saliency.h"
#include "SalientRegionDetector/SymmetricSurroundSaliency.h"
#include "LocalEdgeDensitySaliency.h"

#define SAVE_INTERMEDIATE_RESULT 1

using namespace cv;
using namespace std;

AirportDetector::AirportDetector(void)
{
	nOriYsize = 1;
	nOriXsize = 1;
	nXsize    = 1;
	nYsize    = 1;
	scale     = 1;
}


AirportDetector::~AirportDetector(void)
{
}

BOOL AirportDetector::BlobLabling(const unsigned char* pBuffer, structPixel* pstructPixel, int& nPatches)
{
	if (pBuffer == NULL)
	{
		return 0;
	}
	nPatches = 0;
	if (pstructPixel == NULL)
	{
		pstructPixel = new structPixel[nXsize*nYsize];
	}

	for(int i=0;i<nYsize;i++)
	{
		for (int j=0; j<nXsize; j++)
		{
			pstructPixel[i*nXsize + j].nFlag = 0;
			pstructPixel[i*nXsize + j].nX = i;
			pstructPixel[i*nXsize + j].nY = j;
			pstructPixel[i*nXsize + j].nRegionNUM = 0;
			pstructPixel[i*nXsize + j].nNUM = 0;
			pstructPixel[i*nXsize + j].fThreshold = 0.0;
		}
	}
	//种子点
	structPixel structSeed;
	structSeed.nFlag = 0;
	structSeed.nRegionNUM = 0;
	structSeed.nX = 0;
	structSeed.nY = 0;
	structSeed.nNUM = 0;
	//当前处理的点
	structPixel structCurrent;
	structCurrent.nFlag = 0;
	structCurrent.nRegionNUM = 0;
	structCurrent.nX = 0;
	structCurrent.nY = 0;
	structCurrent.nNUM = 0;

	int nStart = 0;
	int nEnd = 0;
	int nXmax = 0;
	int nXmin = 0;
	int nYmax = 0;
	int nYmin = 0;
	//处理八邻域的数组,从正右方顺时针旋转
	int nArrayX[8] = {0,1,1,1,0,-1,-1,-1};
	int nArrayY[8] = {1,1,0,-1,-1,-1,0,1};
	//记录影像一共被分割成多少块
	int nPatchNUM=0;
	//控制种子点搜寻这个最大的循环，当图像上所有的点都被处理完毕，循环终止
	int nIFContinue = 1;
	while(nIFContinue)
	{
		//定义堆栈
		int *pGrowX= new int[nXsize*nYsize];
		int *pGrowY= new int[nXsize*nYsize];
		nPatchNUM++;
		nStart = 0;
		nEnd = 0;
		//第一步：搜索种子点，满足两块分割图像的边界处及像素未被处理过两个条件
		//判断是否已经全部处理完毕，处理完毕则终止循环
		int nNum = 0;
		for (int i=0;i<nYsize*nXsize;i++)
		{			
			if (pstructPixel[i].nFlag == 0)
			{
				nNum++;
			}			
		}
		if (nNum == 0)
		{
			nIFContinue = -1;
			delete []pGrowX;
			pGrowX = NULL;
			delete []pGrowY;
			pGrowY = NULL;
			break;
		}

		int n=0;
		for (int i=0;i<nYsize;i++)
		{
			for(int j=0;j<nXsize;j++)
			{
				if ((j-1)>=0)
				{
					if ((pBuffer[i*nXsize+j-1]!=pBuffer[i*nXsize+j])&&(pBuffer[i*nXsize+j]>0)&&(pstructPixel[i*nXsize+j].nFlag==0))
					{
						n=1;
						structSeed.nX=i;
						structSeed.nY=j;
						pstructPixel[i*nXsize+j].nFlag=1;
						pstructPixel[i*nXsize+j].nRegionNUM=nPatchNUM;
						i=nXsize;
						j=nYsize;
						break;
					}
				}				
			}
		}

		if ((n==0)&&(nNum>0))
		{
			for (int i=0;i<nYsize;i++)
			{
				for(int j=0;j<nXsize;j++)
				{
					if (pstructPixel[i*nXsize+j].nFlag==0)
					{
						structSeed.nX=i;
						structSeed.nY=j;
						pstructPixel[i*nXsize+j].nFlag=1;
						pstructPixel[i*nXsize+j].nRegionNUM=nPatchNUM;
						i=nXsize;
						j=nYsize;
						break;
					}
				}
			}
		}
		//第二步：基于选定的种子进行生长
		//种子点压栈
		pGrowX[nEnd] = structSeed.nX;
		pGrowY[nEnd] = structSeed.nY;
		//记录邻域像素坐标
		int nXNeighbor = 0;
		int nYNeighbor = 0;
		while(nStart<=nEnd)
		{
			//当前种子点坐标
			structCurrent.nX = pGrowX[nStart];
			structCurrent.nY = pGrowY[nStart];
			//种子点8邻域扫描
			for (int k=0; k<8;k++)
			{
				//当前计算的邻域坐标
				nXNeighbor = structCurrent.nX + nArrayX[k];
				nYNeighbor = structCurrent.nY + nArrayY[k];

				if ((nXNeighbor>=0)&&(nXNeighbor<nYsize)&&(nYNeighbor>=0)&&(nYNeighbor<nXsize))
				{
					if (pBuffer[structCurrent.nX*nXsize + structCurrent.nY]== pBuffer[nXNeighbor*nXsize + nYNeighbor])
					{
						if (pstructPixel[nXNeighbor*nXsize + nYNeighbor].nFlag==0)
						{					
							//将符合条件的像素压栈
							nEnd++;
							pGrowX[nEnd] = nXNeighbor;
							pGrowY[nEnd] = nYNeighbor;
						}
						pstructPixel[nXNeighbor*nXsize + nYNeighbor].nX = nXNeighbor;
						pstructPixel[nXNeighbor*nXsize + nYNeighbor].nY = nYNeighbor;
						pstructPixel[nXNeighbor*nXsize + nYNeighbor].nFlag = 1;
						pstructPixel[nXNeighbor*nXsize + nYNeighbor].nRegionNUM = nPatchNUM;
					}
				}
			}
			nStart++;
		}
		delete []pGrowX;
		pGrowX = NULL;
		delete []pGrowY;
		pGrowY = NULL;
	}

	for (int k=1; k<nPatchNUM;k++)
	{
		//定义堆栈,存储一个斑块的所有像素点
		int *pGrowX= new int[nXsize*nYsize];
		int *pGrowY= new int[nXsize*nYsize];
		int nEnd = 0;//像素数目为nEnd

		for(int i=0; i<nYsize;i++)
			for (int j=0;j<nXsize;j++)
			{
				if (pstructPixel[i*nXsize+j].nRegionNUM == k)
				{
					pGrowX[nEnd] = i;
					pGrowY[nEnd] = j;
					nEnd++;
					if (nEnd==1)
					{
						nXmax = i;
						nXmin = i;
						nYmax = j;
						nYmin = j;
					}
					if (i>nXmax)
					{
						nXmax=i;
					}
					if (i<nXmin)
					{
						nXmin = i;
					}
					if (j>nYmax)
					{
						nYmax = j;
					}
					if (j<nYmin)
					{
						nYmin = j;
					}
				}
			}
			for(int i=0; i<nYsize;i++)
				for (int j=0;j<nXsize;j++)
				{
					if (pstructPixel[i*nXsize+j].nRegionNUM == k)
					{
						pstructPixel[i*nXsize + j].nNUM = nEnd ;
						pstructPixel[i*nXsize + j].fThreshold = float(nEnd)/float((nXmax-nXmin)*(nYmax-nYmin));
						pstructPixel[i*nXsize + j].nXMAX = nXmax;
						pstructPixel[i*nXsize + j].nXMIN = nXmin;
						pstructPixel[i*nXsize + j].nYMAX = nYmax;
						pstructPixel[i*nXsize + j].nYMIN = nYmin;
						pstructPixel[i*nXsize + j].nFlag = 1;
					}
				}
				delete []pGrowX;
				pGrowX = NULL;
				delete []pGrowY;
				pGrowY = NULL;
	}
	nPatches = nPatchNUM;
	return 1;
}

void AirportDetector::Detect(CString inputFileName, CString outputFileName)
{
	if (inputFileName.IsEmpty() || outputFileName.IsEmpty())
	{
		return;
	}
	
	USES_CONVERSION;
	char* pszInputFile = T2A(inputFileName);
	char* pszOutputFile = T2A(outputFileName);
	Detect(pszInputFile,pszOutputFile);
	return ;
}

void AirportDetector::Normalize(const Mat& inI, Mat& outI)
{
	if (inI.channels() != 1)
	{
		return;
	}
	int rows = inI.rows;
	int cols = inI.cols;
	if (outI.data == NULL)
	{
		outI = Mat(rows,cols,CV_8U);
	}

	double minV, MaxV;
	minMaxLoc(inI,&minV,&MaxV);
	if (MaxV == 0.0)
	{
		MaxV = 0.00001;
	}
	inI.convertTo(outI,CV_8U,255/MaxV);
	return;
}

int AirportDetector::GDAL2CV(GDALDataType gdt)
{
	int CV_Type = CV_8U;
	switch (gdt)
	{
	case GDT_Byte:
		break;
	case GDT_Int16:
	case GDT_UInt16:
		CV_Type = CV_16S;
		break;
	case GDT_UInt32:
	case GDT_Int32:
		CV_Type = CV_32S;
		break;
	case GDT_Float32:
		CV_Type = CV_32F;
		break;
	case GDT_Float64:
		CV_Type = CV_64F;
		break;
	default:
		CV_Type = CV_32F;
		break;
	}
	return CV_Type;
}

GDALDataType AirportDetector::CV2GDAL(int CV_Type)
{
	GDALDataType dataType = GDT_Float32;
	switch(CV_Type)
	{
	case CV_8U:
		dataType = GDT_Byte;
		break;
	case CV_16S:
		dataType = GDT_Int16;
		break;
	case CV_32S:
		dataType = GDT_Int32;
		break;
	case CV_32F:
		dataType = GDT_Float32;
		break;
	case CV_64F:
		dataType = GDT_Float64;
		break;
	default:
		dataType = GDT_Float32;
		break;
	}
	return dataType;
}

void AirportDetector::Normalize(const double* inBuffer, unsigned char* outBuffer)
{
	if (inBuffer == NULL)
	{
		return;
	}
	
	double maxV = 0;
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		if (maxV < inBuffer[i])
		{
			maxV = inBuffer[i];
		}
	}

	double minV = maxV;
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		if (minV > inBuffer[i])
		{
			minV = inBuffer[i];
		}
	}
	if (outBuffer == NULL)
	{
		outBuffer = new unsigned char[nXsize*nYsize];
	}
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		outBuffer[i] = unsigned char(255*inBuffer[i]/maxV);
	}
	return;
}

void AirportDetector::ComputeNDVI(
	const char*						pszInputFile,
	float*							ndvi,
	const int						nXsize,
	const int						nYsize,
	const float						ndvi_thresh/* = 0.1f*/,
	const int						bandNir/* = 4*/, 
	const int						bandRed/* = 3*/)
{
	if (pszInputFile == NULL)
	{
		printf("函数：ComputeNDVI\n");
		printf("错误信息：输入文件路径为空。");
		return;
	}

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");

	GDALDataset* pSrcDataSet = (GDALDataset*)GDALOpen(pszInputFile,GA_ReadOnly);
	if (pSrcDataSet == NULL)
	{
		const char* msg;
		msg = CPLGetLastErrorMsg();
		printf("函数：ComputeNDVI\n");
		printf("无法打开文件，错误：\n %s",msg);
		return;
	}

	int nOriXsize = pSrcDataSet->GetRasterXSize();
	int nOriYsize = pSrcDataSet->GetRasterYSize();	
	if (nXsize > nOriXsize || nYsize > nOriYsize)
	{
		printf("函数：ComputeNDVI\n");
		printf("错误信息：访问文件越界\n");
		printf("	nXsize: %d    nYsize: %d",nXsize,nYsize);
		return;
	}
	float* pBufferNir = new float[nXsize*nYsize];
	float* pBufferRed = new float[nXsize*nYsize];

	pSrcDataSet->GetRasterBand(bandNir)->RasterIO(GF_Read,0,0,nOriXsize,nOriYsize,pBufferNir,nXsize,nYsize,GDT_Float32,0,0);
	pSrcDataSet->GetRasterBand(bandRed)->RasterIO(GF_Read,0,0,nOriXsize,nOriYsize,pBufferRed,nXsize,nYsize,GDT_Float32,0,0);

	GDALClose(pSrcDataSet);

	if (ndvi == NULL)
	{
		ndvi = new float[nXsize*nYsize];
	}
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		ndvi[i] = (pBufferNir[i] - pBufferRed[i])/(pBufferNir[i] + pBufferRed[i]);
	}
	if (ndvi_thresh == -1.0f)
	{
		return;
	}
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		if (ndvi[i] > ndvi_thresh || (pBufferNir[i] == 0 && pBufferRed[i] == 0))
		{
			ndvi[i] = 0.0f;
		}
		else
		{
			ndvi[i] = 1.0f;
		}
	}

	delete []pBufferNir; pBufferNir = NULL;
	delete []pBufferRed; pBufferRed = NULL;
	return;
}

void AirportDetector::ComputeNDWI(
	const char* pszInputFile, 
	float* ndwi, 
	const int nXsize, 
	const int nYsize, 
	const float ndwi_thresh /* = 0.1f */, 
	const int bandNir /* = 4 */,
	const int bandGrn /* = 2 */)
{
	if (pszInputFile == NULL)
	{
		printf("函数：ComputeNDVI\n");
		printf("错误信息：输入文件路径为空。");
		return;
	}

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");

	GDALDataset* pSrcDataSet = (GDALDataset*)GDALOpen(pszInputFile,GA_ReadOnly);
	if (pSrcDataSet == NULL)
	{
		const char* msg;
		msg = CPLGetLastErrorMsg();
		printf("函数：ComputeNDVI\n");
		printf("无法打开文件，错误：\n %s",msg);
		return;
	}

	int nOriXsize = pSrcDataSet->GetRasterXSize();
	int nOriYsize = pSrcDataSet->GetRasterYSize();	
	if (nXsize > nOriXsize || nYsize > nOriYsize)
	{
		printf("函数：ComputeNDVI\n");
		printf("错误信息：访问文件越界\n");
		printf("	nXsize: %d    nYsize: %d",nXsize,nYsize);
		return;
	}
	float* pBufferNir = new float[nXsize*nYsize];
	float* pBufferGrn = new float[nXsize*nYsize];

	pSrcDataSet->GetRasterBand(bandNir)->RasterIO(GF_Read,0,0,nOriXsize,nOriYsize,pBufferNir,nXsize,nYsize,GDT_Float32,0,0);
	pSrcDataSet->GetRasterBand(bandGrn)->RasterIO(GF_Read,0,0,nOriXsize,nOriYsize,pBufferGrn,nXsize,nYsize,GDT_Float32,0,0);

	GDALClose(pSrcDataSet);

	if (ndwi == NULL)
	{
		ndwi = new float[nXsize*nYsize];
	}
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		ndwi[i] = (pBufferGrn[i] - pBufferNir[i])/(pBufferNir[i] + pBufferGrn[i] +  0.0000001f);
	}
	if (ndwi_thresh == -1.0f)
	{
		return;
	}
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		if (ndwi[i] > ndwi_thresh || (pBufferGrn[i] == 0 && pBufferNir[i] == 0))
		{
			ndwi[i] = 0.0f;
		}
		else
		{
			ndwi[i] = 1.0f;
		}
	}

	delete []pBufferNir; pBufferNir = NULL;
	delete []pBufferGrn; pBufferGrn = NULL;
	return;
}

void AirportDetector::GetImgBuffer(
	const unsigned char* R, 
	const unsigned char* G, 
	const unsigned char* B, 
	int nWidth,
	int nHeight,
	vector<UINT>& imgBuffer)
{
	if (R == NULL || G == NULL || B == NULL)
	{
		return;
	}
	int imgSize = nWidth*nHeight;
	imgBuffer.resize(imgSize);

	UINT tmp = 0;
	for (int i = 0; i<nHeight; i++)
	{
		for (int j=0; j<nWidth; j++)
		{
			tmp = 0;	
			tmp  = (UINT)B[j*nWidth + j];
			tmp <<= 8;
			tmp += (UINT)G[j*nWidth + j];
			tmp <<= 8;
			tmp += (UINT)R[j*nWidth + j];

			imgBuffer[i*nWidth + j] = tmp;
		}
	}
}

void AirportDetector::GetImgBuffer(
	const Mat&								img,
	int&									nWidth,
	int&									nHeight,
	vector<UINT>&							imgBuffer)
{
	if (img.empty())
	{
		printf("错误信息：数据为空\n");
		return;
	}
	if (img.depth() != CV_8U)
	{
		printf("错误信息：");
	}


	nWidth			= img.cols;
	nHeight			= img.rows;
	long imgSize	= nWidth * nHeight;

	imgBuffer.resize(imgSize);

	UINT tmp = 0;
	if (img.channels() == 3)
	{
		for (int i = 0; i<nHeight; i++)
		{
			uchar* _img = (uchar*)(img.ptr<uchar>(i));
			for (int j=0; j<nWidth; j++)
			{
				tmp = 0;	
				tmp  = (UINT)_img[j*img.channels() + 0];
				tmp <<= 8;
				tmp += (UINT)_img[j*img.channels() + 1];
				tmp <<= 8;
				tmp += (UINT)_img[j*img.channels() + 2];

				imgBuffer[i*nWidth + j] = tmp;
			}
		}
	}
	else if (img.channels() == 1)
	{
		for (int i = 0; i<nHeight; i++)
		{
			uchar* _img = (uchar*)(img.ptr<uchar>(i));;
			for (int j=0; j<nWidth; j++)
			{
				tmp = 0;	
				tmp  = (UINT)_img[j];
				tmp <<= 8;
				tmp += (UINT)_img[j];
				tmp <<= 8;
				tmp += (UINT)_img[j];

				imgBuffer[i*nWidth + j] = tmp;
			}
		}
	}

	return;
}

void AirportDetector::DoMeanShiftSegmentation(
	const vector<UINT>&							inputImg, 
	const int&									width, 
	const int&									height, 
	vector<UINT>&								segimg, 
	const int&									sigmaS, 
	const float&								sigmaR, 
	const int&									minRegion, 
	vector<int>&								labels, 
	int&										numlabels)
{
	//int sz = width*height;
	//BYTE* bytebuff = new BYTE[sz*3];
	//{int i(0);
	//for( int p = 0; p < sz; p++ )
	//{
	//	bytebuff[i+0] = inputImg[p] >> 16 & 0xff;
	//	bytebuff[i+1] = inputImg[p] >>  8 & 0xff;
	//	bytebuff[i+2] = inputImg[p]       & 0xff;
	//	i += 3;
	//}}
	//msImageProcessor mss;
	//mss.DefineImage(bytebuff, COLOR, height, width);		
	//mss.Segment(sigmaS, sigmaR, minRegion, HIGH_SPEEDUP);
	//mss.GetResults(bytebuff);

	//int* p_labels = new int[sz];
	//numlabels = mss.GetLabels(p_labels);
	//labels.resize(sz);
	//for( int n = 0; n < sz; n++ ) labels[n] = p_labels[n];
	//if(p_labels) delete [] p_labels;

	//segimg.resize(sz);
	//int bsz = sz*3;
	//{int i(0);
	//for( int p = 0; p < bsz; p += 3 )
	//{
	//	segimg[i] = bytebuff[p] << 16 | bytebuff[p+1] << 8 | bytebuff[p+2];
	//	i++;
	//}}
	//if(bytebuff) delete [] bytebuff;
}

BOOL AirportDetector::DoMeanShiftSegmentation(
	const char*								pszInputFile, 
	const uchar*							pmask,
	const int&								width, 
	const int&								height, 
	const int&								nScale, 
	structPixel*&							pstructPixel,
	int&									numlables)
{
	if (pszInputFile == NULL)
	{
		printf("错误信息：输入文件路径为空！\n");
		return 0;
	}

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");

	GDALDataset* pDataSet = (GDALDataset*)GDALOpen(pszInputFile,GA_ReadOnly);
	if (pDataSet == NULL)
	{
		printf("错误信息：文件打开失败!\n");
		return 0;
	}

	int nWidth				= pDataSet->GetRasterXSize();
	int nHeight				= pDataSet->GetRasterYSize();
	int nBandCount			= pDataSet->GetRasterCount();
	GDALDataType dataType	= pDataSet->GetRasterBand(1)->GetRasterDataType();
	if (nBandCount < 4)
	{
		printf("错误信息：请输入多光谱数据\n");
		GDALClose(pDataSet);
		return 0;
	}
	if (dataType != GDT_Byte)
	{
		printf("错误信息：数据类型应为Byte型!\n");
		GDALClose(pDataSet);
		return 0;
	}

	string str = pszInputFile;
	int nLength = str.length();
	int nPos = 0;
	for (int i = (nLength -1); i>=0; i--)
	{
		if (pszInputFile[i] == '.')
		{
			nPos = i;
			break;
		}
	}
	string strPrefix = str.substr(0,nPos);
	string strTmpFile = strPrefix + "_temp.tif";
	string strSegFile = strPrefix + "_Seg.tif";
	uchar* pBuffer = new uchar[width*height];
	GDALDriver* pTempDriver = (GDALDriver*)GDALGetDriverByName("GTiff");
	GDALDataset* pTempDataSet =(GDALDataset*)pTempDriver->Create(strTmpFile.c_str(),width,height,nBandCount,GDT_Byte,NULL);

	for (int band = 1; band <= nBandCount; band ++)
	{
		pDataSet->GetRasterBand(band)->RasterIO(GF_Read,0,0,nWidth,nHeight,pBuffer,width,height,GDT_Byte,0,0);
		for (int i = 0; i<width*height; i++)
		{
			if (pmask[i] < 1)
			{
				pBuffer[i] = 0;
			}
		}
		pTempDataSet->GetRasterBand(band)->RasterIO(GF_Write,0,0,width,height,pBuffer,width,height,GDT_Byte,0,0);
	}
	GDALClose(pTempDataSet);

	printf("正在进行 MeanShift 分割...\n");
	RST_SegmentByMS(strTmpFile.c_str(),strSegFile.c_str(),nScale);
	printf("分割完成！\n");

	GDALDataset* pSegDataSet = (GDALDataset*)GDALOpen(strSegFile.c_str(),GA_ReadOnly);
	pSegDataSet->GetRasterBand(1)->RasterIO(GF_Read,0,0,width,height,pBuffer,width,height,GDT_Byte,0,0);
	if (pstructPixel == NULL)
	{
		structPixel* pstructPixel = new structPixel[width*height];
	}
	
	BlobLabling(pBuffer,pstructPixel,numlables);

	delete []pBuffer; pBuffer = NULL;

	printf("删除临时文件。\n");
	FILE* pTmpFile = fopen(strTmpFile.c_str(),"r");
	if (pTmpFile)
	{
		fclose(pTmpFile);
		pTmpFile=NULL;
		remove(strTmpFile.c_str());
	}

	FILE* pSegFile = fopen(strSegFile.c_str(),"r");
	if (pSegFile)
	{
		fclose(pSegFile);
		pSegFile=NULL;
		remove(strSegFile.c_str());
	}

	return 1;
}
void AirportDetector::BackgroundRemove(
	const vector<double>&					salmap, 
	const int&								width, 
	const int&								height, 
	const vector<int>&						labels, 
	const int&								numlabels, 
	vector<bool>&							choose)
{
	int sz = width*height;
	if (labels.size() <= 1 || numlabels <= 1)
	{
		double sum = 0;
		for (int i = 0; i<sz; i++)
		{
			sum += salmap[i];
		}
		double m = sum/sz;
		choose.resize(sz);
		for (int i = 0; i<sz; i++)
		{
			if (salmap[i] > m*2)
			{
				choose[i] = true;
			}
			else
			{
				choose[i] = false;
			}
		}
		return;
	}

	vector<double> salperseg(numlabels,0);
	vector<int> segsz(numlabels,0);
	vector<bool> touchborders(numlabels, false);
	{int i(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			salperseg[labels[i]] += salmap[i];
			segsz[labels[i]]++;

			if(false == touchborders[labels[i]] && (j == height-1 || j == 0 || k == width-1 || k == 0) )
			{
				touchborders[labels[i]] = true;
			}
			i++;
		}
	}}

	double avgimgsal(0);
	{for( int n = 0; n < numlabels; n++ )
	{
		if(true == touchborders[n])
		{
			salperseg[n] = 0;
		}
		else
		{
			avgimgsal += salperseg[n];
			salperseg[n] /= segsz[n];
		}
	}}


	avgimgsal /= sz;

	vector<bool> segtochoose(numlabels, false);
	{for( int n = 0; n < numlabels; n++ )
	{
		if( salperseg[n] > (avgimgsal+avgimgsal) ) segtochoose[n] = true;
	}}

	choose.resize(sz, false);
	bool atleastonesegmentchosent(false);
	{for( int s = 0; s < sz; s++ )
	{
		{
			choose[s] = segtochoose[labels[s]];
			atleastonesegmentchosent = choose[s];
		}
	}}

	if( false == atleastonesegmentchosent )
	{
		int maxsalindex(-1);
		double maxsal(DBL_MIN);
		for( int n = 0; n < numlabels; n++ )
		{
			if( maxsal < salperseg[n] )
			{
				maxsal = salperseg[n];
				maxsalindex = n;
			}
		}
		for( int s = 0; s < sz; s++ )
		{
			if(maxsalindex == labels[s]) choose[s] = true;
		}
	}
}

void AirportDetector::GetMatFromFile(
	const char*								pszInputFileName,
	const int&								width,
	const int&								height,
	const int*								bands,
	const int&								nBands,
	Mat&									img)
{
	if (pszInputFileName == NULL)
	{
		printf("错误信息：输入文件路径为空！\n");
		return;
	}

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");

	GDALDataset* pDataSet = (GDALDataset*)GDALOpen(pszInputFileName,GA_ReadOnly);
	if (pDataSet == NULL)
	{
		printf("错误信息：文件打开失败!\n");
		return;
	}

	int nWidth = pDataSet->GetRasterXSize();
	int nHeight = pDataSet->GetRasterYSize();
	int nBandCount = pDataSet->GetRasterCount();
	GDALDataType dataType = pDataSet->GetRasterBand(1)->GetRasterDataType();
	double* pBuffer = new double[width*height];
	img = cv::Mat::zeros(height,width,CV_MAKE_TYPE(CV_8U,nBands));
	if (nBands == 3)
	{		
		vector<cv::Mat> rgb(0);
		for (int k = 0; k<nBands; k++)
		{
			cv::Mat tmp = cv::Mat::zeros(height,width,CV_8U);
			if (dataType == GDT_Byte)
			{
				pDataSet->GetRasterBand(bands[k])->RasterIO(GF_Read,0,0,nWidth,nHeight,tmp.data,width,height,GDT_Byte,0,0);
			}
			else
			{
				pDataSet->GetRasterBand(bands[k])->RasterIO(GF_Read,0,0,nWidth,nHeight,pBuffer,width,height,GDT_Float64,0,0);

				double minV = 2048.0;
				double maxV = 0.0;
				for (int i = 0; i<width*height; i++)
				{
					if (minV > pBuffer[i])
					{
						minV = pBuffer[i];
					}
					if (maxV < pBuffer[i])
					{
						maxV = pBuffer[i];
					}
				}

				for (int i = 0; i<height; i++)
				{
					unsigned char* _tmp = (unsigned char* )tmp.ptr<unsigned char>(i);
					for (int j = 0; j<width; j++)
					{
						_tmp[j] = (unsigned char)(255*pBuffer[i*width + j]/(maxV-minV) - 255*minV/(maxV - minV));
					}
				}
				equalizeHist(tmp,tmp);
			}			
			rgb.push_back(tmp);
		}
		merge(rgb,img);
		rgb.clear();
	}
	else if (nBands == 1)
	{
		if (GDT_Byte == dataType)
		{
			pDataSet->GetRasterBand(bands[0])->RasterIO(GF_Read,0,0,nWidth,nHeight,img.data,width,height,GDT_Byte,0,0);
		}
		else
		{
			pDataSet->GetRasterBand(bands[0])->RasterIO(GF_Read,0,0,nWidth,nHeight,pBuffer,width,height,GDT_Float64,0,0);

			double minV = 2048.0;
			double maxV = 0.0;
			for (int k = 0; k<width*height; k++)
			{
				if (minV > pBuffer[k])
				{
					minV = pBuffer[k];
				}
				if (maxV < pBuffer[k])
				{
					maxV = pBuffer[k];
				}
			}

			for (int i = 0; i<height; i++)
			{
				unsigned char* _img = (unsigned char* )img.ptr<unsigned char>(i);
				for (int j = 0; j<width; j++)
				{
					_img[j] = (unsigned char)(255*pBuffer[i*width + j]/maxV);
				}
			}
			equalizeHist(img,img);
		}
	}

	delete []pBuffer;
	GDALClose(pDataSet);
	return;
}
void AirportDetector::Preprocessing(
	unsigned char*							pBuffer,
	int										width,
	int										height)
{
	if (pBuffer == NULL)
	{
		printf("错误信息：输入数据为空！");
		return;
	}

	int nPatches = 0;
	structPixel*  pstructPixel = new structPixel[width*height];

	BlobLabling(pBuffer,pstructPixel,nPatches);
	delete []pstructPixel; pstructPixel = NULL;
	
	for (int i = 0; i<width*height; i++)
	{
		if (pstructPixel[i].nNUM < 100)
		{
			pstructPixel[i].nFlag = 1;
			pBuffer[i] = 0;
		}
		else
		{
			pstructPixel[i].nFlag = 2;	
		}
	}

	return;

}


void AirportDetector::Detect(const char* pszInputFile, const char* pszOutputFile)
{
	int nScale = 40;
	int downScale = 3000;
	if (pszInputFile == NULL || pszOutputFile == NULL)
	{
		return;
	}

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");
	
	GDALDataset* pSrcDataSet = (GDALDataset*)GDALOpen(pszInputFile,GA_ReadOnly);
	if (pSrcDataSet == NULL)
	{
		const char* msg;
		msg = CPLGetLastErrorMsg();
		printf("无法打开文件，错误： %s\n",msg);
		return;
	}

	nOriXsize = pSrcDataSet->GetRasterXSize();
	nOriYsize = pSrcDataSet->GetRasterYSize();
	int nBandCount = pSrcDataSet->GetRasterCount();
	double geoTransform[6];
	pSrcDataSet->GetGeoTransform(geoTransform);
	const char* proRef = pSrcDataSet->GetProjectionRef();
	char ** papszMdretadata = pSrcDataSet->GetDriver()->GetMetadata();

	GDALDataType dataType = pSrcDataSet->GetRasterBand(1)->GetRasterDataType();
	if (nBandCount < 4)
	{
		printf("错误信息：只支持多光谱数据！\n");
		GDALClose(pSrcDataSet);
		return;
	}

	scale  = 1;
	nXsize = nOriXsize;
	nYsize = nOriYsize;

	while(nXsize*nYsize > downScale*downScale)
	{
		nXsize /= 2;
		nYsize /= 2;
		nScale *= 2;
	}

	string str = pszOutputFile;
	int nLength = str.length();
	int nPos = 0;
	for (int i = (nLength -1); i>=0; i--)
	{
		if (pszOutputFile[i] == '.')
		{
			nPos = i;
			break;
		}
	}

	string strPrefix = str.substr(0,nPos);
	string strTmpFile = strPrefix + "_temp.tif";
	string strSegFile = strPrefix + "_seg.tif";
	const char* pszTmpFile = strTmpFile.c_str();
	const char* pszSegFile = strSegFile.c_str();

	float* ndvi_mask = new float[nXsize*nYsize];
	float* ndwi_mask = new float[nXsize*nYsize];
	ComputeNDVI(pszInputFile,ndvi_mask,nXsize,nYsize,0.1f);
	ComputeNDWI(pszInputFile,ndwi_mask,nXsize,nYsize,0.1f);

#if SAVE_INTERMEDIATE_RESULT
	string strNDVI = strPrefix + "_NDVI.png";
	Mat ndvi = Mat::zeros(nYsize,nXsize,CV_8U);
	for (int i = 0; i<nYsize; i++)
	{
		uchar* _ndvi = ndvi.ptr<uchar>(i);
		for (int j = 0; j<nXsize; j++)
		{
			if (ndvi_mask[i*nXsize + j] > 0)
			{
				_ndvi[j] = 255;
			}
		}
	}
	imwrite(strNDVI,ndvi);
#endif

#if SAVE_INTERMEDIATE_RESULT
	string strNDWI = strPrefix + "_NDWI.png";
	Mat ndwi = Mat::zeros(nYsize,nXsize,CV_8U);
	for (int i = 0; i<nYsize; i++)
	{
		uchar* _ndwi = ndwi.ptr<uchar>(i);
		for (int j = 0; j<nXsize; j++)
		{
			if (ndwi_mask[i*nXsize + j] > 0)
			{
				_ndwi[j] = 255;
			}
		}
	}
	imwrite(strNDWI,ndwi);
#endif

	vector<UINT>	imgBuffer(0);
	vector<UINT>	segimg(0);
	vector<int>		lables(0);
	vector<double>	salMap(0);
	vector<bool>    forceground;
	int width		= nXsize;
	int height		= nYsize;
	int sigmaS		= 7;
	float sigmaR	= 10.0f;
	int minRegion   = 100;
	int bands[3]    = {3,2,1};
	int nBands      = 3;
	int numlables   = 0;
	cv::Mat img;
	unsigned char* pBuffer = new unsigned char[width*height];
	GetMatFromFile(pszInputFile,width,height,bands,nBands,img);	
	GetImgBuffer(img,width,height,imgBuffer);

#if SAVE_INTERMEDIATE_RESULT
	string strImg = strPrefix + "_RGB.png";
	imwrite(strImg,img);
#endif

	//LocalEdgeDensitySaliency led;
	//led.ComputeSaliencyMap(img,width,height,salMap);
	//Saliency sal;
	//sal.GetSaliencyMap(imgBuffer,width,height,salMap,true);
	SymmetricSurroundSaliency sssal;
	sssal.ComputeMaximumSymmetricSurroundSaliency(imgBuffer,width,height,salMap,true);

	for (int i = 0; i<width*height; i++)
	{
		if (ndvi_mask[i] < 1 || ndwi_mask[i] < 1)
		{
			salMap[i] = 0;
		}
	}

#if SAVE_INTERMEDIATE_RESULT
	string strSalFile = strPrefix + "_saliency.png";
	Mat salMat = Mat::zeros(nYsize,nXsize,CV_8U);
	for (int i = 0; i<nYsize; i++)
	{
		uchar* _salMat = salMat.ptr<uchar>(i);
		for (int j = 0; j<nXsize; j++)
		{
			_salMat[j] = (uchar)salMap[i*nXsize + j];
		}
	}
	imwrite(strSalFile,salMat);
#endif
	uchar* pMask = new uchar[nXsize*nYsize];
	for (int i = 0; i<nXsize*nYsize; i++)
	{
		if (ndwi_mask[i] > 1 && ndvi_mask[i] > 1)
		{
			pMask[i] = 1;
		}
		else
		{
			pMask[i] = 0;
		}
	}

	structPixel* pstructPixel = new structPixel[nXsize*nYsize];
	//if (DoMeanShiftSegmentation(pszInputFile,pMask,nXsize,nYsize,nScale,pstructPixel,numlables) == 0)
	//{
	//	printf("错误信息：分割失败！\n");
	//	return;
	//}

	//lables.resize(nXsize*nYsize);
	//for (int i = 0; i<nXsize*nYsize; i++)
	//{
	//	lables[i] = pstructPixel[i].nRegionNUM;
	//}
	BackgroundRemove(salMap,width,height,lables,numlables,forceground);	

#if 1
	Mat fg = Mat::zeros(height,width,CV_8U);
	for (int i = 0; i<height; i++)
	{
		uchar* _fg = fg.ptr<uchar>(i);
		for (int j = 0; j<width; j++)
		{
			if (forceground[i*width + j])
			{
				_fg[j] = 255;
			}		
		}
	}
	string strFgFile = strPrefix + "_forceground.png";
	imwrite(strFgFile,fg);
#endif

	for (int i = 0; i<width*height; i++)
	{
		if (/*salMap[i] > 1*/ forceground[i])
		{
			pBuffer[i] = 255;	
		}
	}

	cv::Mat iimg = cv::Mat::zeros(nYsize,nXsize,CV_8U);
	for (int i = 0; i<nYsize; i++)
	{
		uchar* _iimg = (uchar*)(iimg.ptr<uchar>(i));
		for (int j = 0; j<nXsize; j++)
		{
			_iimg[j] = pBuffer[i*nXsize + j];
		}
	}

#if SAVE_INTERMEDIATE_RESULT
	string strIimgFile = strPrefix + "_iimg.png";
	imwrite(strIimgFile,iimg);
#endif

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cv::findContours(iimg,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Mat drawing = Mat::zeros( iimg.size(), CV_8UC3 );
	Mat candidate = Mat::zeros( iimg.size(), CV_8U );
	vector<double> shape_index(0);
	for( unsigned int i = 0; i< contours.size(); i++ )
	{

		double area = contourArea(contours[i],false);
		unsigned int sz = contours[i].size();
		double perimeter = arcLength(contours[i],true);
		double shape = perimeter/4*sqrt(double(sz));
		cv::RotatedRect box = minAreaRect(contours[i]);
		Rect r = box.boundingRect();
		float pre = 2*(r.width+r.height);

		float rectIndex = float(perimeter)/pre;
		if (sz > 100 && sz < 5000 && rectIndex < 2)
		{
			shape_index.push_back(rectIndex);
			Mat contour_img = Mat::zeros(iimg.rows,iimg.cols,CV_8U);
			drawContours( contour_img, contours, i, Scalar(255), -1, 8, hierarchy, 0, Point());
		
			vector<cv::Vec4i> lines;
			cv::HoughLinesP(contour_img, lines, 3.5, 6*CV_PI/180, 50, 40, 5);

			if (lines.size() > 1)
			{
				drawContours(candidate,contours,i,Scalar(255), -1, 8, hierarchy, 0, Point());
				Scalar color = Scalar( (rand()&255), (rand()&255), (rand()&255) );
				drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
//				for( size_t i = 0; i < lines.size(); i++ )
//				{
//					line(drawing, Point(lines[i][0], lines[i][1]),
//						Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );			
//				}				
			}
		}
	}
#if 1
	string strConFile = strPrefix + "_contours.png";
	imwrite(strConFile,drawing);
#endif

	Mat fullSizeImg = Mat::zeros( nOriYsize,nOriXsize, CV_8U );
	resize(candidate,fullSizeImg,Size(nOriXsize,nOriYsize),(0,0),(0,0),INTER_NEAREST);
	unsigned char* pBufferAP = new unsigned char[nOriXsize*nOriYsize];
	for (int i = 0; i<nOriYsize; i++)
	{
		uchar* _fullSizeImg = fullSizeImg.ptr<uchar>(i);
		for (int j = 0; j<nOriXsize; j++)
		{
			if (_fullSizeImg[j] > 0)
			{
				pBufferAP[i*nOriXsize + j] = 1;
			}
			else
			{
				pBufferAP[i*nOriXsize + j] = 0;
			}
		}
	}
	GDALDriver* pOutputDriver = (GDALDriver*)GDALGetDriverByName("GTiff");	
	GDALDataset* pOutputDataSet = (GDALDataset*)pOutputDriver->Create(pszOutputFile,nOriXsize,nOriYsize,1,GDT_Byte,papszMdretadata);
	pOutputDataSet->SetGeoTransform(geoTransform);
	pOutputDataSet->SetProjection(proRef);	
	pOutputDataSet->GetRasterBand(1)->RasterIO(GF_Write,0,0,nOriXsize,nOriYsize,pBufferAP,nOriXsize,nOriYsize,GDT_Byte,0,0);
	pOutputDataSet->FlushCache();
	GDALClose(pOutputDataSet);

	FILE* pTmpFile = fopen(pszTmpFile,"r");
	if (pTmpFile)
	{
		fclose(pTmpFile);
		pTmpFile=NULL;
		remove(pszTmpFile);
	}
	FILE* pSegFile = fopen(pszSegFile,"r");
	if (pSegFile)
	{
		fclose(pSegFile);
		pSegFile=NULL;
		remove(pszSegFile);
	}

	delete []pBuffer;   pBuffer = NULL;
	delete []pBufferAP; pBufferAP = NULL;
	delete []ndvi_mask; ndvi_mask = NULL;
	delete []ndwi_mask; ndwi_mask = NULL;
	delete []pMask;		pMask = NULL;
	delete []pstructPixel; pstructPixel = NULL;
	return;
}