////////////////////////////////////////////////////////
// Name     : libppm.cpp
// Purpose  : Read/Write Portable Pixel Map images
// Author   : Chris M. Christoudias
// Modified by
// Created  : 03/20/2002
// Copyright: (c) Chris M. Christoudias
// Version  : v0.1
////////////////////////////////////////////////////////

#include "stdafx.h"
#include "libppm.h"
#include <stdio.h>
#include <string.h>

int writePPMImage(char *filename, unsigned char *image, int height, int width, int depth, char *comments)
{

  if(!filename || !image) return PPM_NULL_PTR;
  FILE *fp;
  
  fopen_s(&fp, filename, "wb");
  if(!fp) return PPM_FILE_ERROR;

  //********************************************************
  //Write header information and comments.
  //********************************************************

  fprintf(fp, "P6\n", width, height);
  if(comments && strlen(comments) <= 70) fprintf(fp, "%s\n", comments);
  fprintf(fp, "%d %d\n%d\n", width, height, depth);
  
  //********************************************************
  //Output raw image data.
  //********************************************************

  int writeCount = (int)fwrite(image, sizeof(unsigned char), height*width*3, fp);
  fclose(fp);
  if(writeCount !=height*width*3) return PPM_FILE_ERROR;
  return PPM_NO_ERRORS;
}

int writePGMImage(char *filename, unsigned char *image, int height, int width, int depth, char *comments)
{

  if(!filename || !image) return PPM_NULL_PTR;
  FILE *fp;
  
  fopen_s(&fp, filename, "wb");
  if(!fp) return PPM_FILE_ERROR;

  //********************************************************
  //Write header information and comments.
  //********************************************************

  fprintf(fp, "P5\n", width, height);
  if(comments && strlen(comments) <= 70) fprintf(fp, "%s\n", comments);
  fprintf(fp, "%d %d\n%d\n", width, height, depth);
  
  //********************************************************
  //Output raw image data.
  //********************************************************

  int writeCount = (int)fwrite(image, sizeof(unsigned char), height*width, fp);
  fclose(fp);
  if(writeCount !=height*width) return PPM_FILE_ERROR;
  return PPM_NO_ERRORS;
}

//write a PNM image
int writePNMImage(char *filename , unsigned char *image, int height, int width, int depth, bool color, 
		  char *comments)
{
  int error;
  if(color) {
    error = writePPMImage(filename, image, height, width, depth, comments);
  } else {
    error = writePGMImage(filename, image, height, width, depth, comments);
  }
  return error;
}

int readPPMImage(char *filename, unsigned char **image, int& height, int& width, int& depth)
{

  if(!filename) return PPM_NULL_PTR;
  FILE *fp;
  
  fopen_s(&fp, filename, "rb");
  
  //********************************************************
  //Read header information.
  //********************************************************
  
  char buf[70];
  if(!fgets(buf, 70, fp)) return PPM_FILE_ERROR;
  if(strncmp(buf, "P6", 2)) return PPM_UNKNOWN_FORMAT;  
  do {
    if(!fgets(buf, 70, fp)) return PPM_FILE_ERROR;
  } while(buf[0] == '#');
  sscanf_s(buf, "%d %d", &width, &height);  
  if(!fgets(buf, 70, fp)) return PPM_FILE_ERROR;
  sscanf_s(buf , "%d", &depth);
  
  //********************************************************
  //Read raw data information.
  //********************************************************

  *image = new unsigned char [height * width * 3];
  int readCount = fread(*image, sizeof(unsigned char), height*width*3, fp);
  fclose(fp);
  if(readCount != height*width*3) return PPM_FILE_ERROR;

	//FILE *fp;
	//fopen_s(&fp,"d:\\segtest\\DSC03290.img","rb");
	//height = 768;
	//width = 1024;
	//*image = new unsigned char [height * width * 3];
	//int readCount = fread(*image, sizeof(unsigned char), height*width*3, fp);
	//fclose(fp);
	//if(readCount != height*width*3) return PPM_FILE_ERROR;	depth = 256;
  return PPM_NO_ERRORS;
}

int readPGMImage(char *filename, unsigned char **image, int& height, int& width, int& depth)
{

  if(!filename) return PPM_NULL_PTR;
  FILE *fp;
  
  fopen_s(&fp, filename, "rb");
  
  //********************************************************
  //Read header information.
  //********************************************************
  
  char buf[70];
  if(!fgets(buf, 70, fp)) return PPM_FILE_ERROR;
  if(strncmp(buf, "P5", 2)) return PPM_UNKNOWN_FORMAT;
  do {
    if(!fgets(buf, 70, fp)) return PPM_FILE_ERROR;
  } while(buf[0] == '#');
  sscanf_s(buf, "%d %d", &width, &height);  
  if(!fgets(buf, 70, fp)) return PPM_FILE_ERROR;
  sscanf_s(buf , "%d", &depth);
  
  //********************************************************
  //Read raw data information.
  //********************************************************

  *image = new unsigned char [height * width];
  int readCount = (int)fread(*image, sizeof(unsigned char), height*width, fp);
  fclose(fp);
  if(readCount != height*width) return PPM_FILE_ERROR;
  return PPM_NO_ERRORS;
}

//read a PNM image
int readPNMImage(char *filename, unsigned char **image, int& height, int& width, int& depth, bool& color)
{
  color = true; //assume PPM format
  int error = readPPMImage(filename, image, height, width, depth);
  if(error == PPM_UNKNOWN_FORMAT) {
    color = false;
    error = readPGMImage(filename, image, height, width, depth);
  }
  return error;
}


