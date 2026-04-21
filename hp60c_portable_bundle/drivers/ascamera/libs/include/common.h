#pragma once

#include <vector>
#include <stdlib.h>

int saveYUVImg(const char *const fileName, void *dataImg, const size_t size);
int savePointCloud(const char *const fileName, float *pointCloud, size_t length);
int savePointCloudWithPcdFormat(const char *const fileName, float *pointCloud, size_t length);
int readData(const char *filepath, void *data, int size);
int save4img(const char *path, float  *tof, float *peak, float *noise, float *muliti, int size, float tmp);
int bgr2mono8(unsigned char *mono8_data, unsigned char *bgr_data, unsigned int img_width, unsigned int img_height);
