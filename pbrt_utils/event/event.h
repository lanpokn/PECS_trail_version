#ifndef EVENT
#define EVENT


// #include<api/pbrt_render.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/videoio.hpp>  // Video write
#include<stdio.h>
#include<vector>
#include<fstream>
#include<string>

#include<api/pbrt_render.h>
void test_cv();
void GenerateIntensityVideo(pbrt_render render, int row, int column,std::string name);

#endif