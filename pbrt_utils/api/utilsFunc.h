#ifndef UTILSFUNC
#define UTILSFUNC

#include<api/render.h>
#include <iostream>
#include<fstream>
#include<string>

void ChangeLookAt(RealisticCameraParam RC,std::ifstream &infile,std::ofstream &outfile);
void ChangeLookAt(PerspectiveCameraParam RC,std::ifstream &infile,std::ofstream &outfile);
void ChangeLookAt(OrthographicCameraParam RC,std::ifstream &infile,std::ofstream &outfile);
void ChangeLookAt(SphericalCameraParam RC,std::ifstream &infile,std::ofstream &outfile);

std::string generateFilenames(std::string filenames,std::string suffix);
std::string generatePbrtFile(RealisticCameraParam RC, std::string filenames);
std::string generatePbrtFile(PerspectiveCameraParam PC, std::string filenames);
std::string generatePbrtFile(OrthographicCameraParam OC, std::string filenames);
std::string generatePbrtFile(SphericalCameraParam SC, std::string filenames);

#endif