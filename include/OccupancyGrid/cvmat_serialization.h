#ifndef CVMAT_SERIALIZATION_H
#define CVMAT_SERIALIZATION_H
#include <string>
#include <opencv2/opencv.hpp>
void saveMat(cv::Mat& m, std::string filename);
void loadMat(cv::Mat& m, std::string filename);
#endif // CVMAT_SERIALIZATION_H
