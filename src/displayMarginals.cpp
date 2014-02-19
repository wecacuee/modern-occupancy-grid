#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;

int main(int argc, const char *argv[]) {
  string fname("Data/Metropolis_Marginals.txt");
  if (argc == 2)
    fname = argv[1];
  ifstream fptr(fname.c_str());
  string line;
  getline(fptr, line);
  istringstream liness(line);
  size_t width, height;
  liness >> width;
  liness >> height;
  cv::Mat img(height, width, CV_64F);
  for (size_t i = 0; i < width * height; i++) {
    double val;
    fptr >> val;
    size_t x = i % width;
    size_t y = (size_t) floor(i / width);
    img.at<double>(y, x) = val;
  }

  img = 255 - img * 255;
  img.convertTo(img, CV_8U);
  cv::namedWindow("c", cv::WINDOW_NORMAL);
  cv::imwrite("out.png", img);
  cv::imshow("c", img);
  cv::waitKey(-1);
  return 0;
}
