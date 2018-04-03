#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

using namespace std;
int main(int argc, char** argv)
{
  if(argc < 2)
  {
    cout << "Missing input file argument." << endl;
  }

  cv::FileStorage ifs(argv[1], cv::FileStorage::READ);
  cv::Mat minput;
  ifs["zmap"] >> minput;

  cout << "Got matrix of size " << minput.size() << endl;

  std::string outfname = std::string(argv[1]);
  outfname.resize(outfname.size() - 5); // erase .yaml extension
  outfname += ".txt";
  
  ofstream zmap_outp;
  zmap_outp.open(outfname);
  for(int x = 0, x_end = minput.rows; x < x_end; x++)
  {
    for(int y = 0, y_end = minput.cols - 1; y < y_end; y++)
    {
      zmap_outp << minput.at<float>(x, y) << ",";
    }
    zmap_outp << minput.at<float>(x, minput.cols - 1) << endl;
  }
}