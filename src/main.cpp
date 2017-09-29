#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"


void GetParamsFromFile(std::string fname, std::string& serial, std::string& firmware,
                       libfreenect2::Freenect2Device::Config& cfg,
                       libfreenect2::Freenect2Device::IrCameraParams& ip,
                       libfreenect2::Freenect2Device::ColorCameraParams& cp)
{
  std::ifstream f;
  f.open(fname);

  std::string version;
  f >> version >> serial >> firmware;
  
  f >> cfg.MinDepth >> cfg.MaxDepth >> cfg.EnableBilateralFilter
    >> cfg.EnableEdgeAwareFilter;

  f >> ip.fx >> ip.fy >> ip.cx >> ip.cy >> ip.k1 >> ip.k2 >> ip.k3 >> ip.p1 >> ip.p2;
  f >> cp.fx >> cp.fy >> cp.cx >> cp.cy >> cp.shift_d >> cp.shift_m
    >> cp.mx_x3y0 >> cp.mx_x0y3 >> cp.mx_x2y1 >> cp.mx_x1y2 >> cp.mx_x2y0 >> cp.mx_x0y2
    >> cp.mx_x1y1 >> cp.mx_x1y0 >> cp.mx_x0y1 >> cp.mx_x0y0 >> cp.my_x3y0 >> cp.my_x0y3
    >> cp.my_x2y1 >> cp.my_x1y2 >> cp.my_x2y0 >> cp.my_x0y2 >> cp.my_x1y1
    >> cp.my_x1y0 >> cp.my_x0y1 >> cp.my_x0y0;
  f.close();
}

void WriteParamsToFile(std::string fname, std::string serial, std::string firmware,
                       libfreenect2::Freenect2Device::Config cfg,
                       libfreenect2::Freenect2Device::IrCameraParams ip,
                       libfreenect2::Freenect2Device::ColorCameraParams cp)
{
  std::ofstream paramFile;
  paramFile.open(fname);
  paramFile << "1.0" << std::endl;
  paramFile << serial << std::endl << firmware << std::endl;
  paramFile << cfg.MinDepth << " " << cfg.MaxDepth << " " <<
    cfg.EnableBilateralFilter << " " << cfg.EnableEdgeAwareFilter << std::endl;

  paramFile << ip.fx << " "
            << ip.fy << " "
            << ip.cx << " "
            << ip.cy << " "
            << ip.k1 << " "
            << ip.k2 << " "
            << ip.k3 << " "
            << ip.p1 << " "
            << ip.p2 << "\n";

  paramFile << cp.fx << " "
            << cp.fy << " "
            << cp.cx << " "
            << cp.cy << " "
            << cp.shift_d << " "
            << cp.shift_m << " "
            << cp.mx_x3y0 << " "
            << cp.mx_x0y3 << " "
            << cp.mx_x2y1 << " "
            << cp.mx_x1y2 << " "
            << cp.mx_x2y0 << " "
            << cp.mx_x0y2 << " "
            << cp.mx_x1y1 << " "
            << cp.mx_x1y0 << " "
            << cp.mx_x0y1 << " "
            << cp.mx_x0y0 << " "
            << cp.my_x3y0 << " "
            << cp.my_x0y3 << " "
            << cp.my_x2y1 << " "
            << cp.my_x1y2 << " "
            << cp.my_x2y0 << " "
            << cp.my_x0y2 << " "
            << cp.my_x1y1 << " "
            << cp.my_x1y0 << " "
            << cp.my_x0y1 << " "
            << cp.my_x0y0;

  paramFile.close();
}

int main(int argc, char *argv[]) 
{
  if (argc != 5) {
    std::cout << argv[0] << " inDep inRgb outDep depParams\n";
    std::cout <<
      "  inDep: (w=512, h=424) input depth image recorded with libfreenect2\n"
      "  inRgb: (w=1920, h=1080) input rgb image recorded with libfreenect2\n"
      "  outDep: (w=1920, h=1080) output depth->rgb aligned image\n"
      "  depParams: input text file with Kinect2 IR, RGB camera parameters\n";
    exit(1);
  }

  std::string inDep = argv[1], inRgb = argv[2], outDep = argv[3],
    depParamsFile = argv[4];

  std::string serialIn, firmwareIn;
  libfreenect2::Freenect2Device::Config cfgIn;
  libfreenect2::Freenect2Device::IrCameraParams ip;
  libfreenect2::Freenect2Device::ColorCameraParams cp;
  GetParamsFromFile(depParamsFile, serialIn, firmwareIn, cfgIn, ip, cp);

  libfreenect2::Registration* registration = new libfreenect2::Registration(ip, cp);

  cv::Mat origDep = cv::imread(inDep, CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat origDepF = cv::Mat(424, 512, CV_32FC1);
  origDep.convertTo(origDepF, CV_32FC1);

  cv::Mat origRgb = cv::imread(inRgb);
  cv::Mat origRgb4 = cv::Mat(1080, 1920, CV_8UC4);
  cv::cvtColor(origRgb, origRgb4, cv::COLOR_BGR2BGRA);

  libfreenect2::Frame depth(512, 424, 4, origDepF.data);
  libfreenect2::Frame rgb(1920, 1080, 4, origRgb4.data);
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),
    bigdepth(1920, 1082, 4);
  int* color_depth_map = new int[512*424];

  registration->apply(&rgb, &depth, &undistorted, &registered, true, &bigdepth, color_depth_map);

  cv::Mat bigdepMat(bigdepth.height, bigdepth.width, CV_32FC1, bigdepth.data);
  cv::Mat bigdepMatUInt16, bigdepMatUInt16clean;
  bigdepMat.convertTo(bigdepMatUInt16, CV_16UC1);

  cv::Mat notInfMask = cv::Mat(bigdepMatUInt16 <= cfgIn.MaxDepth*1000);
  bigdepMatUInt16.copyTo(bigdepMatUInt16clean, notInfMask);
  bigdepMatUInt16 = bigdepMatUInt16clean;

  cv::imwrite(outDep, bigdepMatUInt16);

  return 0;
}
