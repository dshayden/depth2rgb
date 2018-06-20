/*
 * Copyright (C) 2017- David S. Hayden - All Rights Reserved.
*/

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
