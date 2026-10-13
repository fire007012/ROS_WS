#pragma once
#include <opencv2/imgproc.hpp>
#include <zbar.h>
#include <string>
#include <vector>
namespace robot_navigation {
struct DecodedCode { std::string data; zbar::zbar_symbol_type_t type; };
inline bool isLinearBarcode(zbar::zbar_symbol_type_t type) {
  return type == zbar::ZBAR_CODE128 || type == zbar::ZBAR_CODE39 ||
         type == zbar::ZBAR_EAN13 || type == zbar::ZBAR_EAN8 ||
         type == zbar::ZBAR_UPCA || type == zbar::ZBAR_UPCE || type == zbar::ZBAR_I25;
}
// Production decoder and ROS image-regression test use this exact function.
// Native capture remains 640x480. At most one extra preprocessed attempt per
// frame; scaling cannot recover bars which the sensor has already undersampled.
inline std::vector<DecodedCode> decodeCodes(zbar::ImageScanner& scanner,
                                          const cv::Mat& input, bool need_barcode,
                                          unsigned attempt) {
  std::vector<DecodedCode> results;
  auto scan = [&](const cv::Mat& source) {
    cv::Mat gray = source.isContinuous() ? source : source.clone();
    zbar::Image image(gray.cols, gray.rows, "Y800", gray.data, gray.total());
    scanner.scan(image);
    bool linear = false;
    for (auto it=image.symbol_begin(); it!=image.symbol_end(); ++it) {
      results.push_back({it->get_data(),it->get_type()});
      linear = linear || isLinearBarcode(it->get_type());
    }
    image.set_data(nullptr,0); // OpenCV owns the pixels.
    return linear;
  };
  if (scan(input) || !need_barcode) return results;
  cv::Mat processed;
  switch (attempt % 4) {
    case 0: {
      // Always scan the full frame first. A broad central ROI is only an
      // additional attempt, so codes near an edge/long codes retain a chance.
      cv::Mat roi = input;
      if (input.cols >= 1000 && input.rows >= 600) {
        const int x = input.cols / 20, y = input.rows / 8;
        roi = input(cv::Rect(x,y,input.cols-2*x,input.rows-2*y));
      }
      cv::resize(roi,processed,cv::Size(),2,2,cv::INTER_CUBIC); break;
    }
    case 1: cv::rotate(input,processed,cv::ROTATE_90_CLOCKWISE); break;
    case 2: cv::normalize(input,processed,0,255,cv::NORM_MINMAX);
            cv::resize(processed,processed,cv::Size(),2,2,cv::INTER_CUBIC); break;
    default: cv::threshold(input,processed,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU); break;
  }
  scan(processed);
  return results;
}
}
