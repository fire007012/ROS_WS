#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace robot_navigation {
struct VideoGeometry { bool rotate; int width, height, left, right, top, bottom; };
inline VideoGeometry landscapeGeometry(int sw, int sh, int tw, int th, bool rotate_portrait=false) {
  if (sw<=0 || sh<=0 || tw<=th || th<=0) throw std::invalid_argument("landscape video dimensions required");
  const bool rotate=rotate_portrait && sh>sw;
  if (rotate) std::swap(sw,sh);
  const double scale=std::min(1.0,std::min(double(tw)/sw,double(th)/sh));
  const int w=std::max(1,int(std::lround(sw*scale))), h=std::max(1,int(std::lround(sh*scale)));
  const int dx=tw-w, dy=th-h;
  return {rotate,w,h,dx/2,dx-dx/2,dy/2,dy-dy/2};
}
}
