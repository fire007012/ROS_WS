#include "robot_navigation/video_geometry.h"
#include <cassert>
#include <stdexcept>
using robot_navigation::landscapeGeometry;
int main() {
  auto p=landscapeGeometry(640,480,640,480);
  assert(!p.rotate && p.width==640 && p.height==480 && p.left==0 && p.top==0);
  p=landscapeGeometry(480,640,640,480);
  assert(!p.rotate && p.width==360 && p.height==480 && p.left==140 && p.right==140);
  p=landscapeGeometry(480,640,640,480,true);
  assert(p.rotate && p.width==640 && p.height==480);
  p=landscapeGeometry(640,480,640,360);
  assert(p.width==480 && p.height==360 && p.left==80 && p.right==80);
  p=landscapeGeometry(1280,720,640,480);
  assert(p.width==640 && p.height==360 && p.top==60 && p.bottom==60);
  bool rejected=false;
  try { landscapeGeometry(640,480,480,640); } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected);
}
