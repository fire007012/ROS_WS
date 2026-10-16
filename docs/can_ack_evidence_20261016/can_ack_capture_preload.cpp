#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <dlfcn.h>
#include <cstdarg>
#include <cerrno>
#include <cstdio>
#include <ctime>
// An impossible fd ensures an intercepted CAN call cannot reach a device.
constexpr int fake_can = 900000;
template<class T> T real(const char* name) { return reinterpret_cast<T>(dlsym(RTLD_NEXT, name)); }
extern "C" int socket(int domain, int type, int protocol) {
  if (domain == PF_CAN) return fake_can;
  return real<int(*)(int,int,int)>("socket")(domain,type,protocol);
}
extern "C" int ioctl(int fd, unsigned long request, ...) noexcept {
  va_list args; va_start(args,request); void* arg = va_arg(args,void*); va_end(args);
  if (fd != fake_can) return real<int(*)(int,unsigned long,...)>("ioctl")(fd,request,arg);
  auto* ifr = static_cast<ifreq*>(arg);
  if (request == SIOCGIFINDEX) { ifr->ifr_ifindex = 7; return 0; }
  if (request == SIOCGIFFLAGS) { ifr->ifr_flags = IFF_UP; return 0; }
  errno=EINVAL; return -1;
}
extern "C" int bind(int fd, const sockaddr* addr, socklen_t len) {
  if (fd == fake_can) return 0;
  return real<int(*)(int,const sockaddr*,socklen_t)>("bind")(fd,addr,len);
}
extern "C" int setsockopt(int fd, int level, int opt, const void* data, socklen_t len) {
  if (fd == fake_can) return 0;
  return real<int(*)(int,int,int,const void*,socklen_t)>("setsockopt")(fd,level,opt,data,len);
}
extern "C" ssize_t read(int fd, void* data, size_t len) {
  if (fd == fake_can) { errno=EAGAIN; return -1; }
  return real<ssize_t(*)(int,void*,size_t)>("read")(fd,data,len);
}
extern "C" ssize_t write(int fd, const void* data, size_t len) {
  if (fd != fake_can) return real<ssize_t(*)(int,const void*,size_t)>("write")(fd,data,len);
  if (len != sizeof(can_frame)) { errno=EINVAL; return -1; }
  timespec t{}; clock_gettime(CLOCK_MONOTONIC,&t);
  const auto* f=static_cast<const can_frame*>(data);
  std::fprintf(stderr,"WIRE %.9f %08x %u ",t.tv_sec+t.tv_nsec/1e9,f->can_id,f->can_dlc);
  for (unsigned i=0;i<8;++i) std::fprintf(stderr,"%02x",f->data[i]);
  std::fprintf(stderr,"\n");
  return len;
}
extern "C" int close(int fd) {
  if (fd == fake_can) return 0;
  return real<int(*)(int)>("close")(fd);
}
