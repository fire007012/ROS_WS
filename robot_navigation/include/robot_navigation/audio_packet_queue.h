#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <vector>

namespace av {
// Wire protocol v1: AV01, session u32, sequence u32, monotonic sample time ms u64.
inline void put32(uint8_t* p, uint32_t n) { for (int i=3;i>=0;--i) {p[i]=uint8_t(n); n>>=8;} }
inline uint32_t get32(const uint8_t* p) {return uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
inline void put64(uint8_t* p, uint64_t n) {put32(p,uint32_t(n>>32));put32(p+4,uint32_t(n));}
inline uint64_t get64(const uint8_t* p) {return uint64_t(get32(p))<<32|get32(p+4);}
struct Packet {uint32_t session, seq; uint64_t stamp; double arrival; std::vector<uint8_t> data;};
inline std::vector<uint8_t> encode(uint32_t session,uint32_t seq,uint64_t stamp,const uint8_t* data,size_t n) {
  std::vector<uint8_t> b(20+n);b[0]='A';b[1]='V';b[2]='0';b[3]='1';put32(b.data()+4,session);put32(b.data()+8,seq);put64(b.data()+12,stamp);std::copy(data,data+n,b.begin()+20);return b;
}
inline bool decode(const uint8_t* b,size_t n,double arrival,Packet& p) {
  if(n<=20||n>1295||b[0]!='A'||b[1]!='V'||b[2]!='0'||b[3]!='1')return false;
  p={get32(b+4),get32(b+8),get64(b+12),arrival,std::vector<uint8_t>(b+20,b+n)};return true;
}
inline bool newer(uint32_t a,uint32_t b) {return int32_t(a-b)>0;}
// Caller holds mutex. Receiver clock only: no synchronized VM clocks required.
// Minimum observed arrival-sender offset estimates relative extra delay, not one-way latency.
class Queue {
 public:
  Queue(double max_age=100,double prebuffer=40,size_t cap=5):max_age_(max_age),prebuffer_(prebuffer),cap_(cap){}
  void reset(){q_.clear();active_=false;played_=false;primed_=false;}
  bool push(Packet p) {
    if(!active_||p.session!=session_) {
      if(active_&&p.arrival-last_rx_<500) {++dropped;return false;}
      reset();active_=true;session_=p.session;offset_=p.arrival-double(p.stamp);first_=p.arrival;
    }
    if(played_&&!newer(p.seq,last_seq_)){++dropped;return false;}
    for(const auto& x:q_)if(x.seq==p.seq){++dropped;return false;}
    offset_=std::min(offset_,p.arrival-double(p.stamp));last_rx_=p.arrival;
    if(p.arrival-(double(p.stamp)+offset_)>max_age_){++dropped;return false;}
    auto i=std::find_if(q_.begin(),q_.end(),[&](const Packet& x){return newer(x.seq,p.seq);});q_.insert(i,std::move(p));
    while(q_.size()>cap_){q_.pop_front();++dropped;}
    return true;
  }
  bool take(double now,Packet& p) {
    while(!q_.empty()&&(now-q_.front().arrival>max_age_||now-(double(q_.front().stamp)+offset_)>max_age_)) {q_.pop_front();++dropped;}
    if(!primed_&&now-first_<prebuffer_)return false;
    if(q_.empty()){primed_=false;first_=now;return false;}
    primed_=true;p=std::move(q_.front());q_.pop_front();last_seq_=p.seq;played_=true;return true;
  }
  size_t size()const{return q_.size();}
  uint64_t dropped=0;
 private:
  std::deque<Packet> q_;double max_age_,prebuffer_,offset_=0,first_=0,last_rx_=0;size_t cap_;
  bool active_=false,played_=false,primed_=false;uint32_t session_=0,last_seq_=0;
};
}
