#include "audio_packet_queue.h"
#include <cassert>
#include <iostream>
using namespace av;
Packet p(unsigned s,unsigned seq,unsigned long long t,double arrival){return {s,seq,t,arrival,{1}};}
int main(){
 uint8_t data[]={1,2,3};auto b=encode(7,9,1234567890123ULL,data,3);Packet x;assert(decode(b.data(),b.size(),10,x));assert(x.session==7&&x.seq==9&&x.stamp==1234567890123ULL&&x.data.size()==3);b[0]=0;assert(!decode(b.data(),b.size(),10,x));assert(!decode(b.data(),12,10,x));
 Queue q(100,40,3);assert(q.push(p(1,1,1000,10)));assert(!q.take(20,x));assert(q.take(50,x)&&x.seq==1);assert(!q.push(p(1,1,1000,51)));
 q.reset();q.push(p(1,2,1020,30));q.push(p(1,1,1000,31));assert(q.take(71,x)&&x.seq==1);assert(q.take(71,x)&&x.seq==2);
 q.reset();for(int i=1;i<=6;++i)q.push(p(1,i,1000+i*20,i*20));assert(q.size()==3);assert(q.take(120,x)&&x.seq==4);
 q.reset();q.push(p(1,1,1000,10));assert(!q.take(200,x));
 q.reset();q.push(p(1,1,1000,10));assert(!q.push(p(1,2,1020,500)));assert(!q.push(p(2,1,0,510)));assert(q.push(p(2,1,0,1100)));assert(q.take(1140,x)&&x.session==2);
 q.reset();q.push(p(1,0xffffffffu,1000,10));assert(q.take(50,x));q.push(p(1,0,1020,51));assert(q.take(51,x)&&x.seq==0);
 std::cout<<"packet / queue tests passed\n";
}
