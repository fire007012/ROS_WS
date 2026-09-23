// Mono Opus/UDP with bounded receive queue and cancellable pacat pipes.
// Both peers MUST use protocol AV01. Legacy unframed Opus is intentionally rejected.
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <opus/opus.h>
#include "robot_navigation/audio_packet_queue.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <spawn.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>
#include <random>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
extern char** environ;
using Clock=std::chrono::steady_clock;
static double nowMs(){return std::chrono::duration<double,std::milli>(Clock::now().time_since_epoch()).count();}

// Separate stderr from PCM. No shell, no stdio buffering. Parent FD is nonblocking.
class Pacat {
 public:
  ~Pacat(){close();}
  bool open(bool capture,int latency,const std::string& device){
    close();int fds[2];if(pipe2(fds,O_CLOEXEC)<0)return false;
    std::vector<std::string> args={"pacat",capture?"--record":"--playback","--raw","--format=s16le","--rate=48000","--channels=1","--latency-msec="+std::to_string(latency),"--process-time-msec=10"};
    if(device!="default")args.push_back("--device="+device);
    std::vector<char*> argv;for(auto& a:args)argv.push_back(&a[0]);argv.push_back(nullptr);
    posix_spawn_file_actions_t fa;posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa,capture?fds[1]:fds[0],capture?STDOUT_FILENO:STDIN_FILENO);
    posix_spawn_file_actions_addclose(&fa,fds[0]);posix_spawn_file_actions_addclose(&fa,fds[1]);
    int rc=posix_spawnp(&pid_,"pacat",&fa,nullptr,argv.data(),environ);posix_spawn_file_actions_destroy(&fa);
    if(rc){::close(fds[0]);::close(fds[1]);pid_=-1;ROS_ERROR("pacat spawn: %s",strerror(rc));return false;}
    fd_=capture?fds[0]:fds[1];::close(capture?fds[1]:fds[0]);fcntl(fd_,F_SETFL,fcntl(fd_,F_GETFL)|O_NONBLOCK);return true;
  }
  // Deadline is measured from transfer start, not restarted for partial writes.
  bool transfer(void* data,size_t bytes,bool capture,std::atomic<bool>& run,int timeout_ms){
    size_t done=0;double deadline=nowMs()+timeout_ms;
    while(run&&done<bytes&&nowMs()<deadline){
      pollfd p{fd_,short(capture?POLLIN:POLLOUT),0};int n=poll(&p,1,10);
      if(n<0&&errno!=EINTR)return false;
      if(n<=0)continue;
      if(p.revents&(POLLERR|POLLNVAL))return false;
      if(!(p.revents&(capture?POLLIN:POLLOUT))){if(p.revents&POLLHUP)return false;continue;}
      auto* ptr=static_cast<uint8_t*>(data)+done;
      ssize_t k=capture?read(fd_,ptr,bytes-done):write(fd_,ptr,bytes-done);
      if(k>0)done+=size_t(k);else if(k==0||(errno!=EINTR&&errno!=EAGAIN))return false;
    }
    return done==bytes;
  }
  void close(){
    if(fd_>=0){::close(fd_);fd_=-1;}
    if(pid_>0){kill(pid_,SIGTERM);for(int i=0;i<20;++i){if(waitpid(pid_,nullptr,WNOHANG)==pid_){pid_=-1;return;}usleep(5000);}kill(pid_,SIGKILL);while(waitpid(pid_,nullptr,0)<0&&errno==EINTR){}pid_=-1;}
  }
 private:int fd_=-1;pid_t pid_=-1;
};

class AudioChatNode {
 public:
  AudioChatNode():nh_(),pnh_("~"){
    std::string role;pnh_.param<std::string>("role",role,"master");
    pnh_.param<std::string>("remote_ip",remote_,role=="slave"?"192.168.15.128":"192.168.15.129");
    pnh_.param("rx_port",rx_port_,5004);pnh_.param("tx_port",tx_port_,5004);
    pnh_.param<std::string>("mic_device",mic_,"default");pnh_.param<std::string>("speaker_device",speaker_,"default");
    pnh_.param("frame_duration_ms",frame_ms_,20);pnh_.param("sample_rate",rate_,48000);pnh_.param("channels",channels_,1);
    pnh_.param("opus_bitrate",bitrate_,32000);pnh_.param("opus_complexity",complexity_,5);
    pnh_.param("capture_latency_ms",capture_ms_,20);pnh_.param("playback_latency_ms",playback_ms_,40);
    pnh_.param("jitter_buffer_ms",jitter_ms_,40);pnh_.param("max_queue_ms",max_ms_,100);
    std::string topic;pnh_.param<std::string>("ready_topic",topic,"/Ready");
    sub_=nh_.subscribe<std_msgs::Int32>(topic,1,[this](const std_msgs::Int32::ConstPtr& m){ready_=m->data==1;});
    state_=nh_.advertise<std_msgs::Bool>("/audio_chat/active",1,true);
  }
  ~AudioChatNode(){stop();}
  bool init(){
    if(rate_!=48000||channels_!=1||(frame_ms_!=10&&frame_ms_!=20)||capture_ms_<10||playback_ms_<10||jitter_ms_<0||max_ms_<frame_ms_||jitter_ms_>=max_ms_||max_ms_>500||bitrate_<6000||bitrate_>128000||complexity_<0||complexity_>10||rx_port_<1||rx_port_>65535||tx_port_<1||tx_port_>65535){ROS_ERROR("Invalid audio parameters: require 48k mono, 10/20ms frames, 0<=jitter<max_queue<=500ms");return false;}
    queue_=av::Queue(max_ms_,jitter_ms_,size_t(std::max(1,max_ms_/frame_ms_)));
    signal(SIGPIPE,SIG_IGN);
    ROS_INFO("Audio AV01/Opus mono: capture=%d playback=%d jitter=%d max_queue=%d ms. Both peers must upgrade. Pulse requests are NOT measured device latency.",capture_ms_,playback_ms_,jitter_ms_,max_ms_);return true;
  }
  void spin(){ros::WallRate r(50);while(ros::ok()){ros::spinOnce();if(run_&&failed_)stop();if(ready_&&!run_&&!failed_)start();if(!ready_){if(run_)stop();failed_=false;}r.sleep();}stop();}
 private:
  void state(bool active){std_msgs::Bool b;b.data=active;state_.publish(b);}
  void fail(const char* why){ROS_ERROR("[audio_chat] %s; toggle /Ready 0 then 1 to retry",why);failed_=true;}
  void start(){
    rx_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);tx_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    sockaddr_in bind_addr{};bind_addr.sin_family=AF_INET;bind_addr.sin_port=htons(rx_port_);bind_addr.sin_addr.s_addr=INADDR_ANY;
    dest_={};dest_.sin_family=AF_INET;dest_.sin_port=htons(tx_port_);
    if(rx_<0||tx_<0||inet_pton(AF_INET,remote_.c_str(),&dest_.sin_addr)!=1||bind(rx_,reinterpret_cast<sockaddr*>(&bind_addr),sizeof(bind_addr))<0){fail("Socket setup failed");stop();return;}
    int capacity=16384;setsockopt(rx_,SOL_SOCKET,SO_RCVBUF,&capacity,sizeof(capacity));
    {std::lock_guard<std::mutex> lock(mutex_);queue_.reset();}
    run_=true;rx_thread_=std::thread(&AudioChatNode::receive,this);play_thread_=std::thread(&AudioChatNode::play,this);tx_thread_=std::thread(&AudioChatNode::transmit,this);state(true);
  }
  void stop(){
    run_=false;for(auto* t:{&rx_thread_,&play_thread_,&tx_thread_})if(t->joinable())t->join();
    if(rx_>=0){close(rx_);rx_=-1;}if(tx_>=0){close(tx_);tx_=-1;}state(false);
  }
  void receive(){
    uint8_t bytes[1400];uint64_t invalid=0,accepted=0;double report=nowMs();
    while(run_){pollfd fd{rx_,POLLIN,0};if(poll(&fd,1,10)<=0)continue;
      sockaddr_in from{};socklen_t len=sizeof(from);ssize_t n=recvfrom(rx_,bytes,sizeof(bytes),0,reinterpret_cast<sockaddr*>(&from),&len);
      if(n<=0)continue;
      av::Packet p;if(from.sin_addr.s_addr!=dest_.sin_addr.s_addr||!av::decode(bytes,size_t(n),nowMs(),p)||opus_packet_get_nb_samples(p.data.data(),p.data.size(),48000)!=48000*frame_ms_/1000){++invalid;ROS_WARN_THROTTLE(5.0,"[audio_chat] rejected packet: require matching AV01/Opus frame duration and remote_ip on both peers");continue;}
      {std::lock_guard<std::mutex> lock(mutex_);if(queue_.push(std::move(p)))++accepted;
       if(nowMs()-report>5000){ROS_INFO("[audio_chat] rx=%llu invalid=%llu dropped=%llu queued=%zu",(unsigned long long)accepted,(unsigned long long)invalid,(unsigned long long)queue_.dropped,queue_.size());report=nowMs();}}
    }
  }
  void play(){
    int err=0;OpusDecoder* dec=opus_decoder_create(48000,1,&err);if(!dec||err!=OPUS_OK){fail("Decoder init failed");return;}
    Pacat output;if(!output.open(false,playback_ms_,speaker_)){opus_decoder_destroy(dec);fail("Playback spawn failed");return;}
    int samples=48000*frame_ms_/1000;std::vector<int16_t> pcm(samples);auto tick=Clock::now();bool have=false;uint32_t session=0,seq=0;int missing=0;double report=nowMs(),max_write=0;uint64_t resets=0;
    while(run_){
      tick+=std::chrono::milliseconds(frame_ms_);av::Packet p;bool got;
      {std::lock_guard<std::mutex> lock(mutex_);got=queue_.take(nowMs(),p);}
      int decoded=samples;
      if(got){if(!have||session!=p.session||p.seq!=seq+1)opus_decoder_ctl(dec,OPUS_RESET_STATE);decoded=opus_decode(dec,p.data.data(),p.data.size(),pcm.data(),samples,0);session=p.session;seq=p.seq;have=true;missing=0;}
      else if(have&&++missing<=2){decoded=opus_decode(dec,nullptr,0,pcm.data(),samples,0);}
      else {std::fill(pcm.begin(),pcm.end(),0);have=false;}
      if(decoded<0){std::fill(pcm.begin(),pcm.end(),0);decoded=samples;}
      double begin=nowMs();bool ok=output.transfer(pcm.data(),size_t(decoded)*2,false,run_,max_ms_);max_write=std::max(max_write,nowMs()-begin);
      if(!ok&&run_){++resets;if(resets>3){fail("Playback repeatedly stalled");break;}output.close();{std::lock_guard<std::mutex> lock(mutex_);queue_.reset();}have=false;opus_decoder_ctl(dec,OPUS_RESET_STATE);if(!output.open(false,playback_ms_,speaker_)){fail("Playback restart failed");break;}tick=Clock::now();}
      if(nowMs()-report>5000){ROS_INFO("[audio_chat] playback max_write=%.1fms resets=%llu (not end-to-end latency)",max_write,(unsigned long long)resets);report=nowMs();max_write=0;}
      // Do not catch up by rapidly feeding a backlog into PulseAudio.
      if(Clock::now()>tick+std::chrono::milliseconds(frame_ms_))tick=Clock::now();
      std::this_thread::sleep_until(tick);
    }
    output.close();opus_decoder_destroy(dec);
  }
  void transmit(){
    int err=0;OpusEncoder* enc=opus_encoder_create(48000,1,OPUS_APPLICATION_VOIP,&err);if(!enc||err!=OPUS_OK){fail("Encoder init failed");return;}
    opus_encoder_ctl(enc,OPUS_SET_BITRATE(bitrate_));opus_encoder_ctl(enc,OPUS_SET_COMPLEXITY(complexity_));opus_encoder_ctl(enc,OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    Pacat input;if(!input.open(true,capture_ms_,mic_)){opus_encoder_destroy(enc);fail("Capture spawn failed");return;}
    int samples=48000*frame_ms_/1000;std::vector<int16_t> pcm(samples);uint8_t coded[1275];uint32_t session=std::random_device{}(),seq=0;double report=nowMs();uint64_t sent=0,errors=0;
    while(run_){
      if(!input.transfer(pcm.data(),pcm.size()*2,true,run_,1000)){if(run_)fail("Capture EOF/timeout");break;}
      int n=opus_encode(enc,pcm.data(),samples,coded,sizeof(coded));if(n<0){++errors;continue;}
      // Timestamp when PCM is read, not a hardware capture timestamp.
      auto packet=av::encode(session,seq++,uint64_t(nowMs()),coded,size_t(n));
      if(sendto(tx_,packet.data(),packet.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&dest_),sizeof(dest_))==ssize_t(packet.size()))++sent;else ++errors;
      if(nowMs()-report>5000){ROS_INFO("[audio_chat] tx=%llu errors=%llu",(unsigned long long)sent,(unsigned long long)errors);report=nowMs();}
    }
    input.close();opus_encoder_destroy(enc);
  }
  ros::NodeHandle nh_,pnh_;ros::Subscriber sub_;ros::Publisher state_;
  std::string remote_,mic_,speaker_;int rx_port_,tx_port_,frame_ms_,rate_,channels_,bitrate_,complexity_,capture_ms_,playback_ms_,jitter_ms_,max_ms_;
  std::atomic<bool> run_{false},failed_{false};bool ready_=false;int rx_=-1,tx_=-1;sockaddr_in dest_{};
  std::thread rx_thread_,play_thread_,tx_thread_;std::mutex mutex_;av::Queue queue_;
};
int main(int argc,char** argv){ros::init(argc,argv,"audio_chat_node");AudioChatNode node;if(!node.init())return 1;node.spin();return 0;}
