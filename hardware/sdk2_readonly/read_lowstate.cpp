#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
using LowState=unitree_go::msg::dds_::LowState_;
// host time, tick, quaternion[4], gyro[3], accelerometer[3],
// motor[0..11] q/dq/tau_est, foot_force[4], foot_force_est[4].
using Sample=std::array<double,56>;
constexpr std::size_t max_samples=100000;
volatile std::sig_atomic_t interrupted=0;
void interrupt(int) {interrupted=1;}
double seconds(Clock::time_point begin,Clock::time_point end=Clock::now()) {
  return std::chrono::duration<double>(end-begin).count();
}
struct Options {
  std::string interface;
  std::filesystem::path csv="artifacts/go2_lowstate.csv";
  double duration=20;
  int domain=0;
};
Options options(int argc,char** argv) {
  Options result;
  for(int i=1;i<argc;++i) {
    const std::string key=argv[i];
    auto value=[&]() {
      if(++i>=argc) throw std::runtime_error("Missing value for "+key);
      return std::string(argv[i]);
    };
    if(key=="--interface") result.interface=value();
    else if(key=="--duration") result.duration=std::stod(value());
    else if(key=="--csv") result.csv=value();
    else if(key=="--domain") result.domain=std::stoi(value());
    else if(key=="--help") {
      std::cout<<"Read-only Go2 SDK2 capture; subscribes to rt/lowstate.\n"
          "--interface NAME [--duration 20] [--csv artifacts/go2_lowstate.csv] [--domain 0]\n"
          "Duration is measured from the first received sample, in (0,60] seconds.\n"
          "Waits at most 10 s for the first sample. Ctrl+C saves received data.\n"
          "Stops and saves partial data if the stream is silent for 1 s.\n"
          "CSV preserves raw SDK array indices; tick units and force calibration are not assumed.\n";
      std::exit(0);
    } else throw std::runtime_error("Unknown option: "+key);
  }
  if(result.interface.empty()) throw std::runtime_error("--interface is required");
  if(!std::isfinite(result.duration) || result.duration<=0 || result.duration>60)
    throw std::runtime_error("--duration must be in (0,60] seconds");
  if(result.domain<0 || result.domain>232) throw std::runtime_error("--domain must be in [0,232]");
  if(result.csv.empty() || result.csv.extension()!=".csv") throw std::runtime_error("--csv must end in .csv");
  if(std::filesystem::exists(result.csv)) throw std::runtime_error("CSV already exists; choose a new filename");
  return result;
}

struct Recorder {
  explicit Recorder(double duration_s):duration(duration_s) {samples.reserve(max_samples);}
  std::mutex mutex;
  std::vector<Sample> samples;
  Clock::time_point first,last;
  bool started=false,accepting=true,overflow=false;
  const double duration;

  void receive(const void* message) {
    // SDK owns message memory; copy values before this callback returns.
    const auto& state=*static_cast<const LowState*>(message);
    std::lock_guard<std::mutex> lock(mutex);
    if(!accepting) return;
    const auto now=Clock::now();
    if(!started) {first=now;started=true;}
    const double stamp=seconds(first,now);
    if(stamp>=duration) return;
    if(samples.size()>=max_samples) {overflow=true;accepting=false;return;}
    Sample row{};
    last=now;
    std::size_t index=0;
    row[index++]=stamp;row[index++]=state.tick();
    for(const auto value:state.imu_state().quaternion()) row[index++]=value;
    for(const auto value:state.imu_state().gyroscope()) row[index++]=value;
    for(const auto value:state.imu_state().accelerometer()) row[index++]=value;
    for(int i=0;i<12;++i) {
      const auto& motor=state.motor_state()[i];
      row[index++]=motor.q();row[index++]=motor.dq();row[index++]=motor.tau_est();
    }
    for(const auto value:state.foot_force()) row[index++]=value;
    for(const auto value:state.foot_force_est()) row[index++]=value;
    samples.push_back(row);
  }
};

void save(const std::filesystem::path& path,const std::vector<Sample>& samples) {
  if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out<<std::setprecision(17)<<"host_time_s,tick";
  for(int i=0;i<4;++i) out<<",imu_quat_"<<i;
  for(int i=0;i<3;++i) out<<",imu_gyro_"<<i;
  for(int i=0;i<3;++i) out<<",imu_accel_"<<i;
  for(int i=0;i<12;++i) out<<",motor_"<<i<<"_q,motor_"<<i<<"_dq,motor_"<<i<<"_tau_est";
  for(int i=0;i<4;++i) out<<",foot_force_"<<i;
  for(int i=0;i<4;++i) out<<",foot_force_est_"<<i;
  out<<'\n';
  for(const auto& row:samples) {
    for(std::size_t i=0;i<row.size();++i) {if(i) out<<',';out<<row[i];}
    out<<'\n';
  }
  out.close();
  if(!out) throw std::runtime_error("Failed writing CSV: "+path.string());
}

int run(const Options& opts) {
  auto recorder=std::make_shared<Recorder>(opts.duration);
  std::signal(SIGINT,interrupt);std::signal(SIGTERM,interrupt);
  unitree::robot::ChannelFactory::Instance()->Init(opts.domain,opts.interface);
  unitree::robot::ChannelSubscriber<LowState> subscriber("rt/lowstate");
  subscriber.InitChannel([recorder](const void* message) {recorder->receive(message);},0);
  std::cout<<"Listening on "<<opts.interface<<", DDS domain="<<opts.domain
      <<"; waiting for rt/lowstate..."<<std::endl;
  const auto wait_start=Clock::now();
  bool no_samples=false,stream_stalled=false;
  while(!interrupted) {
    {
      std::lock_guard<std::mutex> lock(recorder->mutex);
      if(recorder->overflow) break;
      if(recorder->started && seconds(recorder->first)>=opts.duration) break;
      if(recorder->started && seconds(recorder->last)>=1.0) {stream_stalled=true;break;}
      if(!recorder->started && seconds(wait_start)>=10.0) {no_samples=true;break;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  {
    std::lock_guard<std::mutex> lock(recorder->mutex);
    recorder->accepting=false;
  }
  subscriber.CloseChannel();
  std::vector<Sample> samples;
  bool overflow;
  {
    std::lock_guard<std::mutex> lock(recorder->mutex);
    samples=std::move(recorder->samples);overflow=recorder->overflow;
  }
  if(samples.empty()) throw std::runtime_error(no_samples?
      "No LowState received in 10 s; check the robot connection, interface and DDS domain":
      "Capture stopped before any LowState arrived");
  save(opts.csv,samples);
  std::size_t invalid=0;
  for(const auto& row:samples) if(std::any_of(row.begin(),row.end(),
      [](double value) {return !std::isfinite(value);})) ++invalid;
  std::cout<<"Saved "<<samples.size()<<" samples to "<<opts.csv
      <<"; host sample span="<<samples.back()[0]<<" s; non-finite rows="<<invalid<<'\n';
  if(samples.size()>1 && samples.back()[0]>0) {
    std::vector<double> intervals;intervals.reserve(samples.size()-1);
    for(std::size_t i=1;i<samples.size();++i) intervals.push_back(1000*(samples[i][0]-samples[i-1][0]));
    std::sort(intervals.begin(),intervals.end());
    std::cout<<"Arrival frequency="<<(samples.size()-1)/samples.back()[0]<<" Hz; interval p99/max="
        <<intervals[static_cast<std::size_t>(std::ceil(.99*intervals.size()))-1]
        <<'/'<<intervals.back()<<" ms\n";
  }
  if(overflow) throw std::runtime_error("Capture buffer full; saved partial data");
  if(stream_stalled) throw std::runtime_error("LowState stream silent for 1 s; saved partial data");
  if(invalid) throw std::runtime_error("Capture contains non-finite values; raw data retained");
  return 0;
}
}
int main(int argc,char** argv) {
  try {return run(options(argc,argv));}
  catch(const std::exception& error) {std::cerr<<"go2_read_lowstate: "<<error.what()<<'\n';return 1;}
}
