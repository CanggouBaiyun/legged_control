#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2 {
enum class SessionState {Preparing=1,Handoff=2,Running=3,Stopping=4,Stopped=5,Fault=6};
inline const char* sessionStateName(SessionState state) {
  switch(state) {
    case SessionState::Preparing:return "preparing";
    case SessionState::Handoff:return "handoff";
    case SessionState::Running:return "running";
    case SessionState::Stopping:return "stopping";
    case SessionState::Stopped:return "stopped";
    case SessionState::Fault:return "fault";
  }
  return "unknown";
}
struct SessionEvent {double time_s;SessionState state;std::string reason;};
// Owns lifecycle decisions only. Fault handling here does not prescribe a real
// robot stopping posture; the simulation adapter clears actuation and freezes.
class ControlSession {
 public:
  explicit ControlSession(bool preparation):state_(preparation?SessionState::Preparing:SessionState::Running) {
    events_.push_back({0.0,state_,"start"});
  }
  SessionState state() const {return state_;}
  bool stopping() const {return state_==SessionState::Stopping;}
  bool terminal() const {return state_==SessionState::Stopped || state_==SessionState::Fault;}
  const auto& events() const {return events_;}
  const std::string& reason() const {return events_.back().reason;}
  void startup(int stage,double time) {
    if(stopping() || terminal()) return;
    const auto next=stage==1?SessionState::Preparing:(stage==2?SessionState::Handoff:SessionState::Running);
    if(int(next)<int(state_)) throw std::runtime_error("Startup state moved backwards");
    transition(next,time,"startup");
  }
  bool requestStop(double time) {
    if(stopping() || terminal()) return false;
    stop_at_=time;quiet_since_=-1;
    transition(SessionState::Stopping,time,"stop_requested");return true;
  }
  bool observeStop(double time,bool stable_stance) {
    if(!stopping()) return false;
    if(!stable_stance) quiet_since_=-1;
    else if(quiet_since_<0) quiet_since_=time;
    if(quiet_since_>=0 && time-quiet_since_>=0.3-1e-9) {
      transition(SessionState::Stopped,time,"controlled_stop");return true;
    }
    if(time-stop_at_>=5.0-1e-9) throw std::runtime_error("Controlled stop did not settle within 5 s");
    return false;
  }
  void finish(double time,const std::string& reason) {
    if(terminal()) return;
    if(stopping() && reason=="duration_limit")
      throw std::runtime_error("Duration ended before controlled stop completed");
    transition(SessionState::Stopped,time,reason);
  }
  void fault(double time,const std::string& reason) {
    if(state_!=SessionState::Fault) transition(SessionState::Fault,time,reason);
  }
  double time_s=0.0;
  std::size_t samples=0;
  bool csv_flushed=false;
  std::string fault_source="initialization";
 private:
  void transition(SessionState next,double time,const std::string& reason) {
    if(!std::isfinite(time) || time<0) throw std::runtime_error("Invalid session event time");
    if(next==state_) return;
    state_=next;events_.push_back({time,next,reason});
  }
  SessionState state_;
  double stop_at_=-1,quiet_since_=-1;
  std::vector<SessionEvent> events_;
};
}  // namespace go2
