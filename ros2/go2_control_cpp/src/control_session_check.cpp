#include "go2_control_cpp/control_session.hpp"
#include <iostream>
namespace {
void require(bool value) {if(!value) throw std::runtime_error("Control session check failed");}
}
int main() {
  try {
    go2::ControlSession normal(true);
    normal.startup(1,0);normal.startup(2,2.2);normal.startup(3,2.4);
    require(normal.state()==go2::SessionState::Running);
    require(normal.requestStop(4));require(!normal.requestStop(4.1));
    normal.startup(3,4.2);require(normal.stopping());
    require(!normal.observeStop(4.2,true));require(!normal.observeStop(4.4,false));
    require(!normal.observeStop(4.5,true));require(!normal.observeStop(4.798,true));
    require(normal.observeStop(4.8,true));require(normal.reason()=="controlled_stop");
    require(!normal.requestStop(5));normal.startup(1,5);require(normal.terminal());
    go2::ControlSession fault(false);
    fault.fault(3,"first fault");fault.fault(4,"second fault");
    require(fault.reason()=="first fault" && !fault.requestStop(4));
    fault.startup(3,5);require(fault.state()==go2::SessionState::Fault);
    go2::ControlSession timeout(false);timeout.requestStop(1);
    bool rejected=false;
    try {timeout.observeStop(6,false);} catch(const std::runtime_error&) {rejected=true;}
    require(rejected);
    rejected=false;
    try {timeout.finish(6,"duration_limit");} catch(const std::runtime_error&) {rejected=true;}
    require(rejected);
    go2::ControlSession timed(false);timed.finish(20,"duration_limit");
    require(timed.reason()=="duration_limit" && timed.terminal());
    std::cout<<"Control session startup, continuous settling, timeout and fault-latch checks passed\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
