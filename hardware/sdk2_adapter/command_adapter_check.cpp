#include "go2_sdk2/command_adapter.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>

namespace {
using Tree=boost::property_tree::ptree;
using go2::sdk2::LowCmd;
const std::array<std::string,12> sdk_names{{
    "FR_hip_joint","FR_thigh_joint","FR_calf_joint","FL_hip_joint","FL_thigh_joint","FL_calf_joint",
    "RR_hip_joint","RR_thigh_joint","RR_calf_joint","RL_hip_joint","RL_thigh_joint","RL_calf_joint"}};
const std::array<std::string,5> fields{{"q_rad","dq_radps","kp","kd","tau_ff_nm"}};
void require(bool value,const std::string& what) {if(!value) throw std::runtime_error(what);}
template<class Function> void rejects(Function function) {
  bool caught=false;
  try {function();} catch(const std::exception&) {caught=true;}
  require(caught,"Expected invalid adapter input to be rejected");
}
// Reference loop from SDK2 go2_low_level.cpp, d1abf001 (BSD-3-Clause;
// see UNITREE_LICENSE). memcpy replaces the example's pointer aliasing.
std::uint32_t referenceCrc(const LowCmd& packet) {
  std::array<std::uint32_t,sizeof(LowCmd)/4> words;
  std::memcpy(words.data(),&packet,sizeof(packet));
  std::uint32_t crc=0xffffffffu;
  for(std::size_t i=0;i+1<words.size();++i) {
    std::uint32_t mask=0x80000000u,data=words[i];
    for(int bit=0;bit<32;++bit) {
      if(crc&0x80000000u) crc=(crc<<1)^0x04c11db7u;
      else crc<<=1;
      if(data&mask) crc^=0x04c11db7u;
      mask>>=1;
    }
  }
  return crc;
}
std::array<double,5> sourceValues(const go2::JointCommand& command,int i) {
  return {command.position[i],command.velocity[i],command.kp[i],command.kd[i],command.feedforward_torque[i]};
}
std::array<double,5> packetValues(const unitree_go::msg::dds_::MotorCmd_& motor) {
  return {motor.q(),motor.dq(),motor.kp(),motor.kd(),motor.tau()};
}
void verify(const go2::JointCommand& command,const std::vector<std::string>& names,
            const LowCmd& packet,std::array<double,5>& errors) {
  require(packet.head()==std::array<std::uint8_t,2>{0xfe,0xef} && packet.level_flag()==0xff,"Packet header");
  require(packet.crc()==referenceCrc(packet) && go2::sdk2::validCommandCrc(packet),"Packet CRC");
  for(int i=0;i<12;++i) {
    const auto index=std::find(sdk_names.begin(),sdk_names.end(),names[i])-sdk_names.begin();
    require(index<12,"Test joint name");
    const auto& motor=packet.motor_cmd()[index];
    require(motor.mode()==1,"Active motor mode");
    const auto source=sourceValues(command,i),actual=packetValues(motor);
    for(int f=0;f<5;++f) {
      require(actual[f]==static_cast<float>(source[f]),"Mapped field "+names[i]+"/"+fields[f]);
      errors[f]=std::max(errors[f],std::abs(source[f]-actual[f]));
    }
  }
  for(int i=12;i<20;++i) {
    const auto& motor=packet.motor_cmd()[i];
    require(motor.mode()==1 && motor.q()==2.146e9f && motor.dq()==16000.0f &&
            motor.kp()==0 && motor.kd()==0 && motor.tau()==0,"Unused SDK motor slot");
  }
  for(const auto& motor:packet.motor_cmd())
    require(motor.reserve()==std::array<std::uint32_t,3>{},"Motor reserve fields");
  require(packet.gpio()==0 && packet.bms_cmd().off()==0 && packet.reserve()==0 &&
          packet.sn()==std::array<std::uint32_t,2>{} && packet.version()==std::array<std::uint32_t,2>{},
          "Unexpected auxiliary command fields");
}
void syntheticChecks(const std::vector<std::string>& model_names) {
  auto names=model_names;
  std::reverse(names.begin(),names.end()); // Deliberately break positional equivalence.
  const go2::sdk2::CommandAdapter adapter(names);
  go2::JointCommand command;
  for(int i=0;i<12;++i) {
    command.position[i]=0.01+i*0.11;command.velocity[i]=-0.02-i*0.03;
    command.kp[i]=20+i;command.kd[i]=1+i*0.1;command.feedforward_torque[i]=-3+i*0.4;
  }
  LowCmd packet{},second{};
  std::array<double,5> errors{};
  packet.gpio()=255;packet.bms_cmd().off()=255;packet.led().fill(255);
  adapter.encode(command,packet);verify(command,names,packet,errors);
  adapter.encode(command,second);
  require(std::memcmp(&packet,&second,sizeof(packet))==0,"Packet construction is not deterministic");
  packet.motor_cmd()[0].tau()+=1;
  require(!go2::sdk2::validCommandCrc(packet),"CRC did not detect a changed command");
  go2::sdk2::LowState state{};
  for(int i=0;i<12;++i) {
    state.motor_state()[i].q()=0.25f*i;
    state.motor_state()[i].dq()=-0.5f*i;
    state.motor_state()[i].tau_est()=0.75f*i;
  }
  const auto decoded=adapter.decodeJoints(state);
  for(int i=0;i<12;++i) {
    const auto index=std::find(sdk_names.begin(),sdk_names.end(),names[i])-sdk_names.begin();
    require(decoded.position[i]==0.25*index && decoded.velocity[i]==-0.5*index &&
            decoded.estimated_torque[i]==0.75*index,"LowState named joint mapping");
  }
  auto invalid=names;invalid[0]=invalid[1];rejects([&]{go2::sdk2::CommandAdapter check(invalid);});
  invalid=names;invalid[0]="unknown_joint";rejects([&]{go2::sdk2::CommandAdapter check(invalid);});
  invalid=names;invalid.pop_back();rejects([&]{go2::sdk2::CommandAdapter check(invalid);});
  auto bad=command;bad.kp[0]=-1;rejects([&]{adapter.encode(bad,packet);});
  bad=command;bad.position[0]=std::numeric_limits<double>::quiet_NaN();rejects([&]{adapter.encode(bad,packet);});
  bad=command;bad.feedforward_torque[0]=std::numeric_limits<double>::max();rejects([&]{adapter.encode(bad,packet);});
  state.motor_state()[0].dq()=std::numeric_limits<float>::infinity();rejects([&]{adapter.decodeJoints(state);});
  std::cout<<"Named/permuted mapping, all five fields, unused slots, CRC mutation, deterministic bytes and invalid inputs: PASS\n";
}
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> result;
  std::size_t from=0;
  for(;;) {
    const auto to=line.find(',',from);
    result.push_back(line.substr(from,to==std::string::npos?to:to-from));
    if(to==std::string::npos) break;
    from=to+1;
  }
  return result;
}
double number(const std::string& value) {
  std::size_t used=0;const double result=std::stod(value,&used);
  require(used==value.size() && std::isfinite(result),"Invalid numeric CSV command field");return result;
}
std::size_t replay(const std::filesystem::path& path,const std::vector<std::string>& names,
                   const go2::sdk2::CommandAdapter& adapter,std::array<double,5>& errors) {
  std::ifstream stream(path);require(bool(stream),"Cannot open command CSV");
  std::string line;require(bool(std::getline(stream,line)),"Empty command CSV");
  if(!line.empty() && line.back()=='\r') line.pop_back();
  const auto header=split(line);
  require(std::set<std::string>(header.begin(),header.end()).size()==header.size(),"Duplicate CSV columns");
  auto column=[&](const std::string& name) {
    const auto found=std::find(header.begin(),header.end(),name);
    require(found!=header.end(),"Missing CSV column "+name);return found-header.begin();
  };
  const auto time_column=column("time_s");
  std::array<std::array<std::size_t,5>,12> columns;
  for(int i=0;i<12;++i) for(int f=0;f<5;++f) columns[i][f]=column("cmd_"+names[i]+"_"+fields[f]);
  LowCmd packet{};
  std::size_t rows=0;
  while(std::getline(stream,line)) {
    if(!line.empty() && line.back()=='\r') line.pop_back();
    const auto row=split(line);require(row.size()==header.size(),"CSV column count changed");
    go2::JointCommand command;command.time_s=number(row[time_column]);
    require(std::abs(command.time_s-rows*0.002)<1e-8,"Nonconsecutive command CSV time");
    for(int i=0;i<12;++i) {
      command.position[i]=number(row[columns[i][0]]);command.velocity[i]=number(row[columns[i][1]]);
      command.kp[i]=number(row[columns[i][2]]);command.kd[i]=number(row[columns[i][3]]);
      command.feedforward_torque[i]=number(row[columns[i][4]]);
    }
    adapter.encode(command,packet);verify(command,names,packet,errors);++rows;
  }
  require(stream.eof() && rows>0,"CSV read failed or contained no commands");
  return rows;
}
}
int main(int argc,char** argv) {
  try {
    std::filesystem::path root,csv,report;
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help") {
        std::cout<<"Offline only: --project-root ROOT [--csv native.csv] [--report result.json]\n";return 0;
      }
      require(i+1<argc,"Missing CLI value");const std::string value=argv[++i];
      if(key=="--project-root") root=value;
      else if(key=="--csv") csv=value;
      else if(key=="--report") report=value;
      else throw std::runtime_error("Unknown option: "+key);
    }
    require(!root.empty(),"--project-root is required");
    if(!csv.empty() && csv.is_relative()) csv=root/csv;
    if(!report.empty() && report.is_relative()) report=root/report;
    require(report.empty() || !std::filesystem::exists(report),"Report already exists; choose a new path");
    Tree config;boost::property_tree::read_json((root/"config/go2_model.json").string(),config);
    std::vector<std::string> names;
    for(const auto& entry:config.get_child("nominal_joint_positions_rad")) names.push_back(entry.first);
    const go2::sdk2::CommandAdapter adapter(names);
    syntheticChecks(names);
    Tree result,mapping,error_tree;
    for(std::size_t i=0;i<names.size();++i) {
      mapping.put(names[i],adapter.indices()[i]);
      std::cout<<names[i]<<" -> SDK motor["<<adapter.indices()[i]<<"]\n";
    }
    std::array<double,5> errors{};
    const auto rows=csv.empty()?0:replay(csv,names,adapter,errors);
    result.put("schema",1);result.put("passed",true);result.put("reference_sdk_commit","d1abf001220d3e5f59e00a0c4aaa595e00089a4a");
    result.put("dds_initialized",false);result.put("commands_sent",0);result.put("hardware_verified",false);
    result.put("lowcmd_size_bytes",sizeof(LowCmd));result.put("replayed_samples",rows);result.put("csv",csv.string());
    result.add_child("joint_mapping",mapping);
    for(int f=0;f<5;++f) error_tree.put(fields[f],errors[f]);
    result.add_child("max_float_conversion_error",error_tree);
    if(!report.empty()) {
      if(report.has_parent_path()) std::filesystem::create_directories(report.parent_path());
      boost::property_tree::write_json(report.string(),result);
    }
    std::cout<<"Offline SDK2 checks passed; replayed "<<rows<<" commands; DDS initialized=0; commands sent=0\n";
    for(int f=0;f<5;++f) std::cout<<fields[f]<<" max float conversion error="<<std::setprecision(10)<<errors[f]<<'\n';
  } catch(const std::exception& error) {std::cerr<<"SDK2 adapter: "<<error.what()<<'\n';return 1;}
}
