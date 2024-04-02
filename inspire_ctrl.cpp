#include "inspire.h"
#include <unitree/dds_wrapper/go2/go2_pub.h>
#include <unitree/dds_wrapper/go2/go2_sub.h>
#include "param.h"

class InspireRunner
{
public:
  InspireRunner()
  {
    serial = std::make_shared<SerialPort>(param::serial_port, B115200);

    // inspire
    righthand = std::make_shared<inspire::InspireHand>(serial, 1); // 按邵博格式，先右后左
    lefthand = std::make_shared<inspire::InspireHand>(serial, 2);

    // dds
    handcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/" + param::ns + "/cmd");
    handcmd->msg_.cmds().resize(12);
    handstate = std::make_unique<unitree::robot::go2::publisher::RealTimeMotorStates>(
        "rt/" + param::ns + "/state");
    handstate->msg_.states().resize(12);

    // Start running
    thread = std::make_shared<unitree::common::RecurrentThread>(
      10000, std::bind(&InspireRunner::run, this)
    );
  }

  void run()
  {
    // Set command
    if(!handcmd->isTimeout())
    {
      for(int i(0); i<12; i++)
      {
        qcmd(i) = handcmd->msg_.cmds()[i].q();
      }
      righthand->SetPosition(qcmd.block<6, 1>(0, 0)); // 先右后左
      lefthand->SetPosition(qcmd.block<6, 1>(6, 0));
    }

    // Recv state
    Eigen::Matrix<double, 6, 1> qtemp;
    if(righthand->GetPosition(qtemp) == 0)
    {
      qstate.block<6, 1>(0, 0) = qtemp;
    }
    else
    {
      for(int i(0); i<6; i++)
      {
        handstate->msg_.states()[i].lost()++;
      }
      spdlog::debug("Failed to get right hand state");
    }
    if(lefthand->GetPosition(qtemp) == 0)
    {
      qstate.block<6, 1>(6, 0) = qtemp;
    }
    else
    {
      for(int i(0); i<6; i++)
      {
        handstate->msg_.states()[i+6].lost()++;
      }
      spdlog::debug("Failed to get left hand state");
    }
    if(handstate->trylock())
    {
        for(int i(0); i<12; i++)
        {
            handstate->msg_.states()[i].q() = qstate(i);
        }
        handstate->unlockAndPublish();
    }
  }

  unitree::common::ThreadPtr thread;

  // inspire
  SerialPort::SharedPtr serial;
  std::shared_ptr<inspire::InspireHand> lefthand;
  std::shared_ptr<inspire::InspireHand> righthand;
  Eigen::Matrix<double, 12, 1> qcmd, qstate;

  // dds
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;
  std::unique_ptr<unitree::robot::go2::publisher::RealTimeMotorStates> handstate;
};

int main(int argc, char ** argv)
{
  auto vm = param::helper(argc, argv);
  if (param::network.empty())
  {
    // 在H1上需使用master service启动DDS，不带参数，查找程序同级目录下的parameter.json
    // 否则会报错
    unitree::robot::ChannelFactory::Instance()->Init();
  }
  else {
    unitree::robot::ChannelFactory::Instance()->Init(0, param::network);
  }

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller  " << std::endl;

  InspireRunner runner;
 
  while (true)
  {
    sleep(1);
  }
  return 0;
}