#include <atomic>
#include <cmath>
#include <functional>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace tomato_sim_gazebo
{
class SlideParkingBrake final : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    joint_ = model->GetJoint("machine_slide");
    if (!joint_ || model->GetWorld()->Physics()->GetType() != "ode")
    {
      gzerr << "Slide parking brake requires machine_slide and ODE physics\n";
      return;
    }
    lower_ = joint_->LowerLimit(0);
    upper_ = joint_->UpperLimit(0);
    stop_erp_ = joint_->GetParam("stop_erp", 0);
    stop_cfm_ = joint_->GetParam("stop_cfm", 0);
    node_ = gazebo_ros::Node::Get(sdf);
    subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/slide_velocity_controller/commands", rclcpp::QoS(10),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message)
      {
        if (message->data.size() == 1)
        {
          const double speed = message->data.front();
          moving_.store(std::isfinite(speed) && std::abs(speed) > 1e-9);
        }
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SlideParkingBrake::Update, this));
  }

  void Reset() override
  {
    moving_.store(false);
    if (joint_)
    {
      Release();
    }
  }

private:
  void Release()
  {
    joint_->SetLowerLimit(0, lower_);
    joint_->SetUpperLimit(0, upper_);
    joint_->SetParam("stop_erp", 0, stop_erp_);
    joint_->SetParam("stop_cfm", 0, stop_cfm_);
    locked_ = false;
  }

  void Update()
  {
    // All physics changes run on Gazebo's update thread. The ROS callback
    // communicates only the requested moving/parked state.
    if (moving_.load())
    {
      if (locked_)
      {
        Release();
      }
      return;
    }
    if (!locked_)
    {
      // Equal ODE stops constrain the slide at its actual stopping position.
      // Capture it once so arm reaction forces cannot move the hold target.
      const double position = joint_->Position(0);
      joint_->SetParam("stop_erp", 0, 0.8);
      joint_->SetParam("stop_cfm", 0, 1e-10);
      joint_->SetUpperLimit(0, position);
      joint_->SetLowerLimit(0, position);
      joint_->SetUpperLimit(0, position);
      joint_->SetVelocity(0, 0.0);
      locked_ = true;
    }
  }

  gazebo::physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  gazebo::event::ConnectionPtr update_;
  std::atomic<bool> moving_{false};
  bool locked_{false};
  double lower_{0.0};
  double upper_{0.0};
  double stop_erp_{0.0};
  double stop_cfm_{0.0};
};
}

GZ_REGISTER_MODEL_PLUGIN(tomato_sim_gazebo::SlideParkingBrake)
