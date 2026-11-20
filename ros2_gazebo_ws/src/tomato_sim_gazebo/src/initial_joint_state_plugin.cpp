#include <gazebo/common/Console.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace tomato_sim_gazebo
{
class InitialJointStatePlugin final : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    auto joint_element = sdf->GetElement("joint");
    while (joint_element)
    {
      const auto name = joint_element->Get<std::string>("name");
      const auto position = joint_element->Get<double>("position");
      auto joint = model->GetJoint(name);
      if (!joint || !joint->SetPosition(0, position))
      {
        gzerr << "Unable to initialize joint " << name << " on " << model->GetName() << "\n";
        return;
      }
      joint->SetVelocity(0, 0.0);
      joint_element = joint_element->GetNextElement("joint");
    }
    gzmsg << "Initialized physical joint positions for " << model->GetName() << "\n";
  }
};
}

GZ_REGISTER_MODEL_PLUGIN(tomato_sim_gazebo::InitialJointStatePlugin)
