#include <gazebo/common/Console.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace tomato_sim_gazebo
{
class GripperCollisionFilter final : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) override
  {
    // Match the standalone MJCF's left/right finger contact exclusion.
    // Distinct categories exclude only this internal pair. Both fingers
    // retain contact with the scene, fruit, peduncle and robot housing.
    constexpr unsigned int left_category = 0x0100u;
    constexpr unsigned int right_category = 0x0200u;
    const auto left = model->GetLink("left_finger_link");
    const auto right = model->GetLink("right_finger_link");
    if (!left || !right)
    {
      gzerr << "Gripper contact filter requires both finger links\n";
      return;
    }
    for (const auto &collision : left->GetCollisions())
    {
      collision->SetCategoryBits(left_category);
      collision->SetCollideBits(0xffffu & ~right_category);
    }
    for (const auto &collision : right->GetCollisions())
    {
      collision->SetCategoryBits(right_category);
      collision->SetCollideBits(0xffffu & ~left_category);
    }
    gzmsg << "Excluded internal left/right finger contact for " << model->GetName() << "\n";
  }
};
}

GZ_REGISTER_MODEL_PLUGIN(tomato_sim_gazebo::GripperCollisionFilter)
