#include <fstream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
class MigrationRuntimeProbe : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    output_ = sdf->Get<std::string>("output");
    connection_ = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &) {
      if (done_) return;
      unsigned int plants = 0, links = 0, joints = 0;
      for (const auto &model : world_->Models()) {
        if (model->GetName().find("plant_") == std::string::npos) continue;
        ++plants;
        links += model->GetLinks().size();
        joints += model->GetJoints().size();
      }
      std::ofstream out(output_);
      out << "{\"world\":\"" << world_->Name() << "\",\"plants\":" << plants
          << ",\"plant_links\":" << links << ",\"plant_joints\":" << joints
          << ",\"sim_time\":" << world_->SimTime().Double() << "}\n";
      done_ = true;
    });
  }
 private:
  physics::WorldPtr world_;
  event::ConnectionPtr connection_;
  std::string output_;
  bool done_ = false;
};
GZ_REGISTER_WORLD_PLUGIN(MigrationRuntimeProbe)
}
