#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {

    int num_blocks = 3;
    // Option 2: Insert model from string via function call.
    // Insert a sphere model from string
    for (int i = 0; i <  num_blocks; i++){
    sdf::SDF boxSDF;
        boxSDF.SetFromString(
          "<sdf version ='1.4'>\
              <model name ='box'>\
                <pose>1 0 0 0 0 0</pose>\
                <link name ='link'>\
                  <pose>" + std::to_string(0.5 * (i+1)) + " 0 0 0 0 0</pose>\
                  <collision name ='collision'>\
                    <geometry>\
                      <box><size>0.1 0.1 0.1</size></box>\
                    </geometry>\
                  </collision>\
                  <visual name ='visual'>\
                    <geometry>\
                      <box><size>0.1 0.1 0.1</size></box>\
                    </geometry>\
                  </visual>\
                </link>\
              </model>\
            </sdf>");
        // Demonstrate using a custom model name.
        sdf::ElementPtr model = boxSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString("block " + std::to_string(i));
        _parent->InsertModelSDF(boxSDF);
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
