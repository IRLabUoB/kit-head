
#include <kit_head_hw/kit_head_hw_sim_plugin.h>

namespace kit_head_hw
{


// Overloaded Gazebo entry point
void KITHeadHWSimPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  GazeboRosControlPlugin::Load(parent, sdf);
}



// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(KITHeadHWSimPlugin);

} // namespace
