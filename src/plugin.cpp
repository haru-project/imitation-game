#include <picture_server/SaveImage.h>  

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behavior_tree_ros/behavior_tree_ros.hpp>

BT_REGISTER_NODES(factory)
{
    using namespace BT_ROS;
        
    factory.registerNodeType<AutomaticServiceClient<picture_server::SaveImage>>("CaptureImage");

}
