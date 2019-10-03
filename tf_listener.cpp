#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Model.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "tf2_ros/buffer.h"
#include <vector>

std::vector<osrf_gear::Order> order_vec;

osrf_gear::LogicalCameraImage camera_msg;

void receiveOrder(const osrf_gear::Order &order) {
    order_vec.push_back(order);
}

void seeObjects(const osrf_gear::LogicalCameraImage &msg) {
    camera_msg = msg;
}

int main(int argc, char** argv) {
    order_vec.clear();

    //Initialize ROS node
    ros::init(argc, argv, "tf_listener_node");
    ros::NodeHandle n;

    // Trigger the start of the competition

    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger begin_comp;
    begin_client.call(begin_comp);


    // Warning message?
    //ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());

    // Subscriber to receive orders
    ros::Subscriber order_sub = n.subscribe("/ariac/orders", 1000, receiveOrder); 

    // Subscriber to receive camera location info
    ros::Subscriber cam_sub = n.subscribe("/ariac/logical_camera", 1000, seeObjects); 


    // Transformation buffer
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    // Instance of a move group for MoveIt to create motion plans
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    std::string move_group_name = move_group.getPlanningFrame().c_str();
    ROS_INFO("Manipulator Frame: %s\n", move_group.getPlanningFrame().c_str());
    ROS_INFO("End Effector Frame: %s\n", move_group.getEndEffectorLink().c_str());

    // Retrieve the transformation
    geometry_msgs::TransformStamped tfStamped;
    try {
        tfStamped = tfBuffer.lookupTransform(
            move_group_name.substr(1, sizeof(move_group_name)),
            "logical_camera_frame",
            ros::Time(0.0),
            ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    // tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait");

    // Create Variables
    geometry_msgs::PoseStamped part_pose, goal_pose;

    // Copy pose from the logical camera.
    part_pose.pose = camera_msg.pose;

    tf2::doTransform(part_pose, goal_pose, tfStamped);

    // See the first piston part from the camera
    osrf_gear::Model piston_part = camera_msg.models[0];

    ROS_INFO("part_pose x: %f", part_pose.pose.position.x);
    ROS_INFO("part_pose y: %f", part_pose.pose.position.y);
    ROS_INFO("part_pose z: %f", part_pose.pose.position.z);

    // Add height to goal pose.
    goal_pose.pose.position.x = -0.50; // 10 cm above the part
    goal_pose.pose.position.y = 0.030; // 10 cm above the part
    goal_pose.pose.position.z = 0.824951; // 10 cm above the part

    ROS_INFO("goal_pose x: %f", goal_pose.pose.position.x);
    ROS_INFO("goal_pose y: %f", goal_pose.pose.position.y);
    ROS_INFO("goal_pose z: %f", goal_pose.pose.position.z);


    // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions)
    //goal_pose.pose.orientation.w = 0.707;
    //goal_pose.pose.orientation.x = 0.0;
    //goal_pose.pose.orientation.y = 0.707;
    //goal_pose.pose.orientation.z = 0.0;


    // Set the desired pose for the arm in the arm controller
    move_group.setPoseTarget(goal_pose);

    // Instantiate and create a plan
    moveit::planning_interface::MoveGroupInterface::Plan the_plan;
    ROS_INFO("Plan instantiated");

    // Create a plan based on teh settings (all default settings now) in the_plan.
    
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok()) {
        if (order_vec.size() > 0) {
            //ROS_INFO("Order Number: %d", order_vec.size());
            if (move_group.plan(the_plan)) {
                ROS_INFO("Plan Successful");
                // In the event the plan was created, execute.
                move_group.execute(the_plan);
            } else {
                ROS_INFO("Plan Failed");
            }
            order_vec.pop_back();
            ros::spinOnce();
        }
    }

}

