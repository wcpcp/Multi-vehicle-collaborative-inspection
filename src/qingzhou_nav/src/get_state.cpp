#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

std::string robot_name_odom;
std::string robot_name;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"gazebo_odom");
    ros::NodeHandle n;

    n.getParam("robot_name_odom",robot_name_odom);
    n.getParam("robot_name",robot_name);

    ros::ServiceClient states_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>(robot_name_odom,10);

    static tf::TransformBroadcaster odom_br;//tf广播
    tf::Transform transform;
    tf::Quaternion quaternion;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        gazebo_msgs::GetModelState model_states;
        nav_msgs::Odometry odom;

        model_states.request.model_name = robot_name;   //gazebo中查看的名字
        model_states.request.relative_entity_name = "world";
        states_client.call(model_states);

        odom.pose.pose = model_states.response.pose;
        odom.twist.twist = model_states.response.twist;
        bool success = model_states.response.success;
        odometry_pub.publish(odom);

        tf::quaternionMsgToTF(odom.pose.pose.orientation, quaternion);  //quaternion  msg转tf
        transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z));
        transform.setRotation(quaternion);
        odom_br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),robot_name+"/odom",robot_name+"/base_link"));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}