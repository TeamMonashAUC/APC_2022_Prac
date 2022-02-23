#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>  

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh_;
    ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "move_base_simple/goal", 1 );
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {    
        if (count == 0){
            std::string fixed_frame = "map";
            tf::Quaternion quat;
            quat.setRPY(0.0, 0.0, 1);
            tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(1, 1, 0.0)), ros::Time::now(), fixed_frame);
            geometry_msgs::PoseStamped goal;
            tf::poseStampedTFToMsg(p, goal);

            pub_.publish(goal);
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }
    }
    return 0;

}