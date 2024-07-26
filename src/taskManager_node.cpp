#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

#include "taskManager/taskManager.h"
using namespace std;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "task_manager_llm_node");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("rosTaskManager", 1000);
    ros::Rate loop_rate(10);
    
    taskManager tf(&nh);
    tf.loadConfig("/home/vision/llm_ws/src/task_manager_llm/src/config.json");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
