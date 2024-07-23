#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
// #include "tcpsocket.hpp"
// #include <iostream>

// #include <json/json.h>
// #include <json/writer.h>

// using namespace std;

// vector <int> location;
// vector <int> task;

// int onMessageJobSequence(Json::Value &rMessage) {
//     cout << "jobsequence received.. calling path planner" << endl;
//     // parsing job sequence.. need to revise
//     // cout << rMessage << endl;
//     for (int ii=0; ii<rMessage["js"].size(); ii++) {
//         location.push_back(rMessage["js"][ii]["location"].asInt());
//         task.push_back(rMessage["js"][ii]["tray"].asInt());
//     }
//     for (int ii=0; ii<location.size(); ii++) {
//         cout << "location " << location[ii] << ", tray = " << task[ii] << endl;
//     }
//     return 0;
// }

// int onMessageRequestStatus(TCPSocket& tcpSocket) {
//     cout << "statusRequest received.. sending status" << endl;
//     {
//         Json::Value jsonData;
//         jsonData["type"] = "STATUS";
//         Json::Value robotStatus;
//         robotStatus["logKey"] = "sdfasdfasdead;";
//         robotStatus["rID"] = 45;
//         robotStatus["posx"] = 111.5;
//         robotStatus["posy"] = -22.5;
//         robotStatus["memo"] = "none";
//         jsonData["content"] = robotStatus;

//         Json::StreamWriterBuilder writer;
//         std::string jsonstring = Json::writeString(writer, jsonData);
//         std::cout << "from ros" << std::endl;
//         std::cout << jsonstring << std::endl;

//         std::cout << "jsonstring.size() = " << jsonstring.size() << std::endl;

//         // Send an initial buffer
//         tcpSocket.Send(jsonstring);
//     }
// }

#include "taskManager/taskManager.h"
using namespace std;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "pandemic_task_manager_ros_node");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("rosTaskManager", 1000);
    ros::Rate loop_rate(10);

    taskManager tf(&nh);
    tf.loadConfig("/home/vision/catkin_ws/src/pandemic_task_manager_ros/src/config.json");
    //tf.setRobotName("pandemic101");
    //tf.connectToServer("161.122.114.48",5000);
    // Initialize socket.
//     TCPSocket tcpSocket([](int errorCode, std::string errorMessage)
//                         { cout << "Socket creation error:" << errorCode << " : " << errorMessage << endl; });

//     // Start receiving from the host.
//     tcpSocket.onMessageReceived = [&](string message)
//     {
//         //here we put message handler
//         cout << "Message from the Server: " << message << endl;
//         // parse json message
//         Json::Value rMessage;
//         Json::Reader reader;
//         bool parsingSuccessful = reader.parse( message, rMessage );
//         if ( !parsingSuccessful )
//         {
//             cout << "Error parsing the string" << endl;
//         }
//         if (rMessage["command"]=="jobsequence") {
//             onMessageJobSequence(rMessage);
//         }  
//         else if (rMessage["command"]=="received") {
//             cout << message << endl;
//         }
//         else if (rMessage["command"]=="reqStatus") {
//             onMessageRequestStatus(tcpSocket);
//         }
//         else {
//             cout << "unknown command" << message << endl;
//         }


//     };
//     // If you want to use raw bytes instead of std::string:
//     /*
//     tcpSocket.onRawMessageReceived = [](const char* message, int length) {
//         cout << "Message from the Server: " << message << "(" << length << ")" << endl;
//     };
//     */

//     // On socket closed:
//     tcpSocket.onSocketClosed = [](int errorCode)
//     {
//         cout << "Connection closed: " << errorCode << endl;
//     };

//     // Connect to the host.
    
//    // You should do an input loop so the program will not end immediately:
//     // Because socket listenings are non-blocking.

//     {

//         Json::Value jsonData;
//         jsonData["type"] = "STATUS";
//         Json::Value robotStatus;
//         robotStatus["logKey"] = "ubunturos";
//         robotStatus["rID"] = 45;
//         robotStatus["posx"] = 123.5;
//         robotStatus["posy"] = -22.5;
//         robotStatus["memo"] = "none";
//         jsonData["content"] = robotStatus;

//         Json::StreamWriterBuilder writer;
//         std::string jsonstring = Json::writeString(writer, jsonData);
//         std::cout << "from ros" << std::endl;
//         std::cout << jsonstring << std::endl;

//         std::cout << "jsonstring.size() = " << jsonstring.size() << std::endl;

//         // Send an initial buffer
//         tcpSocket.Send(jsonstring);
//     }

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    // string input;
    // getline(cin, input);
    // while (input != "exit")
    // {
    //     tcpSocket.Send(input);
    //     getline(cin, input);
    // }

    // tcpSocket.Close();

    return 0;
}
