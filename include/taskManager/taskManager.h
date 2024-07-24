#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include "tcpsocket.hpp"
#include <iostream>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include "taskManager/robotStatus.h"
#include <pandemic_task_manager_ros/PlaneEstimation.h>
#include <pandemic_task_manager_ros/robot_data.h>
#include "std_srvs/Trigger.h"
#include "pandemic_task_manager_ros/taskmanager_srv.h"
// #include </home/jskimlab/Desktop/PandemicDeliveryBot/src/service_vision/plane_estimation/include/plane_estimation_class.h>

//#include "taskManager/task.h"

using namespace std;


class taskManager 
{
public:
    taskManager(ros::NodeHandle* nodehandle); //생성자

    int loadConfig(string);

    int setRobotName(string name);
    int setRobotID(string robotID);
    int setLocationCorres(Json::Value js);
    int setHomeLocation(string homeloc);
    int setStartLocation(string startloc);
    int setEvLoc(Json::Value js);
    int setEvDt(Json::Value js);
    int setLoadMap(Json::Value js);
    int setTrayNumber(Json::Value js); // tray info
    int initializeLoadmap();

    void ServerConnectCallback(const ros::TimerEvent&);
    int connectToServer(std::string host, uint16_t port);
    int connectToServerev(std::string host, uint16_t port);

    // static int addSkill_WAIT         (Json::Value params, taskManager* pt);
    static int addSkill_DELIVER      (Json::Value params, taskManager* pt);
    static int addSkill_MOVE         (Json::Value params, taskManager* pt);
    static int addSkill_RESETARM     (Json::Value params, taskManager* pt);
    static int addSkill_HOME         (Json::Value params, taskManager* pt);
    static int addSkill_DEMOHOME     (Json::Value params, taskManager* pt);
    static int addSkill_CHARGE       (Json::Value params, taskManager* pt);
    static int addSkill_ESTOP        (Json::Value params, taskManager* pt);
    static int addSkill_DEMODELIVER  (Json::Value params, taskManager* pt);
    static int addSkill_SWITCHFLOOR  (Json::Value params, taskManager* pt);

    int addSkill(Json::Value nStatus);
    int addDeliveryTask(int trayID, int locationID);
    int listTask();

    map<string,int(*)(Json::Value, taskManager*)> addSkillFunctions;
    vector <taskBase*> vectorTask;
    vector <subtaskBase*> vectorSubTask;

    ros::NodeHandle nh_; 
    int currentTask;

    map<string,string>  evLocation;
    map<string,float>  evDistance;
    map<string,string>  locationCorres;
    map<string,string>  mapNumCor;
    string home;
    string start;

    int netstatus = 0;

    std::string serverAddress;
    uint16_t serverPort;
    uint16_t serverPortev;

public:

    robotStatus currentStatus;

    // Functions
    void NucManiClientservice();
    void NucClientservice(std::string nodename);
    void ConaClientservice(std::string nodename);
    void VisionClientservice(std::string nodename);
    // void statusCallback(const std_srvs::Trigger::Response& res);
    // ros related

    void initializeTimer();
    void initializeSubscribers();
    void initializePublishers();
    void initializeServiceClients();
    void naviStatusCallback(const std_msgs::String::ConstPtr& msg);
    void leftPoseCallback(const pandemic_task_manager_ros::PlaneEstimation::ConstPtr& msg);
    void rightPoseCallback(const pandemic_task_manager_ros::PlaneEstimation::ConstPtr& msg);
    void cmdMappingNavi(const std_msgs::String::ConstPtr& msg);
    void armStatusCallback(const std_msgs::Int32::ConstPtr& msg);
    void deliverCheckCallback(const std_msgs::String::ConstPtr& msg);
    void absolPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void manualCommandCallback(const std_msgs::String::ConstPtr& msg);
    void armInfoCallback(const pandemic_task_manager_ros::robot_data::ConstPtr& msg);
    // void evLoadStatusCallback(const std_msgs::String::ConstPtr& msg);
    void edockresultCallback(const std_msgs::Float64::ConstPtr& msg);
    void keyboardCallback(const std_msgs::String::ConstPtr& msg);
    void cameraStatusCallback(const std_msgs::String::ConstPtr& msg);
    void deliveryStatusCallback(const std_msgs::Bool::ConstPtr& msg); //임시
    // comm related
    int onMessageJobSequence(Json::Value &rMessage);
    int onMessageRequestStatus(TCPSocket* tcpSocket);
    int onMessageRequestStart(TCPSocket* tcpSocket);
    int onMessageRequestTaskClear(TCPSocket* tcpSocket);
    int onMessageRequestRosCmd(Json::Value &rMessage);
    int onMessageRequestEmergency(TCPSocket* tcpSocket); 
    int onMessageRequestStopWait(Json::Value &rMessage);
    int onMessageResponseRePrepareLoad(TCPSocket* tcpSocket);
    int onMessageResponsePrepareLoad(TCPSocket* tcpSocket);
    int onMessageResponseLoadManually(TCPSocket* tcpSocket); 
    int onMessageResponseSwitchFloor(TCPSocket* tcpSocket);
    int onMessageResponseLoadMap(Json::Value &rMessage);
    int onMessageRequestNodeStatus(Json::Value &rMessage);
    int onMessageRequestReboot(Json::Value &rMessage);
    int onMessageManualNextTask(Json::Value &rMessage);

    //전시회 용
    int onMessageRequestStartOffline();
    int onMessageRequestTaskClearOffline();

    int callTaskCallback(Json::Value js);
    int clearTasks();

    int onSendConnect(TCPSocket* tcpSocket);

    void sendStatusToServerCallback(const ros::TimerEvent&);
    int sendStatusToServer();
    int sendTaskSeqToServer();
    int sendInitDataToServer(); // task info

    // void sendJsonToServer(std::string jsonstring);
    // int processStatus(int status){
    //     int nextstatus;
    //     if(status == 0) nextstatus = 1;
    //     else if(status == 1) nextstatus = 0;
    //     return nextstatus;
    // }

    string getSubtaskName();

    // for decide_load subtask
    float evobs;
    string naviStatus;
    // variables

    // task related
    vector <int> location;
    vector <int> task;

   // comm related
    TCPSocket *tcpSocket;
    TCPSocket *tcpSocketev;
    
    // ros related
    //ros::Subscriber navi_status_sub, cmd_operation_sub; 
    //ros::Publisher cmdGUI, conaGo_pub;

    ros::Timer send_status_timer, serverconnect_timer;

    ros::Publisher cmdGUI, pub_left, pub_right, conaGo_pub;
    ros::Publisher  arm_pub, deliverCheck_pub, emergency_pub, ev_load_pub, map_loader,dockdt_pub, speaker_pub;

    ros::Subscriber navi_status_sub, cmd_operation_sub, left_pose_sub, right_pose_sub, sub_left_plane, camera_status_sub; 
    ros::Subscriber arm_status_sub,abs_pose_sub, deliver_check_sub, manual_command_sub, arm_info_sub, ev_load_status_sub, dock_check_sub, keyboard_input_sub;
    ros::Subscriber deliver_status_sub; //임시

    ros::ServiceClient vision_client, cona_client, nuc_client;

    map<string,ros::Publisher*> mapPublishers;
    
    map<int,int> taskListDelivery; // tray and location ; primary key is tray. // insert 부분까지 주석처리
    vector<pair<int,int>> orderedTask; // after ordering the task.. 

    bool emergencyFlag = false;
    int recentArmStatus = 2;
    bool maniCheck = false;

    bool leftcamCheck = false;
    bool rightcamCheck = false;

    string current_nuc_time;
    string current_cona_time;
    string current_vision_time;

    string recentdeliveryCheck = "prepare";
    bool ems = false;
    int obscheck = 0;
    // int absol_pose_cnt = 0; // absol pose callback func cnt (only call when cnt is 10)
};


