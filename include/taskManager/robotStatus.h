#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <map>
#include "tcpsocket.hpp"
#include <iostream>
#include "task.h"
#include <bitset>

#include <cmath>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include <task_manager_llm/robot_data.h>

using namespace std;


class robotStatus {
    
public:
    robotStatus();
    int setRobotName(string name);
    string getRobotName();


    // Place tray정보를 status에 저장하기 위한 함수
    //int setPlaceTray(const std::map<std::string, std::shared_ptr<std::vector<int>>>& number);
    int setPlaceTray(Json::Value number);
    int setHomePlace(string home);
    int setStartPlace(string start);
    int setRobotID(string ID);
    int setIP(string ip);    

    int setNucTime(string nuctime);
    int setConaTime(string conatime);
    int setVisionTime(string visiontime);
    int setNodeStatus(string nodestatus);

    int setPosition(float x, float y) {
        posx = x*1000;
        posy = y*1000;
    }
    int setOrientation(float o) {
        orientation = o*1000;
    }

    int updateNaviStatus(Json::Value nStatus);
    int setNaviGoal(string goal);
    
    int setSkillInfo(string skill, int skillID);
    int writeRobotStatus(Json::Value &nStatus);
    
    int setWholeSequence(Json::Value eachTask);
    int writeSkillSequence(Json::Value &skillSeq);

    int writeRobotinitdata(Json::Value &InitData); //tray info

    int writeRobotNodeStatusdata(Json::Value &NodeData);
    
    int setNotice(string noticeStr);
    int setArmInfo(const task_manager_llm::robot_data::ConstPtr& msg);

    int* getarm1axis();
    int setjoyStatus(string joy);
    int setArmStatus(int armStatus);
    int setTrayUsing(int num);

    int setCamStatus(string camstatus);
    int setHoldStatus(string holdstatus);
    int setdeliveryCheck(string check);
    int setbattery1Remain(int b1r);
    int setbattery2Remain(int b2r);
    int setfloorID(int floorID);
    int getfloorID();

    int setmanipModuleStatus(int ms);

    // int addTask(Json::Value nStatus);
    // int addDeliveryTask(int trayID, int locationID);
    // int listTask();

    //map<string,int(*)(Json::Value, robotStatus*)> addSubTaskFunctions;

    // static int addSubTask_WAIT   (Json::Value params, robotStatus* pt);
    // static int addSubTask_DELIVER(Json::Value params, robotStatus* pt);
    // static int addSubTask_HOME   (Json::Value params, robotStatus* pt);
    // static int addSubTask_CHARGE (Json::Value params, robotStatus* pt);
    // static int addSubTask_ESTOP  (Json::Value params, robotStatus* pt);

private:
    string robotName;       // name of this robot
    string robotID;         // unique id of this robot
    string IP_address;      // network IP address

    std::map<std::string, std::shared_ptr<std::vector<int>>> PlaceTrayCor; // place tray list
    Json::Value placeTrayJson;

    string homeplace;
    string startplace;
    string naviReady2Work;  // 'ready', 'localizing', 'nomap'
    string naviType;       // 'goto', 'approach', 'arrived', 'stop'
    string naviStatus;      // 'stop-obs', 'stop-ems', 'moving', 'avoid'
    string naviGoal = "None";
    // int    naviTo;          // destination ID
    // int    naviNext;        // next destination ID
    // int    naviProgress;    // percentile to the destination (1~100)

    int posx=-1;             // x-position (for later use)
    int posy=-1;             // y-position (for later use)
    int orientation=-1; 
    string mapName;         // map name

    string Nuc_time = "0";
    string Cona_time = "0";
    string Vision_time = "0";
    string Node_status = "0";
    // string manip1State;      // 'neutral','moveTo','moveBack','grip'
    // string manip2State;      // 'neutral','moveTo','moveBack','grip'
    string vision1State;      // 'off','requested','responsed'
    string vision2State;      // 'off','requested','responsed'
    string deliveryCheck="None";
	
    string currentSkill="None";
    int currentSkillID=-1;
    int numTotalSkills=-1;
    Json::Value wholeSequence;
    string notice="";

    string arm_robot_state="None";
    int arm1axis[3]={-1, -1, -1};      // arm 1 axis angles (for later use)
    int arm2axis[3]={-1, -1, -1};      // arm 2 axis angles (for later use)
    int arm_1height=-1;
    int arm_2height=-1;
    float arm_tray=-1.0;
    float arm_front_pos[4]={-1.0, -1.0, -1.0, -1.0};
    float arm_backward_pos[4]={-1.0, -1.0, -1.0, -1.0};
    int arm_tray_status[16]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    bitset<16> trayStatusBit;
    int trayUsing=-1;

    int arm1Status=-1;; // armStatus
    string joy = "None";
    string camStatus="00";
    string holdStatus="off";
    // map<int,int> taskListDelivery; // tray and location ; primary key is tray.
    // vector<pair<int,int>> orderedTask; // after ordering the task.. 

    // int    batteryLevel;    // (1~100)
    int battery1Remain  = -1;
    int battery2Remain  = -1;

    int floor = -1;
    int manipModuleStatus = -1;

};

