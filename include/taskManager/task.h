#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
// #include <taskManager/taskManager.h>


#include <vector>
#include <map>
#include "tcpsocket.hpp"
#include <iostream>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include <task_manager_llm/PlaneEstimation.h>

// #include </home/jskimlab/Desktop/PandemicDeliveryBot/src/service_vision/plane_estimation/include/plane_estimation_class.h>

using namespace std;

enum Skills {
    MoveTo,
    Detect,
    Manipulate,
    SetEvEms,
    PrepareLoad,
    DecideLoad,
    SwitchFloor
};

class skillBase {

public:
    // taskManager taskManager;

    skillBase() {};
    ~skillBase() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs) {
        cout << "      setSkill_"<<getSkillName()<< endl;
        mPublishers = mPubs;
    };
    virtual int invoke_skill() {status = 1; return status;};
    virtual int callback_skill(Json::Value js)=0;
    virtual string getSkillName()=0;
    virtual void listSkill()=0;

public:
    ros::NodeHandle nh_; 
    int status =-1;
    map<string,ros::Publisher*> mPublishers;

};


class skill_MoveTo : public skillBase {

public:
    skill_MoveTo() {};
    ~skill_MoveTo() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "MoveTo";}

    virtual void listSkill() {cout << "\t\t MoveTo  "<< locationID << endl;}

    string locationID;
    string locationStr;

};

class skill_Detect : public skillBase {

public:
    skill_Detect() {};
    ~skill_Detect() {};
    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "Detect";}

    virtual void listSkill() {cout << "\t\t Detect  the requested side = "<< request_side << endl;}

    int request_side = 0;
    int detected_side;

    geometry_msgs::PoseWithCovariance msgr;
};

class skill_Manipulate : public skillBase {

public:
    skill_Manipulate() {};
    ~skill_Manipulate() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "Manipulate";}

    virtual void listSkill() {cout << "\t\t Manipulate  trayID = "<< trayID << endl;}

    void setDetectPtr(skill_Detect* ptr);

	skill_Detect* resultDetect = 0; 
    int trayID;
    string trayIDlist;
};

class skill_PrepareLoad : public skillBase {

public:
    skill_PrepareLoad() {};
    ~skill_PrepareLoad() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "PrepareLoad";}

    virtual void listSkill() {cout << "\t\t PrepareLoad  source floor = " << sourceFloor << endl;}
    std::string sourceFloor;
    TCPSocket* tcpSocket;
};

class skill_SetEvEms : public skillBase {

public:
    skill_SetEvEms() {};
    ~skill_SetEvEms() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "SetEvEms";}

    virtual void listSkill() {cout << "\t\t SetEvEms" << endl;}

    string locationID;
    string locationStr;

};

class skill_DecideLoad : public skillBase {

public:
    skill_DecideLoad() {};
    ~skill_DecideLoad() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "DecideLoad";}

    virtual void listSkill() {cout << "\t\t DecideLoad  "<< endl;}

    float evDistance;
    bool checkperson = false;
    bool checkEVClose = false;
    bool doorcheck2 = false;
    bool doorcheck1 = false;
    TCPSocket* tcpSocket;
    string msgstatus;
    string msgfrom;
    float distance;
    std::vector<float> distances;
};

class skill_SwitchFloor : public skillBase {

public:
    skill_SwitchFloor() {};
    ~skill_SwitchFloor() {};

    virtual int set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invoke_skill();
    virtual int callback_skill(Json::Value js);
    virtual string getSkillName() {return "SwitchFloor";}

    virtual void listSkill() {cout << "\t\t SwitchFloor  targetfloor : " << targetFloor << endl;}

    std::string whichMap;
    std::string targetFloor;
    TCPSocket* tcpSocket;
};
