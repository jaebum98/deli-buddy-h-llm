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
#include <pandemic_task_manager_ros/PlaneEstimation.h>

// #include </home/jskimlab/Desktop/PandemicDeliveryBot/src/service_vision/plane_estimation/include/plane_estimation_class.h>

using namespace std;

enum Tasks {
    WAIT = 0,
    DELIVER,
    HOME,
    CHARGE,
    ESTOP,
};

enum SubTasks {
    STOP = 0,
    MOVETO,
    DETECT,
    MANIPULATE,
    RESET_ARM_POSE,
    MOVETOEV,
    SET_EVDOCK,
    PREPARE_LOAD,
    DECIDE_LOAD,
    LOAD_ROBOT,
    SWITCH
    // DELIVER_CHECK
};

class subtaskBase;

class taskBase {

public:
    taskBase() {};
    ~taskBase() {};
    
    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
        nh_ = *nodehandle;

        mPublishers = mPubs;
        
        cout << "setTask_"<<getTaskName()<< endl;
    };
    virtual int invokeTask();
    virtual int callback_Task(Json::Value js);
    
    virtual string getTaskName()=0;
    int endTask() {
        stage = -1;
        status = -1;
    }

    void listSubTasks();
    int stepForward();
    int stepBackward();

public:
    // ros related
    ros::NodeHandle nh_; 
    vector<subtaskBase*> subtask;

    int stage=-1;
    int status=-1;

    map<string,ros::Publisher*> mPublishers;
  //  ros::Publisher *cmdGUI, *pub_left, *pub_right, *conaGo_pub;

};


class task_DEMODELIVER : public taskBase {
    
public:
    task_DEMODELIVER() {};
    ~task_DEMODELIVER(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "DELIVER"; }
};

class task_WAIT : public taskBase {
    
public:
    task_WAIT() {};
    ~task_WAIT() {};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
        taskBase::setTask(js,nodehandle, mPubs); return 0;};
    virtual int invokeTask() { return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "WAIT"; }

};

class task_DELIVER : public taskBase {
    
public:
    task_DELIVER() {};
    ~task_DELIVER(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "DELIVER"; }
};

class task_MOVE : public taskBase {
    
public:
    task_MOVE() {};
    ~task_MOVE(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "MOVE"; }
};

class task_RESETARM : public taskBase {
    
public:
    task_RESETARM() {};
    ~task_RESETARM(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "RESETARM"; }
};


class task_SWITCHFLOOR : public taskBase {
    
public:
    task_SWITCHFLOOR() {};
    ~task_SWITCHFLOOR(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "SWITCHFLOOR"; }

};

class task_HOME : public taskBase {
    
public:
    task_HOME() {};
    ~task_HOME(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "HOME"; }
};

class task_DEMOHOME : public taskBase {
    
public:
    task_DEMOHOME() {};
    ~task_DEMOHOME(){};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs);
    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "HOME"; }
};

class task_CHARGE : public taskBase {
    
public:
    task_CHARGE() {};
    ~task_CHARGE() {};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {taskBase::setTask(js,nodehandle,mPubs); return 0;};

    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "CHARGE"; }
};

class task_ESTOP : public taskBase {
    
public:
    task_ESTOP() {};
    ~task_ESTOP() {};

    virtual int setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {taskBase::setTask(js,nodehandle,mPubs); return 0;};    virtual int invokeTask() {return taskBase::invokeTask();};
    virtual int callback_Task(Json::Value js) {return taskBase::callback_Task(js);};
    virtual string getTaskName() { return "ESTOP"; }
};

class subtaskBase {

public:
    // taskManager taskManager;

    subtaskBase() {};
    ~subtaskBase() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs) {
        cout << "      setSubtask_"<<getSubtaskName()<< endl;
        mPublishers = mPubs;

    };
    virtual int invokeSubtask() {status = 1; return status;};
    virtual int callback_subtask(Json::Value js)=0;
    virtual string getSubtaskName()=0;
    // virtual int invokeholdtask(Json::Value js)=0;
    virtual void listSubtask()=0;

    int endTask() { status=-1; return status;};

public:
    int status =-1;
    map<string,ros::Publisher*> mPublishers;

};

class subtask_STOP : public subtaskBase {

public:
    subtask_STOP() {};
    ~subtask_STOP() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "STOP";}

    virtual void listSubtask() {cout << "\t\t STOP"<< endl;}


};

class subtask_MOVETO : public subtaskBase {

public:
    subtask_MOVETO() {};
    ~subtask_MOVETO() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    int invokeholdtask(Json::Value js);
    virtual string getSubtaskName() {return "MOVETO";}

    virtual void listSubtask() {cout << "\t\t MOVETO  "<< locationID << endl;}

    int locationID;
    string locationStr;

};

class subtask_DETECT : public subtaskBase {

public:
    subtask_DETECT() {};
    ~subtask_DETECT() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "DETECT";}
    virtual void listSubtask() {cout << "\t\t DETECT  the requested side = "<< request_side << endl;}


    int request_side = 0;
    int detected_side;

    geometry_msgs::PoseWithCovariance msgr;
};

class subtask_MANIPULATE : public subtaskBase {

public:
    subtask_MANIPULATE() {};
    ~subtask_MANIPULATE() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "MANIPULATE";}

    virtual void listSubtask() {cout << "\t\t MANIPULATE  trayID = "<< trayID << endl;}

    void setDetectPtr(subtask_DETECT* ptr);

	subtask_DETECT* resultDetect = 0; 
    int trayID;
    string trayIDlist;
    bool pass_subtask = false;
};

class subtask_RESET_ARM_POSE : public subtaskBase {

public:
    subtask_RESET_ARM_POSE() {};
    ~subtask_RESET_ARM_POSE() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "RESET_ARM_POSE";}

    virtual void listSubtask() {cout << "\t\t RESET_ARM_POSE" << endl;}
};

class subtask_MOVETOEV : public subtaskBase {

public:
    subtask_MOVETOEV() {};
    ~subtask_MOVETOEV() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    int invokeholdtask(Json::Value js);
    virtual string getSubtaskName() {return "MOVETOEV";}

    virtual void listSubtask() {cout << "\t\t MOVETOEV  "<< locationID << endl;}

    int locationID;
    string locationStr;

};

class subtask_SET_EVDOCK : public subtaskBase {

public:
    subtask_SET_EVDOCK() {};
    ~subtask_SET_EVDOCK() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "SET_EVDOCK";}

    virtual void listSubtask() {cout << "\t\t SET_EVDOCK  "<< endl;}

    int chooseLoad;
    std::string evInsideStr_load;
};


// class subtask_RESET_ARM : public subtaskBase {

// public:
//     subtask_RESET_ARM() {};
//     ~subtask_RESET_ARM() {};

//     virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
//     virtual int invokeSubtask();
//     virtual int callback_subtask(Json::Value js);
//     virtual string getSubtaskName() {return "RESET_ARM";}

//     virtual void listSubtask() {cout << "\t\t RESET_ARM  "<< endl;}

// };

class subtask_PREPARE_LOAD : public subtaskBase {

public:
    subtask_PREPARE_LOAD() {};
    ~subtask_PREPARE_LOAD() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "PREPARE_LOAD";}

    virtual void listSubtask() {cout << "\t\t PREPARE_LOAD  "<< endl;}

    std::string source, target;

    TCPSocket* tcpSocket;

};

class subtask_DECIDE_LOAD : public subtaskBase {

public:
    subtask_DECIDE_LOAD() {};
    ~subtask_DECIDE_LOAD() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "DECIDE_LOAD";}

    virtual void listSubtask() {cout << "\t\t DECIDE_LOAD  "<< endl;}

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


class subtask_LOAD_ROBOT : public subtaskBase {

public:
    subtask_LOAD_ROBOT() {};
    ~subtask_LOAD_ROBOT() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPub, TCPSocket* ts, int load);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "LOAD_ROBOT";}

    virtual void listSubtask() {cout << "\t\t LOAD_ROBOT  " << endl;}

    int chooseLoad;
    std::string evInsideStr_load;
    std::string whichMap;
    std::string evInsideStr_unload, evFrontStr_unload;

    TCPSocket* tcpSocket;
};

class subtask_SWITCH : public subtaskBase {

public:
    subtask_SWITCH() {};
    ~subtask_SWITCH() {};

    virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts);
    virtual int invokeSubtask();
    virtual int callback_subtask(Json::Value js);
    virtual string getSubtaskName() {return "SWITCH";}

    virtual void listSubtask() {cout << "\t\t SWITCH  "<< endl;}

    std::string targetFloor;
    TCPSocket* tcpSocket;

};


// class subtask_DELIVER_CHECK : public subtaskBase {

// public:
//     subtask_DELIVER_CHECK() {};
//     ~subtask_DELIVER_CHECK() {};

//     virtual int setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs);
//     virtual int invokeSubtask();
//     virtual int callback_subtask(Json::Value js);
//     virtual string getSubtaskName() {return "DELIVER_CHECK";}

//     virtual void listSubtask() {cout << "\t\t DELIVER_CHECK  " << endl;}

// };



// Hold function
// int holdSubtask(Json::Value js);

