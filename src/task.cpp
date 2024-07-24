#include "taskManager/task.h"

#include <chrono>

using namespace std;

int taskBase::invokeTask(){
    stage = 0;
    status = 1;

    subtask[stage]->invokeSubtask(); // start the subtask
};

int taskBase::stepForward() { // for testing
    stage++;
    status = 1;
    if (stage>=subtask.size()) { stage=subtask.size()-1; status =0; }
    else {
        cout << endl;
        subtask[stage]->invokeSubtask();
        cout << "currentSubTask = " << subtask[stage]->getSubtaskName() << endl;
    }
    return status;
}

int taskBase::stepBackward() { // 엘리베이터 내부에 사람이 있을때 이전 subtask로 이동하기 위해 생성
    stage--;
    status = 1;
    if (stage<0) { stage=0; status =0; }
    else {
        cout << endl;
        subtask[stage]->invokeSubtask();
        cout << "currentSubTask = " << subtask[stage]->getSubtaskName() << endl;
    }
    return status;
}

int taskBase::callback_Task(Json::Value js) {
   // cout << " in callback_Task : " << getTaskName() << ", status = " << status << endl;

    if (stage<subtask.size()) {
        int ret = subtask[stage]->callback_subtask(js); // result from each subtask
        if (ret==0) { // subtask done,
            subtask[stage]->endTask();
            stepForward();
        }
        else if (ret==2) {  // subtask를 다시 돌아가라는 의미
            subtask[stage]->endTask();   //endTask가 왜 넘어가냐
            stepBackward();
        }
        else if (ret==3) { // next task
            status = 0;
        }
    }

    //cout << " in callback_Task : " << getTaskName() << ", status = " << status << endl;

    return status;
};

int task_DELIVER::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
    taskBase::setTask(js,nodehandle,mPubs); 

    subtask_MOVETO* moveto = new subtask_MOVETO;
    moveto->setSubtask(js, mPubs);
    subtask_DETECT* detect = new subtask_DETECT;
    detect->setSubtask(js, mPubs);
    subtask_MANIPULATE* manipulate = new subtask_MANIPULATE;
    manipulate->setSubtask(js, mPubs);
	manipulate->setDetectPtr(detect);
    subtask_RESET_ARM_POSE* reset_arm_pose = new subtask_RESET_ARM_POSE;
    reset_arm_pose->setSubtask(js, mPubs);

    subtask.push_back(moveto);
    // subtask.push_back(detect);
    subtask.push_back(manipulate);    
    // subtask.push_back(reset_arm_pose);  // arm -1

    return 0;
};

int task_DEMODELIVER::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
    taskBase::setTask(js,nodehandle,mPubs); 

    subtask_DETECT* detect = new subtask_DETECT;
    detect->setSubtask(js, mPubs);
    subtask_MANIPULATE* manipulate = new subtask_MANIPULATE;
    manipulate->setSubtask(js, mPubs);
	manipulate->setDetectPtr(detect);

    // subtask.push_back(detect);
    subtask.push_back(manipulate);
    // subtask.push_back(deliverCheck);    
   
    return 0;
};


int task_SWITCHFLOOR::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs, TCPSocket* ts) {
    taskBase::setTask(js,nodehandle,mPubs); 

    subtask_RESET_ARM_POSE* reset_arm_pose = new subtask_RESET_ARM_POSE;
    reset_arm_pose->setSubtask(js, mPubs);
    subtask_MANIPULATE* manipulate = new subtask_MANIPULATE;
    manipulate->setSubtask(js, mPubs);
    subtask_MOVETO* moveto1 = new subtask_MOVETO;
    moveto1->setSubtask(js, mPubs);
    subtask_SET_EVDOCK* set_evdock = new subtask_SET_EVDOCK;
    set_evdock->setSubtask(js, mPubs);
    subtask_PREPARE_LOAD* prepare_load = new subtask_PREPARE_LOAD;
    prepare_load->setSubtask(js, mPubs, ts);    
    subtask_DECIDE_LOAD* decide_load = new subtask_DECIDE_LOAD;
    decide_load->setSubtask(js, mPubs, ts);  
    subtask_LOAD_ROBOT* load_robot = new subtask_LOAD_ROBOT;
    load_robot->setSubtask(js, mPubs, ts, 1);
    subtask_SWITCH* switch_floor = new subtask_SWITCH;
    switch_floor->setSubtask(js, mPubs, ts);  

    // subtask.push_back(reset_arm_pose);// arm -1
    subtask.push_back(moveto1);      // 엘리베이터 문 앞으로 이동
    subtask.push_back(set_evdock);  // go to , ems명령으로 사물인지 과정 세팅
    subtask.push_back(prepare_load); // call ev and door open
    subtask.push_back(decide_load);  // decide whether take on or not
    subtask.push_back(load_robot);      // to coga : load
    subtask.push_back(switch_floor); // to ev : go to other floor and door open

    return 0;
};

int task_MOVE::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
    taskBase::setTask(js,nodehandle,mPubs); 

    subtask_MOVETO* moveto = new subtask_MOVETO;
    moveto->setSubtask(js, mPubs);

    subtask.push_back(moveto);

    return 0;
};

int task_RESETARM::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
    taskBase::setTask(js,nodehandle,mPubs); 

    subtask_RESET_ARM_POSE* reset_arm_pose = new subtask_RESET_ARM_POSE;
    reset_arm_pose->setSubtask(js, mPubs);

    subtask.push_back(reset_arm_pose);  // arm -1

   
    return 0;
};

int task_HOME::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {
//int task_HOME::setTask(Json::Value js,ros::NodeHandle* nodehandle, ros::Publisher *cmdGUI1, ros::Publisher *pub_left1, ros::Publisher *pub_right1, ros::Publisher *conaGo_pub1) {
//    taskBase::setTask(js,nodehandle,cmdGUI1,pub_left1,pub_right1,conaGo_pub1); 
    taskBase::setTask(js,nodehandle,mPubs); 
    subtask_RESET_ARM_POSE* reset_arm_pose = new subtask_RESET_ARM_POSE;
    reset_arm_pose->setSubtask(js, mPubs);
    subtask_MANIPULATE* manipulate = new subtask_MANIPULATE;
    manipulate->setSubtask(js, mPubs);
    subtask_MOVETO* moveto = new subtask_MOVETO;
    moveto->setSubtask(js,mPubs);
    
    // subtask.push_back(manipulate); // arm -1
    // subtask.push_back(reset_arm_pose);
    subtask.push_back(moveto);
 
    return 0;
};

int task_DEMOHOME::setTask(Json::Value js,ros::NodeHandle* nodehandle, map<string,ros::Publisher*> mPubs) {

    taskBase::setTask(js,nodehandle,mPubs); 
    subtask_RESET_ARM_POSE* reset_arm_pose = new subtask_RESET_ARM_POSE;
    reset_arm_pose->setSubtask(js, mPubs);
    subtask_MANIPULATE* manipulate = new subtask_MANIPULATE;
    manipulate->setSubtask(js, mPubs);
    subtask_MOVETO* moveto = new subtask_MOVETO;
    moveto->setSubtask(js,mPubs);

 
    subtask.push_back(reset_arm_pose);
 
    return 0;
};

void taskBase::listSubTasks() {
     //vector<subtaskBase*> subtask;

     for (int ii=0; ii<subtask.size(); ii++)
        subtask[ii]->listSubtask();
}

int subtask_STOP::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){
    subtaskBase::setSubtask(js, mPubs);
}
int subtask_STOP::invokeSubtask(){
    subtaskBase::invokeSubtask();
}
int subtask_STOP::callback_subtask(Json::Value js){

}

int subtask_MOVETO::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){
    subtaskBase::setSubtask(js, mPubs);    
    locationID = js["location"].asInt();
    locationStr = js["locationStr"].asString();
}

int subtask_MOVETO::invokeSubtask(){
    //subtaskBase::invokeSubtask();

    if (status==1) {
        cout << "something wrong ini subtask_MOVETO; status==1" << endl; 
        return -1;
    }
    status = 1;

    sleep(1);

    std_msgs::String speakermp3;
    speakermp3.data = "moveto_place";
    mPublishers["speaker_pub"]->publish(speakermp3); 

    std_msgs::String rosGoMsg;
    std::stringstream ssGo;

    ssGo << "goTo " << locationStr;

    rosGoMsg.data = ssGo.str();
    mPublishers["conaGo_pub"]->publish(rosGoMsg);

    cout << "Requested MOVETO" << endl;

    sleep(1);
}

int subtask_MOVETO::callback_subtask(Json::Value js){

    // original code

    cout << "here, subtask_MOVETO::callback_subtask(Json::Value js): status = " << status << endl;
    // cout << js << endl;
    if (status==-1)
        return -1;

    cout << "here, status = " << status;

    string naviReady2Work  = js["ready2work"].asString();
    string naviState       = js["type"].asString();
    string naviStatus      = js["status"].asString();
    
    // invokeholdtask(js);
    // string naviState = js["data"]
    
    if ((status)&&(naviState.find("arrived") != string::npos)) { 
        status = 0;
        cout << "subtask_MOVETO::callback_subtask(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "moveto_place_arrived";
        mPublishers["speaker_pub"]->publish(speakermp3); 
    }
    //cout << "here, status2 = " << status;
    
    return status;
}
int subtask_MOVETO::invokeholdtask(Json::Value js){

    std_msgs::String rosGoMsg;

    rosGoMsg.data = js["type"].asString();
    mPublishers["conaGo_pub"]->publish(rosGoMsg);  // "/cona/cmd" 토픽으로 subtask 보내줌(모든 subtask동일)

    if (js["type"].asString() == "stop"){
        cout << "Requested HOLD ON" << endl;
    }
    else if (js["type"].asString() == "go"){
        cout << "Requested HOLD OFF" << endl;
    }

    sleep(1);
}

//subtask_DETECT::subtask_DETECT() {};
//subtask_DETECT::~subtask_DETECT() {};

int subtask_DETECT::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){
    subtaskBase::setSubtask(js, mPubs);    
    // request_side = js["CameraSide"].asInt();
    // cout << js << endl;
    if(js["tray"].asInt() >= 1 && js["tray"].asInt() <= 8) request_side = 0;
    else if (js["tray"].asInt() >= 9 && js["tray"].asInt() <= 16) request_side = 1;
    cout << request_side << endl;
}

int subtask_DETECT::invokeSubtask() {
    //subtaskBase::invokeSubtask();
    cout << "subtask_DETECT::invokeSubtask" << endl;
    if (status==1) {
        cout << "something wrong in subtask_DETECT::invokeSubtask; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::Bool msgJS;
    msgJS.data = false;
    // cout << request_side << endl;
    if(request_side == 0)
    {
        cout << "0" << request_side << endl;
        mPublishers["pub_left"]->publish(msgJS);
    }
    else if(request_side == 1)
    {
        cout << "1" << request_side << endl;
        mPublishers["pub_right"]->publish(msgJS);
    }
    else if(request_side == 2)
    {
        cout << "2" << request_side << endl;
        mPublishers["pub_left"]->publish(msgJS);
        mPublishers["pub_right"]->publish(msgJS);
    }

}
int subtask_DETECT::callback_subtask(Json::Value js){
    if (status==-1)
        return -1;
    string msgfrom  = js["type"].asString();
    if (msgfrom=="detect") {
        cout << "detect message arrived" << endl;
        status = 0;
        cout << "subtask_DETECT::callback_subtask(Json::Value js) : arrived"  << endl;

        // store data (maybe for later use)
        string side = js["direction"].asString();
        if (side=="left") {
            if (request_side==1) {
                cout << "subtask_DETECT::callback_subtask;; something wrong. right side requested, but left gives the answer." << endl;
            }
            detected_side = 0;
        }
        if (side=="right") {
            if (request_side==0) {
                cout << "subtask_DETECT::callback_subtask;; something wrong. left side requested, but right gives the answer." << endl;
            }
            detected_side = 1;
        }
        if(js["orientation"][0].asFloat() == 0 || js["position"][0].asFloat() <= 0){
            status = 3;
            std_msgs::String speakermp3;
            speakermp3.data = "delivery_fail";
            mPublishers["speaker_pub"]->publish(speakermp3); 
        }
        else {
            msgr.pose.position.x =js["position"][0].asFloat();
            msgr.pose.position.y =js["position"][1].asFloat();
            msgr.pose.position.z =js["position"][2].asFloat();
            msgr.pose.orientation.w = js["orientation"][0].asFloat();
            msgr.pose.orientation.x = js["orientation"][1].asFloat();
            msgr.pose.orientation.y = js["orientation"][2].asFloat();
            msgr.pose.orientation.z = js["orientation"][3].asFloat();
        }

    }
    else {
        cout << "subtask_DETECT::callback_subtask::::ignore message" << endl;
    }
    return status;
}
//subtask_MANIPULATE::subtask_MANIPULATE() {};
//subtask_MANIPULATE::~subtask_MANIPULATE() {};

int subtask_MANIPULATE::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){
    subtaskBase::setSubtask(js, mPubs);    
    trayID = js["tray"].asInt();

}

void subtask_MANIPULATE::setDetectPtr(subtask_DETECT* ptr)
{
   resultDetect = ptr;
}

int subtask_MANIPULATE::invokeSubtask(){
    //subtaskBase::invokeSubtask();
    cout << "subtask_MANIPULATE::invokeSubtask" << endl;
    if (status==1) {
        cout << "something wrong in subtask_MANIPULATE::invokeSubtask; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::String speakermp3;
    speakermp3.data = "delivery_start";
    mPublishers["speaker_pub"]->publish(speakermp3); 

    std_msgs::Float32MultiArray array;

    trayIDlist = std::to_string(trayID);
    float trayArray[3] = {0.0f};
    for (int i=0;i<trayIDlist.length();i++){
        trayArray[i] = trayIDlist[i] - '0';
    }
    // if (trayIDlist.length() == 1) array.data[0] = std::stoi(trayIDlist[0]);
    // else if (trayIDlist.length() == 2) {
    //     array.data[0] = std::stoi(trayIDlist[0]);
    //     array.data[1] = std::stoi(trayIDlist[1]);
    // }
    // else if (trayIDlist.length() == 3) {
    //     array.data[0] = std::stoi(trayIDlist[0]);
    //     array.data[1] = std::stoi(trayIDlist[1]);
    //     array.data[2] = std::stoi(trayIDlist[2]);
    // }
    // array.data.push_back(resultDetect->msgr.pose.position.x );
    // array.data.push_back(resultDetect->msgr.pose.position.y );
    // array.data.push_back(resultDetect->msgr.pose.position.z );
    array.data.push_back(trayArray[0]);
    array.data.push_back(trayArray[1]);
    array.data.push_back(trayArray[2]);
    array.data.push_back(1.0);
    mPublishers["arm_pub"]->publish(array);

}
int subtask_MANIPULATE::callback_subtask(Json::Value js){
    if (status==-1)
        return -1;
    // if (pass_subtask == true) {
    //     status = 0; // subtask 넘기기
    //     pass_subtask = false;
    // }
    string msgfrom  = js["type"].asString();

    if (msgfrom=="manipulate") {
        cout << "manipulate message arrived" << endl;
        status = 0;
        cout << "subtask_MANIPULATE::callback_subtask(Json::Value js) : arrived"  << endl;

        // publish devliery check msg
        std_msgs::String delCheckMsg;
        std::stringstream delCheck;
        delCheck << "delivery_check";

        delCheckMsg.data = delCheck.str();
        mPublishers["deliverCheck_pub"]->publish(delCheckMsg);
        sleep(1);
    }
    else {
        cout << "subtask_MANIPULATE::callback_subtask::ignore message" << endl;
    }
    return status;
}

int subtask_RESET_ARM_POSE::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){  // arm reset subtask
    subtaskBase::setSubtask(js, mPubs);    
}

int subtask_RESET_ARM_POSE::invokeSubtask(){
    //subtaskBase::invokeSubtask();
    cout << "subtask_RESET_ARM_POSE::invokeSubtask" << endl;
    if (status==1) {
        cout << "something wrong in subtask_RESET_ARM_POSE::invokeSubtask; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::Float32MultiArray array;
    array.data.clear();
    array.data.push_back(-1); // arm의 초기 자리
    array.data.push_back(0);
    array.data.push_back(0);
    array.data.push_back(0);

    mPublishers["arm_pub"]->publish(array);
}
int subtask_RESET_ARM_POSE::callback_subtask(Json::Value js){
   if (status==-1)
        return -1;
    string msgfrom  = js["type"].asString();
    if (msgfrom=="ResetArm") {
        cout << "Arm Reset" << endl;
        status = 0;
        cout << "subtask_RESET_ARM_POSE::callback_subtask(Json::Value js) : arrived"  << endl;
     }
    else {
        cout << "subtask_RESET_ARM_POSE::callback_subtask::ignore message" << endl;
    }
    return status;
}

// 엘리베이터 앞 이동 subtask
int subtask_MOVETOEV::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){
    subtaskBase::setSubtask(js, mPubs);    
    locationID = js["location"].asInt();
    locationStr = js["locationStr"].asString();
}

int subtask_MOVETOEV::invokeSubtask(){
    //subtaskBase::invokeSubtask();

    if (status==1) {
        cout << "something wrong ini subtask_MOVETOEV; status==1" << endl; 
        return -1;
    }
    status = 1;

    sleep(1);

    std_msgs::String speakermp3;
    speakermp3.data = "moveto_ev";
    mPublishers["speaker_pub"]->publish(speakermp3); 

    std_msgs::String rosGoMsg;
    std::stringstream ssGo;

    ssGo << "goTo " << locationStr;

    rosGoMsg.data = ssGo.str();
    mPublishers["conaGo_pub"]->publish(rosGoMsg);

    cout << "Requested MOVETOEV" << endl;

    sleep(1);
}

int subtask_MOVETOEV::callback_subtask(Json::Value js){

    // original code

    cout << "here, subtask_MOVETOEV::callback_subtask(Json::Value js): status = " << status << endl;
    // cout << js << endl;
    if (status==-1)
        return -1;

    cout << "here, status = " << status;

    string naviReady2Work  = js["ready2work"].asString();
    string naviState       = js["type"].asString();
    string naviStatus      = js["status"].asString();
    
    // invokeholdtask(js);
    // string naviState = js["data"]
    
    if ((status)&&(naviState.find("arrived") != string::npos)) { 
        status = 0;
        cout << "subtask_MOVETOEV::callback_subtask(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "moveto_ev_arrived";
        mPublishers["speaker_pub"]->publish(speakermp3); 
    }
    //cout << "here, status2 = " << status;
    
    return status;
}

int subtask_MOVETOEV::invokeholdtask(Json::Value js){

    std_msgs::String rosGoMsg;

    rosGoMsg.data = js["type"].asString();
    mPublishers["conaGo_pub"]->publish(rosGoMsg);  // "/cona/cmd" 토픽으로 subtask 보내줌(모든 subtask동일)

    if (js["type"].asString() == "stop"){
        cout << "Requested HOLD ON" << endl;
    }
    else if (js["type"].asString() == "go"){
        cout << "Requested HOLD OFF" << endl;
    }

    sleep(1);
}


int subtask_SET_EVDOCK::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs){
    subtaskBase::setSubtask(js, mPubs);    
    evInsideStr_load = js["evInsideStr_load"].asString();
}

int subtask_SET_EVDOCK::invokeSubtask(){

    if (status==1) {
        cout << "something wrong ini subtask_SET_EVDOCK; status==1" << endl; 
        return -1;
    }
    status = 1;



    // sleep(1);

    std_msgs::String rosGoMsg;
    std::stringstream ssGo;
    cout << evInsideStr_load << endl;
    ssGo << "goTo " << evInsideStr_load;
    rosGoMsg.data = ssGo.str();
    mPublishers["conaGo_pub"]->publish(rosGoMsg);  
    cout << "Requested Loading" << endl; 

    // sleep(1);
    std_msgs::String rosStopMsg;
    std::stringstream ssStop;

    ssStop << "ems";

    rosStopMsg.data = ssStop.str();
    mPublishers["conaGo_pub"]->publish(rosStopMsg);  // 바로 stop

    cout << "Requested EMS" << endl;
    sleep(1);
}

int subtask_SET_EVDOCK::callback_subtask(Json::Value js){

    // original code

    cout << "here, subtask_SET_EVDOCK::callback_subtask(Json::Value js): status = " << status << endl;
    // cout << js << endl;
    if (status==-1)
        return -1;
    

    
    string naviReady2Work  = js["ready2work"].asString();
    string naviState       = js["type"].asString();
    string naviStatus      = js["status"].asString();
    
    cout << naviStatus << endl;
    if ((status)&&(naviStatus).find("stop-ems") != string::npos) { 
        status = 0;
        cout << "subtask_SET_EVDOCK::callback_subtask(Json::Value js) : STOP!"  << endl;
    }
    sleep(1);
    return status;

}





int subtask_PREPARE_LOAD::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    subtaskBase::setSubtask(js, mPubs);    

    source = js["sourceFloor"].asString(); // 3
    target = js["targetFloor"].asString(); // 1

    tcpSocket = ts;

}

int subtask_PREPARE_LOAD::invokeSubtask(){
    // subtaskBase::invokeSubtask();
    cout << "subtask_PREPARE_LOAD::invokeSubtask" << endl;
    if (status==1) {
        cout << "something wrong in subtask_PREPARE_LOAD::invokeSubtask; status==1" << endl; 
        return -1;
    }
    status = 1;
 
    std_msgs::String speakermp3;
    speakermp3.data = "ev_call";
    mPublishers["speaker_pub"]->publish(speakermp3); 
 
    Json::Value sendbuffer;
    sendbuffer["data"] = "prepareLoad"; 
    sendbuffer["source"] = source; 
    sendbuffer["target"] = target; 
    sendbuffer["lk"] = "pandemic201";
    Json::Value jsonData;
    jsonData["type"] = "evReq";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    std::cout << jsonstring << std::endl;

    tcpSocket->SendJson(jsonstring);
}


int subtask_PREPARE_LOAD::callback_subtask(Json::Value js){
    if (status==-1)
        return -1;

    string msgfrom  = js["type"].asString();
    // cout << msgfrom << endl;
    if (msgfrom=="completePrepareLoad") {
        status = 0;
        cout << "subtask_PREPARE_LOAD::callback_subtask(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "ev_call_finish";
        mPublishers["speaker_pub"]->publish(speakermp3); 
    }
    // else {
    //     cout << "subtask_PREPARE_LOAD::callback_subtask::::ignore message" << endl;
    // }
    return status;
}


int subtask_DECIDE_LOAD::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    subtaskBase::setSubtask(js, mPubs);    
    // auto start = std::chrono::high_resolution_clock::now();
    tcpSocket = ts;
    evDistance = js["distance"].asFloat();
}

int subtask_DECIDE_LOAD::invokeSubtask(){
    // subtaskBase::invokeSubtask();
    cout << "subtask_DECIDE_LOAD::invokeSubtask" << endl;
    if (status==1) {
        cout << "something wrong in subtask_DECIDE_LOAD::invokeSubtask; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::String speakermp3;
    speakermp3.data = "ev_decide";
    mPublishers["speaker_pub"]->publish(speakermp3); 

    std_msgs::Float64 dockdistance;
    dockdistance.data = evDistance; //mm단위
    cout << "Publish :" << dockdistance.data << endl;
    mPublishers["dockdt_pub"] -> publish(dockdistance);
    checkEVClose = false;
    sleep(1);
}


int subtask_DECIDE_LOAD::callback_subtask(Json::Value js){
    if (status==-1)
        return -1;

    distance = js["data"].asFloat();
    msgfrom = js["type"].asString();
    msgstatus = js["status"].asString();
    cout << distance << endl;
    cout << msgstatus << endl;
    cout << msgfrom << endl;
    
    if (msgstatus=="stop-ems") {
        // if (msgfrom=="evobsdt") {
        if (checkEVClose == false) {
            if(checkperson == false) {
                distances.push_back(distance);
                if(distances.size() == 5) checkperson = true;
            }
            else {
                cout << "start to check person!" << endl;
                int count = 0;
                for (size_t i = 0; i < distances.size(); i++){
                    if (distance >= 2400) count ++;
                }
                if (count >= 3) {
                    status = 0;
                    cout << "subtask_DECIDE_LOAD::callback_subtask(Json::Value js) : no person"  << endl;
                    distances.clear();
                    std_msgs::String speakermp3;
                    speakermp3.data = "ev_decide_no_obs";
                    mPublishers["speaker_pub"]->publish(speakermp3); 
                    checkperson = false;
                }
                else {
                    // status = 2;
                    cout << "subtask_DECIDE_LOAD::callback_subtask(Json::Value js) : person"  << endl;
                    Json::Value sendbuffer;
                    sendbuffer["data"] = "closeDoor";
                    sendbuffer["lk"] = "pandemic201";
                    Json::Value jsonData;
                    jsonData["type"] = "evReq";
                    jsonData["content"] = sendbuffer;
                    Json::StreamWriterBuilder writer;
                    std::string jsonstring = Json::writeString(writer, jsonData);
                    // std::cout << jsonstring << std::endl;
                    tcpSocket->SendJson(jsonstring);
                    distances.clear();
                    checkperson = false;
                    checkEVClose = true;

                    std_msgs::String speakermp3;
                    speakermp3.data = "ev_decide_obs";
                    mPublishers["speaker_pub"]->publish(speakermp3); 
                }
            } 
        }
        else {
            cout << "start ev status request" << endl;
            if (doorcheck1 == false){
                Json::Value sendbuffer;
                sendbuffer["data"] = "ReqEVstatus"; //ev문이 닫혔는지 아닌지 알기 위해 요청
                sendbuffer["lk"] = "pandemic201";
                Json::Value jsonData;
                jsonData["type"] = "evReq";
                jsonData["content"] = sendbuffer;
                Json::StreamWriterBuilder writer;
                std::string jsonstring = Json::writeString(writer, jsonData);
                tcpSocket->SendJson(jsonstring);
            }

            // cout << jsonstring << endl;
            if (msgfrom == "RePrepareLoad") {
                doorcheck1 = true;
                cout << "door closed based server" << endl;
            }
            
            if (doorcheck2 == false) distances.push_back(distance); // save distance
            cout << msgfrom << endl;
            cout << distance << endl;
            if(distances.size() == 7){
                cout << "check distance" << endl;
                int count = 0;
                for (size_t i = 0; i < distances.size(); i++){
                    if(distance < 8000 && distance > 0) count++;
                }
                if (count >= 4) {
                    cout << "door closed based distance" << endl;
                    doorcheck2 = true;
                }
                distances.clear(); // 다시 계산
            }
            if(doorcheck1 == true && doorcheck2 == true) {
                status = 2;
                cout << "subtask_DECIDE_LOAD::callback_subtask(Json::Value js) : RePrepareLoad"  << endl;
                checkEVClose = false;
                doorcheck1 = false;
                doorcheck2 = false;
            }
        }
    }
    else {
        cout << "subtask_DECIDE_LOAD::callback_subtask::::ignore message" << endl;
    }
    sleep(1);
    return status;
}






int subtask_LOAD_ROBOT::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts, int load){
    subtaskBase::setSubtask(js, mPubs);    

    tcpSocket = ts;
    chooseLoad = load; // 1 : load, 0 : unload

    if(chooseLoad == 1){
        evInsideStr_load = js["evInsideStr_load"].asString();
        whichMap = js["switchMap"].asString();
    }
    else if(chooseLoad == 0){ // if unload, robot have to load another floor map
        // whichMap = js["switchMap"].asString();
        evFrontStr_unload = js["evFrontStr_unload"].asString();
    }
}

int subtask_LOAD_ROBOT::invokeSubtask(){
    //subtaskBase::invokeSubtask();

    std_msgs::String map;
    std::stringstream mapLoad;

    if (status==1) {
        cout << "something wrong in subtask_LOAD_ROBOT; status==1" << endl; 
        return -1;
    }
    status = 1;

    sleep(1);

    // std_msgs::String rosGoMsg, evLoadMsg;
    // std::stringstream ssGo;

    // if(chooseLoad == 1){
    //     ssGo << "go"; // go명령만 주면 출발
    //     rosGoMsg.data = ssGo.str();
    //     mPublishers["conaGo_pub"]->publish(rosGoMsg);  
    //     cout << "Requested Loading" << endl; 
    // }

    std_msgs::String rosGoMsg, evLoadMsg;
    std::stringstream ssGo;

    if(chooseLoad == 1){
        ssGo << "goTo " << evInsideStr_load;
        rosGoMsg.data = ssGo.str();
        mPublishers["conaGo_pub"]->publish(rosGoMsg);  
        cout << "Requested Loading" << endl; 

        std_msgs::String speakermp3;
        speakermp3.data = "ev_load";
        mPublishers["speaker_pub"]->publish(speakermp3);
    }
    else if(chooseLoad == 0){ 
        // load another map        
        // map.data = whichMap.str();
        // mPublishers["map_loader"]->publish(map);
        // cout << "Requested Map change" << endl;

        // unload
        ssGo << "goTo " << evFrontStr_unload; // where to get out
        rosGoMsg.data = ssGo.str();
        mPublishers["conaGo_pub"]->publish(rosGoMsg);
        cout << "Requested Unloading" << endl;
    }

    sleep(1);
}

int subtask_LOAD_ROBOT::callback_subtask(Json::Value js){

    std_msgs::String map;
    std::stringstream mapLoad;
    string msgfrom  = js["type"].asString();
    cout << "here, subtask_LOAD_ROBOT::callback_subtask(Json::Value js): status = " << status << endl;

    if (status==-1)
        return -1;

    if(chooseLoad == 1){ // load complete
        if (msgfrom == "arrived"){
            status = 0;
            std_msgs::String speakermp3;
            speakermp3.data = "ev_load_finish";
            mPublishers["speaker_pub"]->publish(speakermp3);
            
            mapLoad << whichMap;
            map.data = mapLoad.str();
            mPublishers["map_loader"]->publish(map);
            cout << "Requested Map change" << endl;
        }
    }
    // else if(chooseLoad == 0){ // unload complete
    //     status = 0;

    //     // // close ev door
    //     // Json::Value sendbuffer;
    //     // sendbuffer["data"] = "closeDoor"; 

    //     // Json::Value jsonData;
    //     // jsonData["type"] = "evReq";
    //     // jsonData["content"] = sendbuffer;
    //     // Json::StreamWriterBuilder writer;
    //     // std::string jsonstring = Json::writeString(writer, jsonData);

    //     // std::cout << jsonstring << std::endl;

    //     // tcpSocket->SendJson(jsonstring);
    // }

    return status;
}

int subtask_SWITCH::setSubtask(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    subtaskBase::setSubtask(js, mPubs);   

    tcpSocket = ts;
    targetFloor = js["switchFloor"].asString();
}

int subtask_SWITCH::invokeSubtask(){
    // subtaskBase::invokeSubtask();
    cout << "subtask_SWITCH::invokeSubtask" << endl;
    if (status==1) {
        cout << "something wrong in subtask_SWITCH::invokeSubtask; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::String speakermp3;
    speakermp3.data = "ev_switch" + targetFloor;
    mPublishers["speaker_pub"]->publish(speakermp3);

    Json::Value sendbuffer;
    sendbuffer["data"] = "switchFloor"; 
    sendbuffer["lk"] = "pandemic201";
    Json::Value jsonData;
    jsonData["type"] = "evReq";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    // std::cout << jsonstring << std::endl;
    tcpSocket->SendJson(jsonstring);
}


int subtask_SWITCH::callback_subtask(Json::Value js){
    if (status==-1)
        return -1;

    string msgfrom  = js["type"].asString();
    if (msgfrom=="completeSwitchFloor") {
        status = 0;
        cout << "subtask_SWITCH::callback_subtask(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "ev_switch_finish" + targetFloor;
        mPublishers["speaker_pub"]->publish(speakermp3);

    }
    else {
        cout << "subtask_SWITCH::callback_subtask::::ignore message" << endl;
    }
    return status;
}


// -------------------------------------------------------------------------------------- //
// new code //

// js : params
int skill_MoveTo::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs){
    skillBase::set_skill(js, mPubs);    
    locationID = js["location"].asString();
    locationStr = js["locationStr"].asString();
}

int skill_MoveTo::invoke_skill(){
    if (status==1) {
        cout << "something wrong ini skill_MoveTo; status==1" << endl; 
        return -1;
    }
    status = 1;
    sleep(1);
    std_msgs::String speakermp3;
    speakermp3.data = "moveto_place";
    mPublishers["speaker_pub"]->publish(speakermp3); 
    std_msgs::String rosGoMsg;
    std::stringstream ssGo;
    ssGo << "goTo " << locationStr;
    rosGoMsg.data = ssGo.str();
    mPublishers["conaGo_pub"]->publish(rosGoMsg);
    cout << "Requested MOVETO" << locationID << endl;
    sleep(1);
}

int skill_MoveTo::callback_skill(Json::Value js){

    // original code
    cout << "here, skill_MoveTo::callback_skill(Json::Value js): status = " << status << endl;
    if (status==-1)
        return -1;

    cout << "here, status = " << status;

    string naviReady2Work  = js["ready2work"].asString();
    string naviState       = js["type"].asString();
    string naviStatus      = js["status"].asString();
    if ((status)&&(naviState.find("arrived") != string::npos)) {
        status = 0;
        cout << "skill_MOVETO::callback_skill(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "moveto_place_arrived";
        mPublishers["speaker_pub"]->publish(speakermp3); 
    }
    return status;
}

// -------------------------------------------------------------------------------------- //


int skill_Detect::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs){
    skillBase::setSkill(js, mPubs);    
    request_side = 0;
    // if(js["tray"].asInt() >= 1 && js["tray"].asInt() <= 8) request_side = 0;
    // else if (js["tray"].asInt() >= 9 && js["tray"].asInt() <= 16) request_side = 1;
    cout << request_side << endl;
}

int skill_Detect::invoke_skill() {
    cout << "skill_Detect::invoke_skill" << endl;
    if (status==1) {
        cout << "something wrong in skill_Detect::invoke_skill; status==1" << endl; 
        return -1;
    }
    status = 1;
    std_msgs::Bool msgJS;
    msgJS.data = false;
    if(request_side == 0)
    {
        cout << "0" << request_side << endl;
        mPublishers["pub_left"]->publish(msgJS);
    }
    else if(request_side == 1)
    {
        cout << "1" << request_side << endl;
        mPublishers["pub_right"]->publish(msgJS);
    }
    else if(request_side == 2)
    {
        cout << "2" << request_side << endl;
        mPublishers["pub_left"]->publish(msgJS);
        mPublishers["pub_right"]->publish(msgJS);
    }

}
int skill_Detect::callback_skill(Json::Value js){
    if (status==-1)
        return -1;
    string msgfrom  = js["type"].asString();
    if (msgfrom=="detect") {
        cout << "detect message arrived" << endl;
        status = 0;
        cout << "skill_Detect::callback_skill(Json::Value js) : arrived"  << endl;

        // store data (maybe for later use)
        string side = js["direction"].asString();
        if (side=="left") {
            if (request_side==1) {
                cout << "skill_Detect::callback_skill;; something wrong. right side requested, but left gives the answer." << endl;
            }
            detected_side = 0;
        }
        if (side=="right") {
            if (request_side==0) {
                cout << "skill_Detect::callback_skill;; something wrong. left side requested, but right gives the answer." << endl;
            }
            detected_side = 1;
        }
        if(js["orientation"][0].asFloat() == 0 || js["position"][0].asFloat() <= 0){
            status = 3;
            std_msgs::String speakermp3;
            speakermp3.data = "delivery_fail";
            mPublishers["speaker_pub"]->publish(speakermp3);
        }
        else {
            msgr.pose.position.x =js["position"][0].asFloat();
            msgr.pose.position.y =js["position"][1].asFloat();
            msgr.pose.position.z =js["position"][2].asFloat();
            msgr.pose.orientation.w = js["orientation"][0].asFloat();
            msgr.pose.orientation.x = js["orientation"][1].asFloat();
            msgr.pose.orientation.y = js["orientation"][2].asFloat();
            msgr.pose.orientation.z = js["orientation"][3].asFloat();
        }
    }
    else {
        cout << "skill_Detect::callback_skill::::ignore message" << endl;
    }
    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_Manipulate::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs){
    skillBase::set_skill(js, mPubs);    
    trayID = js["tray"].asInt();
}

int skill_Manipulate::invoke_skill(){
    cout << "skill_Manipulate::invoke_skill" << endl;
    if (status==1) {
        cout << "something wrong in skill_Manipulate::invoke_skill; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::String speakermp3;
    speakermp3.data = "delivery_start";
    mPublishers["speaker_pub"]->publish(speakermp3); 

    std_msgs::Float32MultiArray array;

    trayIDlist = std::to_string(trayID);
    float trayArray[3] = {0.0f};
    for (int i=0;i<trayIDlist.length();i++){
        trayArray[i] = trayIDlist[i] - '0';
    }
    array.data.push_back(trayArray[0]);
    array.data.push_back(trayArray[1]);
    array.data.push_back(trayArray[2]);
    array.data.push_back(1.0);
    mPublishers["arm_pub"]->publish(array);
}

int skill_Manipulate::callback_skill(Json::Value js){
    if (status==-1)
        return -1;

    string msgfrom  = js["type"].asString();

    if (msgfrom=="manipulate") {
        cout << "manipulate message arrived" << endl;
        status = 0;
        cout << "skill_Manipulate::callback_skill(Json::Value js) : arrived"  << endl;

        // publish devliery check msg
        std_msgs::String delCheckMsg;
        std::stringstream delCheck;
        delCheck << "delivery_check";

        delCheckMsg.data = delCheck.str();
        mPublishers["deliverCheck_pub"]->publish(delCheckMsg);
        sleep(1);
    }
    else {
        cout << "skill_Manipulate::callback_skill::ignore message" << endl;
    }
    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_PrepareLoad::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    skillBase::set_skill(js, mPubs);
    source = js["sourceFloor"].asString(); // 3
    tcpSocket = ts;
}

int skill_PrepareLoad::invoke_skill(){
    cout << "skill_PrepareLoad::invoke_skill" << endl;
    if (status==1) {
        cout << "something wrong in skill_PrepareLoad::invoke_skill; status==1" << endl;
        return -1;
    }
    status = 1;
 
    std_msgs::String speakermp3;
    speakermp3.data = "ev_call";
    mPublishers["speaker_pub"]->publish(speakermp3);
 
    Json::Value sendbuffer;
    sendbuffer["data"] = "prepareLoad";
    sendbuffer["source"] = source;
    sendbuffer["lk"] = "pandemic201";
    Json::Value jsonData;
    jsonData["type"] = "evReq";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    std::cout << jsonstring << std::endl;

    tcpSocket->SendJson(jsonstring);
}

int skill_PrepareLoad::callback_skill(Json::Value js){
    if (status==-1)
        return -1;
    string msgfrom  = js["type"].asString();
    if (msgfrom=="completePrepareLoad") {
        status = 0;
        cout << "skill_PrepareLoad::callback_skill(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "ev_call_finish";
        mPublishers["speaker_pub"]->publish(speakermp3); 
    }
    return status;
}