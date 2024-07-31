#include "taskManager/taskManager.h"
#include <fstream>
#include <tf/tf.h>
#include <iostream>
#include <unistd.h> // for sleep()


// 서버주소 전역변수
std::string serverAddress;
uint16_t serverPort;

taskManager::taskManager(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    cout << "taskManager::taskManager"  << endl;

    // add skill functions
    addSkillFunctions["MoveTo"]=&addSkill_MoveTo; 
    addSkillFunctions["Detect"]=&addSkill_Detect; 
    addSkillFunctions["Manipulate"]=&addSkill_Manipulate;
    addSkillFunctions["SetEvEms"]=&addSkill_SetEvEms;
    addSkillFunctions["PrepareLoad"]=&addSkill_PrepareLoad;
    addSkillFunctions["DecideLoad"]=&addSkill_DecideLoad;
    addSkillFunctions["SwitchFloor"]=&addSkill_SwitchFloor;

    // comm related
    tcpSocket = new TCPSocket([](int errorCode, std::string errorMessage)
                        { cout << "Socket creation error:" << errorCode << " : " << errorMessage << endl; });
    cout << "taskManager::taskManager after TCPsocket"  << endl;

    tcpSocketev = new TCPSocket([](int errorCode, std::string errorMessage)
                        { cout << "Socket creation error:" << errorCode << " : " << errorMessage << endl; });
    cout << "taskManager::taskManager after TCPsocketev"  << endl;

    //async message handler
    tcpSocket->onMessageReceived = [&](string message) {
        
        // parse json message
        message = message.substr(6);
        cout << "Message from the Server: " << message << endl;
        
        // bool parsingSuccessful = Json::parseFromStream(reader, jsonStream, &rMessage, &err);

        Json::Value rMessage;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse( message, rMessage );
        // cout << rMessage << endl;
        if ( !parsingSuccessful )
        {
            cout << "Error parsing the string" << reader.getFormattedErrorMessages()<< endl;
            // std::cerr << "Parsing failed with error: " << err << std::endl;
        }
        if      (rMessage["command"]=="jobsequence")            onMessageJobSequence(rMessage);
        else if (rMessage["command"]=="reqStart")               onMessageRequestStart(tcpSocket);  
        
        else if (rMessage["command"]=="reqSkillClear")          onMessageRequestSkillClear(tcpSocket);
        else if (rMessage["command"]=="reqRosCmd")              onMessageRequestRosCmd(rMessage);                
        else if (rMessage["command"]=="reqEm")                  onMessageRequestEmergency(tcpSocket); 
        else if (rMessage["command"]=="reqStopWait")            onMessageRequestStopWait(rMessage); 

        else if (rMessage["command"]=="resPrepareLoad")         onMessageResponsePrepareLoad(tcpSocket);
        else if (rMessage["command"]=="resSwitchFloor")         onMessageResponseSwitchFloor(tcpSocket);

        else if (rMessage["command"]=="reqloadmap")             onMessageResponseLoadMap(rMessage);
        else if (rMessage["command"]=="reqnodeStatus")          onMessageRequestNodeStatus(rMessage); 
        else if (rMessage["command"]=="reqRebootProcess")       onMessageRequestReboot(rMessage); 
        else if (rMessage["command"]=="manualNextSkill")        onMessageManualNextSkill(rMessage); 

        else {
            cout << "unknown command" << message << endl;
        }
    };

    // On socket closed:
    tcpSocket->onSocketClosed = [this](int errorCode)
    {
        send_status_timer.stop();
        cout << "Connection closed: " << errorCode << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "serverdisconnect";
        mapPublishers["speaker_pub"]->publish(speakermp3); 
        system("rosnode kill task_manager_llm_node");
        sleep(0.5);
        system("rosrun task_manager_llm task_manager_llm_node");
    };

    // initialize ros network handlers
    initializeSubscribers();
    initializeTimer();
    initializePublishers();
    initializeServiceClients();
    // NucClientservice();
    // set a map
    Json::Value rootMap;
    Json::Value cmdMap;
    cmdMap["cmd"] = "load";
    cmdMap["name"] = "mapname";//setMap.c_str(); <<---- need to be determined
    cmdMap["keycmd"] = "non";
    cmdMap["table"] = "non";
    rootMap["cmdMapping"] = cmdMap;
    rootMap["mode"] = "mapping";
    std::string jsonMapMsg = rootMap.toStyledString();
    std_msgs::String rosMapMsg;
    rosMapMsg.data = jsonMapMsg;
    mapPublishers["cmdGUI"]->publish(rosMapMsg);

    // set a velocity
    int vel = 50;//std::stoi(setSpeed);  <<---- need to be determined
    Json::Value rootParam;
    Json::Value cmdParam;
    cmdParam["vel"] = vel;
    rootParam["cmdParam"] = cmdParam;
    rootParam["mode"] = "admin";
    std::string jsonParamMsg = rootParam.toStyledString();
    std_msgs::String rosParamMsg;
    rosParamMsg.data = jsonParamMsg;
    mapPublishers["cmdGUI"]->publish(rosParamMsg);

    std_msgs::String speakermp3;
    speakermp3.data = "taskmanager_operate";
    mapPublishers["speaker_pub"]->publish(speakermp3); 
    //reset ems
    std::stringstream ssGo;
    std_msgs::String rosGoMsg;
    ssGo << "go";
    rosGoMsg.data = ssGo.str();
    mapPublishers["conaGo_pub"]->publish(rosGoMsg);  
    cout << "Requested Loading" << endl; 


    std_msgs::Float64 dockdistance;
    dockdistance.data = 1000; //mm단위
    cout << "Publish :" << dockdistance.data << endl;
    mapPublishers["dockdt_pub"] -> publish(dockdistance);

}

void taskManager::initializeTimer() // timer변수를 taskmanager 멤버변수로 넣어야함.
{
    send_status_timer = nh_.createTimer(ros::Duration(1.0), &taskManager::sendStatusToServerCallback, this);
    serverconnect_timer = nh_.createTimer(ros::Duration(1.0), &taskManager::ServerConnectCallback, this);
    skill_force_end_timer = nh_.createTimer(ros::Duration(5.0), &taskManager::SkillEndCallback, this);
    skill_force_end_timer.stop(); // 즉시 정지
}

void taskManager::initializeSubscribers()
{
    navi_status_sub = nh_.subscribe("/navistatus", 5, &taskManager::naviStatusCallback, this);
    postbox_sub = nh_.subscribe("/service_vision/camera_left/camera_left/plane_estimation/postbox", 1, &taskManager::postboxCallback, this);
    left_pose_sub = nh_.subscribe("/service_vision/camera_left/camera_left/plane_estimation", 1, &taskManager::leftPoseCallback, this);
    right_pose_sub = nh_.subscribe("/service_vision/camera_right/camera_right/plane_estimation", 1, &taskManager::rightPoseCallback, this);
    arm_status_sub = nh_.subscribe("/arm_status", 10, &taskManager::armStatusCallback, this);
    deliver_check_sub = nh_.subscribe("/service_vision/delivery_result", 1, &taskManager::deliverCheckCallback, this);
    abs_pose_sub = nh_.subscribe("/cona/absol_pose", 1, &taskManager::absolPoseCallback, this);
    manual_command_sub = nh_.subscribe("/manualCmd", 1, &taskManager::manualCommandCallback, this);
    arm_info_sub = nh_.subscribe("/arm_info", 1, &taskManager::armInfoCallback, this);
    dock_check_sub = nh_.subscribe("/e_dock/check_result", 1 ,&taskManager::edockresultCallback, this);
    camera_status_sub = nh_.subscribe("/service_vision/camera_status", 1, &taskManager::cameraStatusCallback, this);
    deliver_status_sub = nh_.subscribe("/deliver_check", 1 ,&taskManager::deliveryStatusCallback, this);
}
void taskManager::initializeServiceClients(){
    vision_client = nh_.serviceClient<task_manager_llm::taskmanager_srv>("/vision_service");
    cona_client = nh_.serviceClient<task_manager_llm::taskmanager_srv>("/cona_service");
    nuc_client = nh_.serviceClient<task_manager_llm::taskmanager_srv>("/nuc_service");
}

void taskManager::NucClientservice(string nodename){
    std::string nucmessage;
    if (!nuc_client.waitForExistence(ros::Duration(5.0))) {
        cout << "상태 확인 서비스를 찾을 수 없습니다." << endl;
    }
    task_manager_llm::taskmanager_srv srv;
    srv.request.request = nodename;

    if (nuc_client.call(srv)) {
        nucmessage = srv.response.response;
        ROS_INFO("%s", nucmessage.c_str());
    } else {
        nucmessage = "NUC service not respond";
        ROS_ERROR("%s", nucmessage.c_str());
    }
    
    currentStatus.setNodeStatus(nucmessage);
    Json::Value sendbuffer;
    currentStatus.writeRobotNodeStatusdata(sendbuffer);
    
    Json::Value jsonData;
    jsonData["type"] = "NODESTATUS";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    tcpSocket->SendJson(jsonstring);
}

void taskManager::ConaClientservice(std::string nodename){
    std::string conamessage;
    if (!cona_client.waitForExistence(ros::Duration(5.0))) {
        cout << "상태 확인 서비스를 찾을 수 없습니다." << endl;
    }
    task_manager_llm::taskmanager_srv srv;
    if (nodename == "driving"){
        srv.request.request = "checkError-Odom";
    }
    else if (nodename == "reboot"){
        srv.request.request = "system-reboot";
    }
    else srv.request.request = "";

    if (cona_client.call(srv)) {
        conamessage = srv.response.response;
        ROS_INFO("%s", conamessage.c_str());
    } else {
        conamessage = "CONA service not respond";
        ROS_ERROR("%s", conamessage.c_str());
    }
    
    currentStatus.setNodeStatus(conamessage);
    Json::Value sendbuffer;
    currentStatus.writeRobotNodeStatusdata(sendbuffer);
    
    Json::Value jsonData;
    jsonData["type"] = "NODESTATUS";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    tcpSocket->SendJson(jsonstring);
}

void taskManager::initializePublishers()
{
    cmdGUI = nh_.advertise<std_msgs::String>("cmdGUI", 10, true);
    conaGo_pub = nh_.advertise<std_msgs::String>("/cona/cmd", 5, true);
    pub_posebox = nh_.advertise<std_msgs::Bool>("/service_vision/camera_left/depth/request/postbox", 5, true);
    pub_left = nh_.advertise<std_msgs::Bool>("/service_vision/camera_left/depth/request", 5, true);
    // pub_right = nh_.advertise<std_msgs::Bool>("/service_vision/camera_right/depth/request", 1, true);
    arm_pub = nh_.advertise<std_msgs::Float32MultiArray>("/arm/cmd", 1, true);
    deliverCheck_pub = nh_.advertise<std_msgs::String>("/service_vision/delivery_check", 1, true);
    emergency_pub = nh_.advertise<std_msgs::Bool>("/stop/request", 5, true); 
    // ev_load_pub = nh_.advertise<std_msgs::String>("/input_Mode", 5, true);
    map_loader = nh_.advertise<std_msgs::String>("/cona/cmd", 5, true);
    dockdt_pub = nh_.advertise<std_msgs::Float64>("/e_dock/check", 5, true);
    speaker_pub = nh_.advertise<std_msgs::String>("/sound/cmd", 5, true);

    mapPublishers["cmdGUI"] = &cmdGUI;
    mapPublishers["conaGo_pub"] = &conaGo_pub;
    mapPublishers["pub_posebox"] = &pub_posebox;
    mapPublishers["pub_left"] = &pub_left;
    // mapPublishers["pub_right"] = &pub_right;
    mapPublishers["arm_pub"] = &arm_pub;
    mapPublishers["deliverCheck_pub"] = &deliverCheck_pub;
    mapPublishers["emergency_pub"] = &emergency_pub;
    // mapPublishers["ev_load_pub"] = &ev_load_pub; 
    mapPublishers["map_loader"] = &map_loader; 
    mapPublishers["dockdt_pub"] = &dockdt_pub; 
    mapPublishers["speaker_pub"] = &speaker_pub;
}

void taskManager::edockresultCallback(const std_msgs::Float64::ConstPtr& msg)
{
    
    if(emergencyFlag == true){
        return;
    }
    evobs = msg->data;
    Json::Value jsonData;

    if((naviStatus).find("stop-ems") != string::npos){
        jsonData["data"] = evobs;
        jsonData["status"] = naviStatus;
        if(callSkillCallback(jsonData) == -1){ 
            return;
        }
        currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
    }
}

void taskManager::absolPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    
    currentStatus.setPosition(msg->pose.position.x,msg->pose.position.y);

    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentStatus.setOrientation(yaw);
}



void taskManager::deliverCheckCallback(const std_msgs::String::ConstPtr& msg)
{

    cout << "taskManager::deliverCheckCallback" << endl;
    if(emergencyFlag == true){
        return;
    }

    Json::Reader reader;
    Json::Value s;
    reader.parse(msg->data, s);

    string result = msg->data;

    cout << "Deliver Check : " << result << endl;


    // send result to server
    currentStatus.setdeliveryCheck(result);



    // reset check result
    currentStatus.setdeliveryCheck("None");
}

void taskManager::cameraStatusCallback(const std_msgs::String::ConstPtr& msg){
    string status = msg->data;
    if (status == "11"){
        std::string current_time_str = std::to_string(ros::Time::now().toSec());
        current_vision_time = current_time_str.substr(8,16);
        currentStatus.setVisionTime(current_vision_time);
    }
    currentStatus.setCamStatus(status);
}

void taskManager::deliveryStatusCallback(const std_msgs::Bool::ConstPtr& msg){
    bool delstatus = msg->data;
    std_msgs::String speakermp3;
    if (delstatus == true) speakermp3.data = "delivery_success";
    else if (delstatus == false) speakermp3.data = "delivery_fail";
    mapPublishers["speaker_pub"]->publish(speakermp3); 
}

void taskManager::armStatusCallback(const std_msgs::Int32::ConstPtr& msg)
{
    // cout << "taskManager::armStatusCallback" << endl;
    if(emergencyFlag == true){
        return;
    }

    int armStatus = msg->data;
    
    // nucStatus = processStatus(nucStatus);

    currentStatus.setArmStatus(armStatus);  // 0 or 1 or 2 (0 when manipulating, 1 when rotating)

	Json::Value jsonData;
    jsonData["type"] = "ignore";

    

    int* arm1axisArr = currentStatus.getarm1axis(); 
    int jointArr[3];
    for(int i=0; i<3; i++){
        jointArr[i] = arm1axisArr[i];
    }

    // (When mani starts) set tray number using for the deliver in this skill 
    if((msg->data==1 && recentArmStatus==2) && vectorSkill.size()!=0) // 우선 manipulate에 회전하는거까지 들어 있으므로 1로 하기
    {   
        if(vectorSkill[currentSkill]->getSkillName() == "Manipulate") {
            currentStatus.setTrayUsing(((skill_Manipulate*)vectorSkill[currentSkill])->trayID);
        }
    }
	
    // (When mani goes finish)  data is 2 when arm starts to reset
	if (msg->data==2 && recentArmStatus==1)
    {
        maniCheck = true;
        cout << "maniCheck : " << maniCheck << endl;
    }

     
    recentArmStatus = msg->data;
    
    // Finished manipulation
    if (maniCheck)
    {
	    cout << "Manipulate Done." << endl;

        // sleep(5); // manipulate 이후 arm reset 하는 동안 5초간 대기

        maniCheck = false;

        if (vectorSkill[currentSkill]->getSkillName() == "RESET_ARM_POSE"){
	        jsonData["type"] = "ResetArm";
        }
        else jsonData["type"] = "manipulate";
	    
	    jsonData["status"] = "done";

        Json::StreamWriterBuilder writer;
        std::string jsonstring = Json::writeString(writer, jsonData);

        if(callSkillCallback(jsonData) == -1){
            return;
        }
        currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);     
        currentStatus.setTrayUsing(-1); // reset number of tray using.
	}
    
    std::string current_time_str = std::to_string(ros::Time::now().toSec());
    current_nuc_time = current_time_str.substr(5,7);
    currentStatus.setNucTime(current_nuc_time);
    static ros::Time last_check_time = ros::Time::now();
    static string previous_cona_time = current_cona_time;
    ros::Duration duration = ros::Time::now() - last_check_time;
    if (duration.toSec() >= 1.2) {
        // cout << "check" << previous_cona_time << " and " << current_cona_time << endl;
        if (current_cona_time == previous_cona_time){
            ROS_WARN("cona PC was died!!!!");
        }
        last_check_time = ros::Time::now();
        previous_cona_time = current_cona_time;
    }
    // ROS_INFO("nuc : %s", current_nuc_time.c_str());
    
}

void taskManager::postboxCallback(const std_msgs::Int32::ConstPtr& msg)
{
    cout << "postboxCallback() callback" << endl;

    if(emergencyFlag == true){
        return;
    }

    Json::Value jsonData;
    jsonData["type"] = "detect";
    jsonData["direction"] = "left";
    if (msg->data == 0) jsonData["result"] = "false";
    else if (msg->data == 1) jsonData["result"] = "true";

    if(callSkillCallback(jsonData) == -1){
        return;
    }
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
}

void taskManager::leftPoseCallback(const task_manager_llm::PlaneEstimation::ConstPtr& msg)
{
    cout << "leftPoseCallback() callback" << endl;

    if(emergencyFlag == true){
        return;
    }

    if(leftcamCheck == true){
        std::string visionmessage;
        if(msg->pose.pose.orientation.w == -1) visionmessage = "VISION : Left camera Not Operation!";
        else visionmessage = "VISION : Left camera Normal Operation!";
        
        currentStatus.setNodeStatus(visionmessage);
        Json::Value sendbuffer;
        currentStatus.writeRobotNodeStatusdata(sendbuffer);
        Json::Value jsonData;
        jsonData["type"] = "NODESTATUS";
        jsonData["content"] = sendbuffer;
        Json::StreamWriterBuilder writer;
        std::string jsonstring = Json::writeString(writer, jsonData);
        tcpSocket->SendJson(jsonstring);

        leftcamCheck = false;
        
        std_msgs::Bool msg;
        msg.data = false;   
        mapPublishers["pub_right"]->publish(msg);
    }

    Json::Value jsonData;
    jsonData["type"] = "detect";
    jsonData["direction"] = "left";
    
    Json::Value pos;
    pos.append(msg->pose.pose.position.x);
    pos.append(msg->pose.pose.position.y);
    pos.append(msg->pose.pose.position.z);
    jsonData["position"] = pos;

    Json::Value ori;
    ori.append(msg->pose.pose.orientation.w);
    ori.append(msg->pose.pose.orientation.x);
    ori.append(msg->pose.pose.orientation.y);
    ori.append(msg->pose.pose.orientation.z);

    jsonData["orientation"] = ori;

    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);
    
    if(callSkillCallback(jsonData) == -1){
        return;
    }
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
}


void taskManager::rightPoseCallback(const task_manager_llm::PlaneEstimation::ConstPtr& msg)
{

    cout << "rightPoseCallback() callback" << endl;
    if(emergencyFlag == true){
        return;
    }

    if(rightcamCheck == true){
        std::string visionmessage;
        if(msg->pose.pose.orientation.w == -1) visionmessage = "VISION : Right camera Not Operation!";
        else visionmessage = "VISION : Right camera Normal Operation!";
        
        currentStatus.setNodeStatus(visionmessage);
        Json::Value sendbuffer;
        currentStatus.writeRobotNodeStatusdata(sendbuffer);
        Json::Value jsonData;
        jsonData["type"] = "NODESTATUS";
        jsonData["content"] = sendbuffer;
        Json::StreamWriterBuilder writer;
        std::string jsonstring = Json::writeString(writer, jsonData);
        tcpSocket->SendJson(jsonstring);

        rightcamCheck = false;
    }

   geometry_msgs::PoseWithCovariance msgr;

    
    Json::Value jsonData;
    jsonData["type"] = "detect";
    jsonData["direction"] = "right";
    
    Json::Value pos;
    pos.append(msg->pose.pose.position.x);
    pos.append(msg->pose.pose.position.y);
    pos.append(msg->pose.pose.position.z);
    jsonData["position"] = pos;

    Json::Value ori;
    ori.append(msg->pose.pose.orientation.w);
    ori.append(msg->pose.pose.orientation.x);
    ori.append(msg->pose.pose.orientation.y);
    ori.append(msg->pose.pose.orientation.z);

    jsonData["orientation"] = ori;

    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);
    
    if(callSkillCallback(jsonData) == -1){
        return;
    }
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
}

void taskManager::armInfoCallback(const task_manager_llm::robot_data::ConstPtr& msg)
{
    if(emergencyFlag == true){
        return;
    }
    if (msg->delivery_check=="success" && recentdeliveryCheck=="prepare")
    {
        std_msgs::String speakermp3;
        speakermp3.data = "detect_success";
        mapPublishers["speaker_pub"]->publish(speakermp3); 
    }
    else if (msg->delivery_check=="fail" && recentdeliveryCheck=="prepare")
    {
        std_msgs::String speakermp3;
        speakermp3.data = "detect_fail";
        mapPublishers["speaker_pub"]->publish(speakermp3); 
    }

    recentdeliveryCheck = msg->delivery_check;
    currentStatus.setArmInfo(msg);    
}

int taskManager::clearSkills()
{    
    vectorSkill.clear(); 
    taskListDelivery.clear(); 
    unique_trayID.clear();
    skill_force_end_timer.stop();
    Json::Value noneSkill;
    currentStatus.setWholeSequence(noneSkill);

    currentSkill = -1;   

    emergencyFlag = false;
}

int taskManager::callSkillCallback(Json::Value s)
{
    if (vectorSkill.size()==0) {
        // cout << "no Job assigned... waiting.." << endl; // 나중에 풀기
        return -1;
    }
    if (currentSkill>=vectorSkill.size()) { // usually something wrong, clear the vectorSkill and end 
        clearSkills();
    }

    // call skill callback
    int skillStatus = vectorSkill[currentSkill]->callback_skill(s); // step forward to next subtask
    if(skillStatus==0 && vectorSkill.size()==currentSkill+1){ // when last task finished, size of vectorSkill goes 0
        clearSkills();  

        cout << "Task Done" << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "home_arrived";
        mapPublishers["speaker_pub"]->publish(speakermp3);         
              
        currentStatus.setSkillInfo("", currentSkill); // 목록창 지울 때는 -1 전송
        currentStatus.setNotice("Task Done");    
        if (netstatus) // when network is alive,
        {
            sendSkillSeqToServer();
        }
                       
        return -1;
    }
    else if (skillStatus==0) { // invoke next task
        currentSkill++;
        vectorSkill[currentSkill]->invoke_skill();
        cout << "626 : " << vectorSkill[currentSkill]->getSkillName() << " " << currentSkill << endl;
        currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill); 
        // if (vectorSkill[currentSkill]->getSkillName() == "Manipulate") ManualEndSkill();
    }    
    return 0;
}

void taskManager::SkillEndCallback(const ros::TimerEvent&)
{    
    Json::Value jsonData;
    if (vectorSkill[currentSkill]->getSkillName() == "Manipulate") {
        jsonData["type"] = "manipulate";    
	    jsonData["status"] = "done";
        if(callSkillCallback(jsonData) == -1){
            return;
        }
        currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
        
    }
    

    cout << "649 : " << currentSkill << endl;  
}


void taskManager::manualCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    cout << "taskManager::manualCommandCallback" << endl;
    cout << *msg << endl;

    Json::Reader reader;
    Json::Value s;
    reader.parse(msg->data, s);
    cout << "msg->data = " << msg->data << endl;
    cout << s << endl;
    string cmd = s["command"].asString();
    if (cmd=="jobsequence") { // job sequence
        cout << "taskManager::manualCommandCallback(const std_msgs::String::ConstPtr& msg) :: jobsequence" << endl;
        onMessageJobSequence(s);
    } else if (cmd == "reqStart") {
        cout << "taskManager::manualCommandCallback(const std_msgs::String::ConstPtr& msg) :: reqStart" << endl;
        onMessageRequestStart(tcpSocket);;
    } else {
        cout << "do not understand the manual command" << endl;
    }

}

void taskManager::naviStatusCallback(const std_msgs::String::ConstPtr& msg)
{    
    if(emergencyFlag == true){
        return;
    }

    std::string current_time_str = std::to_string(ros::Time::now().toSec());

    current_cona_time = current_time_str.substr(5,7);
    currentStatus.setConaTime(current_cona_time);
    // ROS_INFO("cona : %s", current_cona_time.c_str());

    Json::Reader reader;
    Json::Value s;
    string goal;
    reader.parse(msg->data, s);

    // string naviReady2Work  = s["ready2work"].asString();
    string naviState       = s["type"].asString();
    naviStatus      = s["status"].asString();   
    string naviTo = s["to"].asString();

    currentStatus.updateNaviStatus(s);

    // find corresponding place in locationCorres
    auto iter = locationCorres.begin();
    while (iter != locationCorres.end()) {
        if((iter->second).compare(naviTo) == 0){
            int floor = (currentStatus.getfloorID());
            int goalfloor = (iter->first)[1] - '0'; // goal의 위치를 알기 위해 층수 매칭
            if(goalfloor == floor){
                goal = iter->first;           
                break;
            }
        }
        ++iter;
    }
    // Clientservice();
    currentStatus.setNaviGoal(goal);
    
    if((naviState == "goto") && (naviStatus == "stop-obs")){  // 장애물에 막혔을때 나는 소리
        obscheck++;
        if(obscheck >= 4){
            std_msgs::String speakermp3;
            speakermp3.data = "Obstacle";
            mapPublishers["speaker_pub"]->publish(speakermp3); 
            sleep(4);
            obscheck = 0;
        }
    }
    else obscheck = 0;

    // when arrived 
    if((naviState).find("arrived") != string::npos){   // when arrived
        sleep(2);
        cout << "arrived !!!" << endl;

        if(callSkillCallback(s) == -1){ // call next subtask [ex.detect]
            return;
        }
        currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);

    }  

}

void taskManager::sendStatusToServerCallback(const ros::TimerEvent&)
{
    if (netstatus) // when network is alive,
        sendStatusToServer();  
}

void taskManager::ServerConnectCallback(const ros::TimerEvent&)
{
    int ret = 0;
    tcpSocket->Connect(
        serverAddress, serverPort, [&]
        {
            cout << "connect 443" << endl;
            std_msgs::String speakermp3;
            speakermp3.data = "serverconnect";
            mapPublishers["speaker_pub"]->publish(speakermp3); 
            onSendConnect(tcpSocket);
            ret = 0;
            netstatus = 1;
            serverconnect_timer.stop();
            send_status_timer.start();
        },
        [&](int errorCode, std::string errorMessage)
        {
            // CONNECTION FAILED
            cout << errorCode << " : " << errorMessage << endl;
            ret = 1;

            netstatus = 0;
        });    
    usleep(100000); //try 0.1s
    // return ret;
    // connectToServer(serverAddress, serverPort);
}

int taskManager::sendStatusToServer()
{
    // cout << "sendStatusToServer()" << endl;
    Json::Value sendbuffer;
    currentStatus.writeRobotStatus(sendbuffer);
    
    Json::Value jsonData;
    jsonData["type"] = "STATUS";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);
    tcpSocket->SendJson(jsonstring);
}

int taskManager::sendSkillSeqToServer()
{
    Json::Value sendbuffer;
    currentStatus.writeSkillSequence(sendbuffer);
    
    Json::Value jsonData;
    jsonData["type"] = "SKILLSEQ";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    
    std::string jsonstring = Json::writeString(writer, jsonData);
    tcpSocket->SendJson(jsonstring);
    sleep(1);
}

int taskManager::onMessageJobSequence(Json::Value &rMessage)
{
    cout << endl << "jobsequence received.. calling path planner" << endl;
    cout << "Setting task sequence" << endl;

    std:string locFromjs;
    Json::Value infojs; 

    std::string sourceFloor, targetFloor;

    // Task 추가
    for (int ii=0; ii<rMessage["js"].size(); ii++) {
        infojs = rMessage["js"][ii];
        if (infojs["skill"].asString() == "MoveTo"){
            locFromjs = infojs["location"].asString();
            // 찾은 value값을 원래 rMessage의 skill에 추가하기 // ex. locationStr : Place 01
            infojs["locationStr"] = locationCorres.at(locFromjs);
        }
        else if (infojs["skill"].asString() == "PrepareLoad"){
            cout << rMessage["js"][ii]["floor"] << endl;
            infojs["sourceFloor"]  = rMessage["js"][ii]["floor"].asInt();
            infojs["direction"] = rMessage["js"][ii]["direction"].asString();/////////////////////////////
        }
        else if (infojs["skill"].asString() == "DecideLoad"){
            
        }
        else if (infojs["skill"].asString() == "SwitchFloor"){
            targetFloor = to_string(rMessage["js"][ii]["floor"].asInt()); 
            infojs["targetFloor"]  = rMessage["js"][ii]["floor"].asInt(); 
            infojs["switchMap"] = mapNumCor.at(targetFloor);
        }
        else if (infojs["skill"].asString() == "Detect"){
            
        }
        else if (infojs["skill"].asString() == "Manipulate"){
            infojs["tray"]  = to_string(rMessage["js"][ii]["tray"].asInt()); 
        }   

        if(addSkill(infojs) == 1){
            cout << "Duplciated tray requested. Cleared Skills" << endl;
            std_msgs::String speakermp3;
            speakermp3.data = "tasknotAssigned";
            mapPublishers["speaker_pub"]->publish(speakermp3); 
            clearSkills();       
            currentStatus.setSkillInfo("", currentSkill);
            currentStatus.setNotice("Duplicated Tray : Already used");         
            
            return 0;
        }
    }
    cout << endl << endl;

    Json::Value jsonWholeSkill;

    for (int ii=0; ii<vectorSkill.size(); ii++) {
        // Skill 목록 출력
        cout << "Skill: " << vectorSkill[ii]->getSkillName() << endl;
        // 서버로 보내기 위한 job sequence 구성
        jsonWholeSkill.append(vectorSkill[ii]->getSkillName());
    }    
    
    currentStatus.setWholeSequence(jsonWholeSkill);    
    sendSkillSeqToServer(); // 서버로 job sequence 전송
    
    // Skill 실행 준비
    currentSkill = -1;

    cout << endl;

    currentStatus.setSkillInfo("", currentSkill); // 프론트 skill 목록 창 지울 때 -1 보내지만 notice가 task assigned이면 목록창 출력
    currentStatus.setNotice("Task Assigned");
    std_msgs::String speakermp3;
    speakermp3.data = "TaskAssigned";
    mapPublishers["speaker_pub"]->publish(speakermp3); 
    return 0;
}

int taskManager::onMessageRequestStart(TCPSocket* tcpSocket)
{   
    cout << "startRequest received.. " << endl;
    if (vectorSkill.size()==0) {
        cout << "   no job assigned.. waiting.." << endl;
        return -1;
    }
    std_msgs::String speakermp3;
    speakermp3.data = "TaskStart";
    mapPublishers["speaker_pub"]->publish(speakermp3); 
    currentSkill = 0; // task 시작
    vectorSkill[currentSkill]->invoke_skill();

    cout << "currentSkill = " << vectorSkill[currentSkill]->getSkillName() << endl;
    cout << "currentSkillID = " << currentSkill << endl;
 
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
    currentStatus.setNotice("Task Start");

    // if (vectorSkill[currentSkill]->getSkillName() == "Manipulate") ManualEndSkill(); // manipulate 는 5초뒤에 종료

    return 0;
}

int taskManager::onMessageRequestSkillClear(TCPSocket* tcpSocket)
{
    cout << "Request for clearing all skills received.. " << endl;
    if(vectorSkill.size() > 0){
        cout << vectorSkill.size() << endl;
        clearSkills();
        
        initializeLoadmap(); 
        currentStatus.setSkillInfo("", currentSkill); 
        currentStatus.setNotice("Task not assigned");
        cout << vectorSkill.size() << endl;
        currentStatus.setHoldStatus("off");
        if (netstatus) // when network is alive,
            sendSkillSeqToServer();
    }

    return 0;
}

int taskManager::onMessageRequestRosCmd(Json::Value &rMessage)
{
    cout << "Command received.. " << rMessage["js"] << endl;
    // publsih
    string rosCmd = rMessage["js"].asString();
    system(rosCmd.c_str());

    return 0;
}

int taskManager::onMessageRequestEmergency(TCPSocket* tcpSocket)
{
    cout << "Emergency Request received.. " << endl;
    // if (vectorSkill.size()==0) {
    //     cout << "   no job assigned.." << endl;
    //     return -1;
    // }
    cout << "Emergency : Stop Whole Skill.. " << endl;

    // ROS publish
    std_msgs::Bool emMsg;
    emMsg.data = true;

    mapPublishers["emergency_pub"] -> publish(emMsg);

    // 모든 skill 삭제
    clearSkills();    
    emergencyFlag = true;   

    currentStatus.setSkillInfo("", currentSkill);
    currentStatus.setNotice("EMERGENCY");

    return 0;
}

int taskManager::onMessageResponsePrepareLoad(TCPSocket* tcpSocket)
{
    Json::Value jsonData;
    jsonData["type"] = "completePrepareLoad";

    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    if(callSkillCallback(jsonData) == -1){
        return 0;
    }
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);

}


int taskManager::onMessageResponseSwitchFloor(TCPSocket* tcpSocket)
{
    currentStatus.setfloorID(stoi(((skill_SwitchFloor*)(vectorSkill[currentSkill]))->targetFloor));

    Json::Value jsonData;
    jsonData["type"] = "completeSwitchFloor";

    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);

    if(callSkillCallback(jsonData) == -1){
        return 0;
    }
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
}




int taskManager::onMessageRequestStopWait(Json::Value &rMessage)
{
    cout << "Stop Wait Request received.. " << endl;

    Json::Value jsonData;
    Json::StreamWriterBuilder writer;
    std::string jsonstring;

    if (vectorSkill.size()==0) {
        cout << "   no job assigned.." << endl;
        return -1;
    }
    else if(vectorSkill[currentSkill]->getSkillName() == "MoveTo"){ 
        std_msgs::String rosGoMsg;
        rosGoMsg.data = rMessage["sg"].asString();     // stop, go가 들어가있는 key
        mapPublishers["conaGo_pub"]->publish(rosGoMsg);  // "/cona/cmd" 토픽으로 skill 보내줌(모든 skill동일)
        if (rMessage["sg"].asString() == "stop"){
            cout << "Requested HOLD ON" << endl;
            std_msgs::String speakermp3;
            speakermp3.data = "hold_on";
            mapPublishers["speaker_pub"]->publish(speakermp3); 
            currentStatus.setHoldStatus("on");
        }
        else if (rMessage["sg"].asString() == "go"){
            cout << "Requested HOLD OFF" << endl;
            std_msgs::String speakermp3;
            speakermp3.data = "hold_off";
            mapPublishers["speaker_pub"]->publish(speakermp3); 
            currentStatus.setHoldStatus("off");
        }
    }
    else{
        cout << "   Current subtask is not WAIT.." << endl;
        return -1;
    }
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);

    return 0;
}

int taskManager::onMessageResponseLoadMap(Json::Value &rMessage)
{
    std_msgs::String map;
    std::stringstream mapLoad;

    std::string floor = rMessage["floor"].asString();
    std::string whichMap = mapNumCor.at(floor);

    mapLoad << whichMap;
    map.data = mapLoad.str();
    mapPublishers["map_loader"]->publish(map);

    currentStatus.setfloorID(stoi(floor));
}

int taskManager::onMessageRequestNodeStatus(Json::Value &rMessage)
{
    std::string nodename = rMessage["js"].asString();
    std::string nodestatus;

    if (nodename == "driving") {
        ConaClientservice(nodename);
        NucClientservice(nodename);
    }
    // else if (nodename == "nucdriving") NucWheelClientservice();
    else if (nodename == "manipulate") NucClientservice(nodename);
    else if (nodename == "vision") {
        leftcamCheck = true;
        rightcamCheck = true;
        std_msgs::Bool msg;
        msg.data = false;   
        mapPublishers["pub_left"]->publish(msg);
    }
    else {
        nodestatus = "Node name is incorrect";
        currentStatus.setNodeStatus(nodestatus);
        Json::Value sendbuffer;
        currentStatus.writeRobotNodeStatusdata(sendbuffer);

        Json::Value jsonData;
        jsonData["type"] = "NODESTATUS";
        jsonData["content"] = sendbuffer;
        Json::StreamWriterBuilder writer;
        std::string jsonstring = Json::writeString(writer, jsonData);

        tcpSocket->SendJson(jsonstring);
    }
}

int taskManager::onMessageRequestReboot(Json::Value &rMessage)
{
    std::string nodename = rMessage["js"].asString();
    std::string nodestatus;

    // std::string service = ;
    if (nodename == "cona") {
        NucClientservice("reboot");
        ConaClientservice("reboot");
        system("reboot");
    }
    else if (nodename == "nuc") NucClientservice("reboot");
    else if (nodename == "jetson") system("reboot");
    else {
        nodestatus = "Node name is incorrect";
        currentStatus.setNodeStatus(nodestatus);
        Json::Value sendbuffer;
        currentStatus.writeRobotNodeStatusdata(sendbuffer);

        Json::Value jsonData;
        jsonData["type"] = "NODESTATUS";
        jsonData["content"] = sendbuffer;
        Json::StreamWriterBuilder writer;
        std::string jsonstring = Json::writeString(writer, jsonData);

        tcpSocket->SendJson(jsonstring);
    }
}

int taskManager::onMessageManualNextSkill(Json::Value &rMessage)
{
    vectorSkill[currentSkill]->invoke_skill();
    currentSkill++;
    currentStatus.setSkillInfo(vectorSkill[currentSkill]->getSkillName(), currentSkill);
}

int taskManager::loadConfig(string filename) {
  Json::Value root;
  cout << "filename =" << filename << endl;
  std::ifstream cfgfile(filename);
  cfgfile >> root;
  cout << "filename =" << filename << endl;

  std::string name = root["robotName"].asString();
  std::string mapName = root["MapName"].asString();
  serverAddress = root["server"].asString();
  serverPort = root["port"].asUInt();
  std::string robotID = root["robotID"].asString();
  Json::Value mapCor = root["mapLoad"];
  Json::Value locCor = root["locationCorresponence"];
  // place가 갈수있는 tray 정보
  Json::Value tray = root["Tray_Number"];
  
  std::string homeLoc = root["home"].asString();
  std::string startLoc = root["start"].asString();

  cout << "int taskManager::loadConfig(string filename)" << endl << "===========================" << endl;
  cout << "RobotName = " << name << endl;
  cout << "serverAddress = " << serverAddress << ", port = " << serverPort << endl;
  cout << "mapName = " << mapName << endl;
  cout << "Home = " << homeLoc << endl;
  cout << "Start = " << startLoc << endl;
//   cout << "Location Correspondence" << endl << locCor << endl;

  setRobotName(name);
  setRobotID(robotID);
  setLoadMap(mapCor);
  setLocationCorres(locCor);
  setHomeLocation(homeLoc);
  setStartLocation(startLoc);
  //tray
  setTrayNumber(tray);
  initializeLoadmap(); 

}
int taskManager::setRobotName(string name)
{
    currentStatus.setRobotName(name);
}

int taskManager::setRobotID(string robotID)
{
    currentStatus.setRobotID(robotID);
}

int taskManager::setLoadMap(Json::Value js)
{
    std::string floorNum, mapNum; 

    for(int i=0; i<js.size(); i++){
        floorNum = js[i].getMemberNames()[0];
        mapNum = js[i][floorNum].asString();

        cout << "Set Floor Num - Map Number correspondence : " << floorNum << " <--> " << mapNum << endl;
        mapNumCor.insert({ floorNum, mapNum });        
    }
}

int taskManager::setLocationCorres(Json::Value js)
{
    std::string locNum, locPlace; 

    for(int i=0; i<js.size(); i++){
        locNum = js[i].getMemberNames()[0];
        locPlace = js[i][locNum].asString();

        cout << "Set location correspondence : " << locNum << " <--> " << locPlace << endl;
        locationCorres.insert({ locNum, locPlace });        
    }
}

//task
int taskManager::setTrayNumber(Json::Value js)
{
    currentStatus.setPlaceTray(js);
    std::string placeNum;
    
    std::map<std::string, std::shared_ptr<std::vector<int>>> PlaceTrayCorres;

    for (const auto& place : js.getMemberNames()) {
        const Json::Value& trayNumJson = js[place];
        std::shared_ptr<std::vector<int>> trayNum = std::make_shared<std::vector<int>>();
        
        for (const auto& num : trayNumJson) {
            trayNum->push_back(num.asInt());
        }
        
        std::cout << "Set location correspondence : " << place << " <--> ";
        for (const auto& num : *trayNum) {
            std::cout << num << " ";
        }
        std::cout << std::endl;

        PlaceTrayCorres.insert(std::make_pair(place, trayNum));
    }   
}

int taskManager::initializeLoadmap()
{   
    std_msgs::String map;
    std::stringstream mapLoad;

    std::string floor = string(1, start.at(1)); // home이 있는 층의 map load
    std::string whichMap = mapNumCor.at(floor);

    mapLoad << whichMap;
    map.data = mapLoad.str();
    mapPublishers["map_loader"]->publish(map);

    currentStatus.setfloorID(stoi(floor));
}

int taskManager::setHomeLocation(string homeloc){
    home = homeloc;
    currentStatus.setHomePlace(homeloc);
}

int taskManager::setStartLocation(string startloc){
    start = startloc;
    currentStatus.setStartPlace(startloc);
}

int taskManager::onSendConnect(TCPSocket* tcpSocket)
{
    cout << "Connected to the server successfully." << endl;

    Json::Value sendbuffer;
    
    currentStatus.writeRobotinitdata(sendbuffer);

    Json::Value jsonData;
    jsonData["type"] = "CONNECT";
    jsonData["content"] = sendbuffer;

    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);
    // cout << jsonstring << endl;
    tcpSocket->SendJson(jsonstring);
    
    return 0;
}

int taskManager::sendInitDataToServer()
{
    Json::Value sendbuffer;
    currentStatus.writeRobotinitdata(sendbuffer);

    Json::Value jsonData;
    jsonData["type"] = "InitialValue";
    jsonData["content"] = sendbuffer;
    Json::StreamWriterBuilder writer;
    std::string jsonstring = Json::writeString(writer, jsonData);
    tcpSocket->SendJson(jsonstring);
    // cout << jsonstring << endl;
}


int taskManager::connectToServer(std::string host, uint16_t port)
{
    int ret = 0;
    tcpSocket->Connect(
        host, port, [&]
        {
            cout << "connect 443" << endl;
            std_msgs::String speakermp3;
            speakermp3.data = "serverconnect";
            mapPublishers["speaker_pub"]->publish(speakermp3); 
            onSendConnect(tcpSocket);
            ret = 0;
            netstatus = 1;
            serverconnect_timer.stop();
            send_status_timer.start();
        },
        [&](int errorCode, std::string errorMessage)
        {
            // CONNECTION FAILED
            cout << errorCode << " : " << errorMessage << endl;
            ret = 1;

            netstatus = 0;
        });    
    usleep(100000); //try 0.1s
    return ret;
}

int taskManager::addSkill(Json::Value nSkill)
{
    string skill = nSkill["skill"].asString();
    auto iter = addSkillFunctions.find(skill);
    if (iter == addSkillFunctions.end())
    {
        // not found
        cout << "addSkill something wrong, skill " << skill << " undefined " << endl;
    }
    else
        // modified : 중복 tray 반환값 == 1
        if(addSkillFunctions[skill](nSkill, this) == 1){
            return 1;
        }
}

int taskManager::addSkill_MoveTo(Json::Value params, taskManager* pt){
    std::string locationID = params["location"].asString();
    std::string locationSTR = params["locationStr"].asString();

    cout << "location = " << locationID << ", Place = "  << locationSTR << endl;

    skill_MoveTo* skill = new skill_MoveTo;
    skill->set_skill(params,pt->mapPublishers);// setting skill parameter
    pt->vectorSkill.push_back(skill);
}

int taskManager::addSkill_Detect(Json::Value params, taskManager* pt){

    cout << "Detect" << endl;

    skill_Detect* skill = new skill_Detect;
    skill->set_skill(params,pt->mapPublishers);

    pt->vectorSkill.push_back(skill);
}

int taskManager::addSkill_Manipulate(Json::Value params, taskManager* pt){
    int trayID = stoi(params["tray"].asString());
    
    pt->taskListDelivery.push_back(trayID);
    auto result = pt->unique_trayID.insert(trayID);
    if (!result.second) { //already trayID is used
        cout << "error in addSkill, trayID " << trayID << " is already used in the task list." << endl;
        return 1;
    }
    else {
        skill_Manipulate* skill = new skill_Manipulate;
        skill->set_skill(params,pt->mapPublishers, pt->skill_force_end_timer);//, &pt->pub_left, &pt->pub_right, &pt->conaGo_pub);
        pt->vectorSkill.push_back(skill);
    }
    
}

int taskManager::addSkill_SetEvEms(Json::Value params, taskManager* pt){ //pt : taskmanager 클래스의 인스턴스에 대한 포인터
    std::string locationID = params["location"].asString();;
    std::string locationSTR = params["locationStr"].asString();

    cout << "location = " << locationID << ", Place = "  << locationSTR << endl;

    skill_SetEvEms* skill = new skill_SetEvEms;
    skill->set_skill(params,pt->mapPublishers);// setting skill parameter
    pt->vectorSkill.push_back(skill);
}


int taskManager::addSkill_PrepareLoad(Json::Value params, taskManager* pt){
    cout << "Source Floor = " << params["sourceFloor"].asInt() << "Direction = " << params["direction"].asString() << endl;
    skill_PrepareLoad* skill = new skill_PrepareLoad;
    skill->set_skill(params,pt->mapPublishers, pt->tcpSocket);
    pt->vectorSkill.push_back(skill);
}

int taskManager::addSkill_DecideLoad(Json::Value params, taskManager* pt){
    
    cout << "Decide Load" << endl;
    skill_DecideLoad* skill = new skill_DecideLoad;
    skill->set_skill(params,pt->mapPublishers, pt->tcpSocket);
    pt->vectorSkill.push_back(skill);
}

int taskManager::addSkill_SwitchFloor(Json::Value params, taskManager* pt){
    
    cout << "Target Floor = "  << params["targetFloor"].asInt() << endl;
    skill_SwitchFloor* skill = new skill_SwitchFloor;
    skill->set_skill(params,pt->mapPublishers, pt->tcpSocket);
    pt->vectorSkill.push_back(skill);
}