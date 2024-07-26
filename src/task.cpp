#include "taskManager/task.h"

#include <chrono>

using namespace std;

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
    cout << "Requested MoveTo" << locationID << endl;
    sleep(1);
}

int skill_MoveTo::callback_skill(Json::Value js){

    // original code
    cout << "here, skill_MoveTo::callback_skill(Json::Value js): status = " << status << endl;


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
    else status = -1;

    return status;
}
// -------------------------------------------------------------------------------------- //
int skill_Detect::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs){
    skillBase::set_skill(js, mPubs);    
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
        status = -1;
    }
    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_Manipulate::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs){
    skillBase::set_skill(js, mPubs);    
    trayID = stoi(js["tray"].asString());
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
        status = -1;
    }
    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_PrepareLoad::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    skillBase::set_skill(js, mPubs);
    sourceFloor = js["sourceFloor"].asString(); // 3
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
    sendbuffer["source"] = sourceFloor;
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
    string msgfrom  = js["type"].asString();
    if (msgfrom=="completePrepareLoad") {
        status = 0;
        cout << "skill_PrepareLoad::callback_skill(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "ev_call_finish";
        mPublishers["speaker_pub"]->publish(speakermp3); 
    }
    else status = -1;

    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_SetEvEms::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs){
    skillBase::set_skill(js, mPubs);    
    locationID = js["location"].asString();
    locationStr = js["locationStr"].asString();
}

int skill_SetEvEms::invoke_skill(){
    if (status==1) {
        cout << "something wrong ini skill_SetEvEms; status==1" << endl; 
        return -1;
    }
    status = 1;
    std_msgs::String rosGoMsg;
    std::stringstream ssGo;
    cout << locationStr << endl;
    ssGo << "goTo " << locationStr;
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
int skill_SetEvEms::callback_skill(Json::Value js){
    // original code

    cout << "here, skill_SetEvEms::callback_skill(Json::Value js): status = " << status << endl;
    // cout << js << endl;
    string naviReady2Work  = js["ready2work"].asString();
    string naviState       = js["type"].asString();
    string naviStatus      = js["status"].asString();
    cout << naviStatus << endl;
    if ((status)&&(naviStatus).find("stop-ems") != string::npos) { 
        status = 0;
        cout << "skill_SetEvEms::callback_skill(Json::Value js) : STOP!"  << endl;
    }
    else status = -1;
    sleep(1);
    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_DecideLoad::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    skillBase::set_skill(js, mPubs);    
    tcpSocket = ts;
}

int skill_DecideLoad::invoke_skill(){
    cout << "skill_DecideLoad::invoke_skill" << endl;
    if (status==1) {
        cout << "something wrong in skill_DecideLoad::invoke_skill; status==1" << endl; 
        return -1;
    }
    status = 1;

    std_msgs::String speakermp3;
    speakermp3.data = "ev_decide";
    mPublishers["speaker_pub"]->publish(speakermp3); 
    std_msgs::Float64 dockdistance;
    dockdistance.data = 3000; //mm단위
    cout << "Publish :" << dockdistance.data << endl;
    mPublishers["dockdt_pub"] -> publish(dockdistance);
    checkEVClose = false;
    sleep(1);
}

int skill_DecideLoad::callback_skill(Json::Value js){

    distance = js["data"].asFloat();
    msgfrom = js["type"].asString();
    msgstatus = js["status"].asString();
    
    if (msgstatus=="stop-ems") {
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
                    cout << "skill_DecideLoad::callback_skill(Json::Value js) : no person"  << endl;
                    distances.clear();
                    std_msgs::String speakermp3;
                    speakermp3.data = "ev_decide_no_obs";
                    mPublishers["speaker_pub"]->publish(speakermp3); 
                    checkperson = false;
                }
                else {
                    cout << "skill_DecideLoad::callback_skill(Json::Value js) : person"  << endl;
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
                cout << "skill_DecideLoad::callback_skill(Json::Value js) : RePrepareLoad"  << endl;
                checkEVClose = false;
                doorcheck1 = false;
                doorcheck2 = false;
            }
        }
    }
    else {
        cout << "skill_DecideLoad::callback_skill::::ignore message" << endl;
        status = -1;
    }
    sleep(1);
    return status;
}

// -------------------------------------------------------------------------------------- //

int skill_SwitchFloor::set_skill(Json::Value js, map<string,ros::Publisher*> mPubs, TCPSocket* ts){
    skillBase::set_skill(js, mPubs);   

    tcpSocket = ts;
    targetFloor = js["floor"].asString();
    whichMap = js["switchMap"].asString();
}

int skill_SwitchFloor::invoke_skill(){
    cout << "skill_SwitchFloor::invoke_skill" << endl;
    if (status==1) {
        cout << "something wrong in skill_SwitchFloor::invoke_skill; status==1" << endl; 
        return -1;
    }
    status = 1;

    // 층간이동전에 map을 바꿈.
    std_msgs::String map;
    std::stringstream mapLoad;
    mapLoad << whichMap;
    map.data = mapLoad.str();
    mPublishers["map_loader"]->publish(map);
    cout << "Requested Map change" << endl;

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

int skill_SwitchFloor::callback_skill(Json::Value js){
    string msgfrom  = js["type"].asString();
    if (msgfrom=="completeSwitchFloor") {
        status = 0;
        cout << "skill_SwitchFloor::callback_skill(Json::Value js) : arrived"  << endl;
        std_msgs::String speakermp3;
        speakermp3.data = "ev_switch_finish" + targetFloor;
        mPublishers["speaker_pub"]->publish(speakermp3);

    }
    else {
        cout << "skill_SwitchFloor::callback_skill::::ignore message" << endl;
        status = -1;
    }
    return status;
}