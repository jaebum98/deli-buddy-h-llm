#include "taskManager/robotStatus.h"

using namespace std;

robotStatus::robotStatus()
{
    // reserve for later use
    // addSubTaskFunctions["WAIT"]=&addSubTask_WAIT;
    // addSubTaskFunctions["DELIVER"]=&addSubTask_DELIVER;
    // addSubTaskFunctions["HOME"]=&addSubTask_HOME;
    // addSubTaskFunctions["CHARGE"]=&addSubTask_CHARGE;
    // addSubTaskFunctions["ESTOP"]=&addSubTask_ESTOP;

    trayStatusBit.set();
}
int robotStatus::setRobotName(string name)
{
    robotName = name;
    return 0;
}

int robotStatus::setPlaceTray(Json::Value number) //tray info
{
    placeTrayJson = number;
}

int robotStatus::setHomePlace(string home) //homeplace info
{
    homeplace = home;
}

int robotStatus::setStartPlace(string start)
{
    startplace = start;
}
// string robotStatus::getRobotName()
// {
//     return robotName;
// }

int robotStatus::setRobotID(string ID)
{
    robotID = ID;
    return 0;
}
int robotStatus::setIP(string ip)
{
    IP_address=ip;
    return 0;
}

int robotStatus::setNucTime(string nuctime)
{
    Nuc_time = nuctime;
}
int robotStatus::setConaTime(string conatime)
{
    Cona_time = conatime;
}
int robotStatus::setVisionTime(string visiontime)
{
    Vision_time = visiontime;
}
int robotStatus::setNodeStatus(string nodestatus)
{
    Node_status = nodestatus;
}

int robotStatus::updateNaviStatus(Json::Value nStatus)
{   

    naviReady2Work  = nStatus["ready2work"].asString();
    naviType       = nStatus["type"].asString();
    naviStatus      = nStatus["status"].asString();  

    return 0;
}

int robotStatus::setNaviGoal(string goal)
{
    naviGoal = goal;
}


int robotStatus::setNotice(string noticeStr)
{
    notice = noticeStr;
}

int robotStatus::setdeliveryCheck(string check)
{
    deliveryCheck = check;
}

int robotStatus::setbattery1Remain(int b1r)
{
    battery1Remain = b1r;
}

int robotStatus::setbattery2Remain(int b2r)
{
    battery2Remain = b2r;
}

int robotStatus::setfloorID(int floorID)
{
    floor = floorID;
}

int robotStatus::getfloorID(){
    return floor;
}

int robotStatus::setmanipModuleStatus(int ms)
{
    manipModuleStatus = ms;
}

int robotStatus::setSkillInfo(string skill, int skillID)
{
	currentSkill = skill;
    currentSkillID = skillID;
}

int robotStatus::setWholeSequence(Json::Value wholeSkills) // job 할당할 때, clear 할 때 호출
{ 
    if(wholeSkills.size() != 0)          wholeSequence = wholeSkills;   
    else                                 wholeSequence.clear();  

    numTotalSkills = wholeSequence.size();
    
    // cout << numTotalSkills << endl;                       
}

int* robotStatus::getarm1axis(){ // return arm1axis array
    return arm1axis;
}

int robotStatus::setjoyStatus(string joystatus)
{
    joy = joystatus;
}

int robotStatus::setArmInfo(const task_manager_llm::robot_data::ConstPtr& msg)
{   

    arm_robot_state = msg->robot_state;
    setdeliveryCheck(msg->delivery_check);
    setjoyStatus(msg->joy_state);
    // arm1axis[0] = msg->forward_joint1*1000;    // 뒤에 소수점을 제거하기 위해 1000을 곱하고 int형 변환
    // arm1axis[1] = msg->forward_joint2*1000;    
    // arm1axis[2] = msg->forward_joint3*1000;    

    // arm2axis[0] = msg->backward_joint1*1000;    
    // arm2axis[1] = msg->backward_joint2*1000;    
    // arm2axis[2] = msg->backward_joint3*1000; 

    // arm_1height = msg-> forward_height*1000;
    // arm_2height = msg-> backward_height*1000;

    arm_tray = msg->tray;

    // arm_front_pos[0] = msg->front_pos_x;
    // arm_front_pos[1] = msg->front_pos_y;
    // arm_front_pos[2] = msg->front_pos_z;    
    // arm_front_pos[3] = msg->front_pos_a;    

    // arm_backward_pos[0] = msg->backward_pos_x;
    // arm_backward_pos[1] = msg->backward_pos_y;
    // arm_backward_pos[2] = msg->backward_pos_z;    
    // arm_backward_pos[3] = msg->backward_pos_a;  

    // arm_tray_status[0] = msg->table_status_1;
    // arm_tray_status[1] = msg->table_status_2;
    // arm_tray_status[2] = msg->table_status_3;
    // arm_tray_status[3] = msg->table_status_4;
    // arm_tray_status[4] = msg->table_status_5;
    // arm_tray_status[5] = msg->table_status_6;
    // arm_tray_status[6] = msg->table_status_7;
    // arm_tray_status[7] = msg->table_status_8;
    // arm_tray_status[8] = msg->table_status_9;
    // arm_tray_status[9] = msg->table_status_10;
    // arm_tray_status[10] = msg->table_status_11;
    // arm_tray_status[11] = msg->table_status_12;
    // arm_tray_status[12] = msg->table_status_13;
    // arm_tray_status[13] = msg->table_status_14;
    // arm_tray_status[14] = msg->table_status_15;
    // arm_tray_status[15] = msg->table_status_16;    
    
    // arm tray status bit 로 만들기    
    trayStatusBit.set(); // 16자리 모두 1로 set
    for(int i=0; i<16; i++){
        if (arm_tray_status[i]==0){
            trayStatusBit.flip(i);
        }
    }

    // cout<<"bit.to_ulong()::"<<(int)trayStatusBit.to_ulong()<<endl;
    
}

int robotStatus::setArmStatus(int armStatus)
{
    arm1Status = armStatus;
    // nucStatus = nucStatus;
    // cout << arm1Status << endl;
}

int robotStatus::setHoldStatus(string holdstatus)
{
    holdStatus = holdstatus;
}

int robotStatus::setCamStatus(string camstatus){
    camStatus = camstatus;
}

int robotStatus::setTrayUsing(int num) // 이건 안쓰는게 나을듯(지금은 delivery 시퀀스가 오로지 nuc에서만 돌아서 보는의미가 없음)
{
    trayUsing = num;
}

int robotStatus::writeRobotStatus(Json::Value &nStatus)
{
    nStatus["lk"] = robotName; // logKey
    nStatus["rID"] = robotID;
    nStatus["px"] = posx;
    nStatus["py"] = posy;
    nStatus["ori"]  = orientation;    

    nStatus["nte"] = naviType; 
    nStatus["nss"] = naviStatus; 
    nStatus["goal"] = naviGoal; // added

    nStatus["cs"] = currentSkill;
    nStatus["csid"] = currentSkillID;

    // notice
    if(currentSkillID >= 0){
        nStatus["nt"] = "Task Start";
    }
    else{
        nStatus["nt"] = notice;
    }
    // cout << nStatus["nt"] << endl;    


    // arm info
    Json::Value arm1axis_json, arm2axis_json, arm_front_pos_json; //, arm_tray_status_json;

    for(int i=0; i<sizeof(arm1axis)/sizeof(arm1axis[0]); i++){
        arm1axis_json.append(arm1axis[i]);
    }
    for(int i=0; i<sizeof(arm2axis)/sizeof(arm2axis[0]); i++){
        arm2axis_json.append(arm2axis[i]);
    }
    for(int i=0; i<sizeof(arm_front_pos)/sizeof(arm_front_pos[0]); i++){
        arm_front_pos_json.append(arm_front_pos[i]);
    }
    // for(int i=0; i<sizeof(arm_tray_status)/sizeof(arm_tray_status[0]); i++){
    //     arm_tray_status_json.append(arm_tray_status[i]);
    // }

    nStatus["ars"] = arm_robot_state;
    nStatus["a1x"] = arm1axis_json;
    nStatus["a2x"] = arm2axis_json;
    nStatus["a1h"] = arm_1height;
    nStatus["a2h"] = arm_2height;
    nStatus["aty"] = arm_tray;
    nStatus["afp"] = arm_front_pos_json;
    nStatus["ats"] = (int)trayStatusBit.to_ulong();
    nStatus["as"] = arm1Status;
    nStatus["tray"] = trayUsing;

    nStatus["ams"] = manipModuleStatus; 
    nStatus["dc"] = deliveryCheck;
    
    nStatus["cam"] = camStatus;
    nStatus["hs"] = holdStatus;

    nStatus["b1"] = battery1Remain;
    nStatus["b2"] = battery2Remain;
    nStatus["fi"] = floor;

    nStatus["joy"] = joy;    
    //프로세스의 상태
    nStatus["np"] = Nuc_time;
    nStatus["cp"] = Cona_time;
    nStatus["vp"] = Vision_time;

    return 0;
}

int robotStatus::writeSkillSequence(Json::Value &skillSeq)
{
    skillSeq["lk"] = robotName;
    skillSeq["ts"] = wholeSequence; 
    skillSeq["nts"] = numTotalSkills;
}

int robotStatus::writeRobotinitdata(Json::Value &InitData)
{
    InitData["lk"] = robotName;  // 기존 getRobotName을 없애고 통합
    InitData["hm"] = homeplace;
    InitData["st"] = startplace;
    InitData["pt"] = placeTrayJson;
}

int robotStatus::writeRobotNodeStatusdata(Json::Value &NodeData)
{
    NodeData["lk"] = robotName;  // 기존 getRobotName을 없애고 통합
    NodeData["nd"] = Node_status;
}