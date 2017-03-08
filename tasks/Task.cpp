/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <regex>
#include "Task.hpp"

#define TC_SERVER_PORT_NUMBER 7031
#define TM_SERVER_PORT_NUMBER 7032
#define TC_REPLY_SERVER_PORT_NUMBER 7033

const int GNC_LLO_ACKERMANN_ACTIVITY = 1;
const int GNC_LLO_TURNSPOT_ACTIVITY = 8;
const int GNC_LLO_TRAJECTORY_ACTIVITY = 11;
const int PANCAM_WAC_GET_IMAGE_ACTIVITY = 2;
const int MAST_PTU_MOVE_TO_ACTIVITY = 3;
const int PANCAM_WAC_RRGB_ACTIVITY = 4;
const int LOCCAMFRONT_GET_IMAGE_ACTIVITY = 5;
const int LOCCAMREAR_GET_IMAGE_ACTIVITY = 9;
const int BEMA_DEPLOY_1_ACTIVITY = 6;
const int BEMA_DEPLOY_2_ACTIVITY = 7;
const int BEMA_DEPLOY_3_ACTIVITY = 10;

const double DEG2RAD = 3.141592/180;
const double RAD2DEG = 180/3.141592;

const double OMEGA = 0.02;  //in Rad/s the commanded angular velocity to the walking actuators when deploying

const double PANLIMIT_LEFT = 50*DEG2RAD;
const double PANLIMIT_RIGHT = -235*DEG2RAD;
const double TILTLIMIT = 90*DEG2RAD;
const double BEMALIMIT = 95;
const double TARGET_WINDOW = 0.01;
const double TARGET_WINDOW2 = 0.01;

using namespace telemetry_telecommand;
//using namespace frame_helper;


RobotProcedure*  theRobotProcedure;// = new RobotProcedure("exoter");

RobotTask* GetRTFromName (char* rtname) {
  RobotTask* RT = ( RobotTask* ) theRobotProcedure->GetRTFromName((char*)rtname);
  return RT;
}

extern "C" {
  
  int orcExecAct(const char* rtname, const char *rtparams, int req_id) {
    RobotTask* RT = ( RobotTask* ) theRobotProcedure->GetRTFromName((char*)rtname);
    if (RT != NULL) {
      RT->SetParam((char *)rtparams);
      RT->SetTcRequestId(req_id);
      RT->Control();

      //RT->waitEndActionExec ();

      char tc_reply[80];
      sprintf(tc_reply, "%d 2 RspAck\n", RT->GetTcRequestId());
      // sprintf(tc_reply, "%d 2 RspError\n", RT->tcRequestId); // in case of error
      //tcReplyServer->sendData(tc_reply);
    }
    else {

    }
  }
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    ptu_command.resize(2);
    ptu_command.names[0]= "MAST_PAN";
    ptu_command.names[1]= "MAST_TILT";
    ptu.resize(2);
    bema.resize(6);
    inPanCamActivity=0;
    WACL_index = 1;
    WACR_index = 1;
    PAN_STEREO_index = 1;
    FLOCL_index = 1;
    FLOCR_index = 1;
    FLOC_STEREO_index = 1;
    RLOCL_index = 1;
    RLOCR_index = 1;
    RLOC_STEREO_index = 1;
    //first_estimate=true;
    //first_imu_estimate_yaw=0.0;
    initial_3Dpose=_initial_pose;
    absolute_pose=initial_3Dpose;
    initial_absolute_heading=initial_3Dpose.getYaw();
    return true;
}
bool Task::startHook()
{
  if (! TaskBase::startHook())
    return false;
  //    signal(SIGPIPE, SIG_IGN);
  
  activemq::library::ActiveMQCPP::initializeLibrary();
  bool useTopics = true;
  bool sessionTransacted = false;
  int numMessages = 2000;

  std::string brokerURI = _activeMQ_brokerURI.value();
  //std::string brokerURI = "failover:(tcp://192.168.200.241:9009)";
  
  activemqTMSender = new ActiveMQTMSender(brokerURI, 
					  numMessages, 
					  useTopics, 
					  false, "");
  
  activemqAdmin = new ActiveMQAdmin(brokerURI, numMessages, useTopics);
  
  activemqTCReceiver = new ActiveMQTCReceiver(brokerURI, 
					      2000, // numMessages, 
					      true, // useTopics, 
					      false // sessionTransacted
					      );

  theRobotProcedure = new RobotProcedure("exoter");
  tcComm = new CommTcServer( TC_SERVER_PORT_NUMBER); 
  tmComm = new CommTmServer( TM_SERVER_PORT_NUMBER, theRobotProcedure, activemqTMSender);
  tcReplyServer =  new CommTcReplyServer( TC_REPLY_SERVER_PORT_NUMBER );


  RobotTask* rt1 = new RobotTask("ADE_LEFT_Initialise"); // Simulated
  RobotTask* rt2 = new RobotTask("ADE_LEFT_conf");  // Simulated
  RobotTask* rt3 = new RobotTask("ADE_LEFT_ReleaseHDRM"); // Simulated
  RobotTask* rt4 = new RobotTask("ADE_LEFT_SwitchOff"); // Simulated
  RobotTask* rt5 = new RobotTask("ADE_RIGHT_Initialise"); // Simulated
  RobotTask* rt6 = new RobotTask("ADE_RIGHT_conf"); // Simulated
  RobotTask* rt7 = new RobotTask("ADE_RIGHT_ReleaseHDRM"); // Simulated
  RobotTask* rt8 = new RobotTask("ADE_RIGHT_SwitchOff"); // Simulated

  RobotTask* rt9 = new RobotTask("SA_LEFT_Initialise"); // Simulated
  RobotTask* rt10 = new RobotTask("SA_LEFT_PrimaryMoveTo"); // Simulated
  RobotTask* rt11 = new RobotTask("SA_LEFT_SecondaryMoveTo"); // Simulated
  RobotTask* rt12 = new RobotTask("SA_LEFT_SwitchOff"); // Simulated
  RobotTask* rt13 = new RobotTask("SA_RIGHT_Initialise"); // Simulated
  RobotTask* rt14 = new RobotTask("SA_RIGHT_PrimaryMoveTo"); // Simulated
  RobotTask* rt15 = new RobotTask("SA_RIGHT_SecondaryMoveTo"); // Simulated
  RobotTask* rt16 = new RobotTask("SA_RIGHT_SwitchOff"); // Simulated

  RobotTask* rt17 = new RobotTask("PanCam_Initialise"); // Simulated
  RobotTask* rt18 = new RobotTask("PanCam_InitWACs"); // Simulated
  RobotTask* rt19 = new RobotTask("PanCam_SwitchOn"); // Simulated
  RobotTask* rt20 = new RobotTask("PanCam_WACAcqImage"); // Simulated
  RobotTask* rt21 = new RobotTask("PanCam_WACGetImage"); // Executed (params WAC_L, WAC_R)
  RobotTask* rt22 = new RobotTask("PanCam_SwitchOff"); // Simulated 
  RobotTask* rt23 = new RobotTask("PanCam_PIUSwitchOff"); // Simulated 
  RobotTask* rt24 = new RobotTask("PANCAM_WAC_RRGB"); // Executed (params TBD)
  RobotTask* rt25 = new RobotTask("PanCam_FilterSel"); // Simulated  
  RobotTask* rt251 = new RobotTask("FrontLocCam_GetImage"); // Executed  
  RobotTask* rt252 = new RobotTask("RearLocCam_GetImage"); // Executed  

  RobotTask* rt26 = new RobotTask("MAST_DEP_Initialise"); // Simulated  
  RobotTask* rt27 = new RobotTask("MAST_DEP_Deploy"); // Simulated 
  RobotTask* rt28 = new RobotTask("MAST_PTU_Initialise"); // Simulated 
  RobotTask* rt29 = new RobotTask("MAST_PTU_MoveTo"); // Executed (params: pan, tilt (deg, deg))
  RobotTask* rt30 = new RobotTask("MAST_SwitchOff"); // Simulated 

  RobotTask* rt31 = new RobotTask("GNC_Initialise"); // Simulated 
  RobotTask* rt32 = new RobotTask("GNC_LLO_ACKERMANN"); // Executed  (params: distance, speed (m, m/hour))
  RobotTask* rt321 = new RobotTask("GNC_LLO_TURNSPOT"); // Executed  (params: distance, speed (m, m/hour))
  RobotTask* rt322 = new RobotTask("GNC_LLO_TRAJECTORY"); // Executed  (params: number of waypoints, vector of waypoints(x,y,h))
  RobotTask* rt33 = new RobotTask("GNC_SwitchOff"); // Simulated 
  RobotTask* rt331 = new RobotTask("BEMA_Deploy_1"); // Simulated or Executed 
  RobotTask* rt332 = new RobotTask("BEMA_Deploy_2"); // Simulated or Executed

  RobotTask* rt34 = new RobotTask("RV_WakeUp"); // Simulated  
  RobotTask* rt35 = new RobotTask("MMS_WaitAbsTime"); // Simulated 
  RobotTask* rt36 = new RobotTask("RV_Prepare4Comms"); // Simulated 
  RobotTask* rt37 = new RobotTask("RV_PostComms"); // Simulated 
  RobotTask* rt38 = new RobotTask("DHS_Go2Nominal"); // Simulated 
  RobotTask* rt39 = new RobotTask("RV_Prepare4Travel"); // Simulated 
  RobotTask* rt40 = new RobotTask("RV_Prepare4Night"); // Simulated 
  RobotTask* rt41 = new RobotTask("RV_Prepare4Dozing"); // Simulated 


  theRobotProcedure->insertRT(rt1);
  theRobotProcedure->insertRT(rt2);
  theRobotProcedure->insertRT(rt3);
  theRobotProcedure->insertRT(rt4);
  theRobotProcedure->insertRT(rt5);
  theRobotProcedure->insertRT(rt6);
  theRobotProcedure->insertRT(rt7);
  theRobotProcedure->insertRT(rt8);
  theRobotProcedure->insertRT(rt9);
  theRobotProcedure->insertRT(rt10);
  theRobotProcedure->insertRT(rt11);
  theRobotProcedure->insertRT(rt12);
  theRobotProcedure->insertRT(rt13);
  theRobotProcedure->insertRT(rt14);
  theRobotProcedure->insertRT(rt15);
  theRobotProcedure->insertRT(rt16);
  theRobotProcedure->insertRT(rt17);
  theRobotProcedure->insertRT(rt18);
  theRobotProcedure->insertRT(rt19);
  theRobotProcedure->insertRT(rt20);
  theRobotProcedure->insertRT(rt21);
  theRobotProcedure->insertRT(rt22);
  theRobotProcedure->insertRT(rt23);
  theRobotProcedure->insertRT(rt24);
  theRobotProcedure->insertRT(rt25);
  theRobotProcedure->insertRT(rt251);
  theRobotProcedure->insertRT(rt26);
  theRobotProcedure->insertRT(rt27);
  theRobotProcedure->insertRT(rt28);
  theRobotProcedure->insertRT(rt29);
  theRobotProcedure->insertRT(rt30);
  theRobotProcedure->insertRT(rt31);
  theRobotProcedure->insertRT(rt32);
  theRobotProcedure->insertRT(rt321);
  theRobotProcedure->insertRT(rt322);
  theRobotProcedure->insertRT(rt33);
  theRobotProcedure->insertRT(rt331);
  theRobotProcedure->insertRT(rt332);
  theRobotProcedure->insertRT(rt34);
  theRobotProcedure->insertRT(rt35);
  theRobotProcedure->insertRT(rt36);
  theRobotProcedure->insertRT(rt37);
  theRobotProcedure->insertRT(rt38);
  theRobotProcedure->insertRT(rt39);
  theRobotProcedure->insertRT(rt40);
  theRobotProcedure->insertRT(rt41);

  //! ToDo: Check if this fix is still necessary. It might be fixed at ptu_control component configure/start hook.
  //! Send ptu and motion commands to activate the joint dispatcher. Otherwise stays waiting.
  pan = 0.0; tilt = 0.0; sendPtuCommand();
  targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();


  /**
   * Routine to send the bema command to the rover stowed position (beggining of Egress)
   */
/*
  currentActivity = BEMA_DEPLOY_1_ACTIVITY;
  bema_command=-90.0;
  std::cout <<  "BEMA Deploy 1: " << bema_command << std::endl;
  bema_command = bema_command*DEG2RAD;
  _bema_command.write(-2.0*OMEGA);

  target_reached=false;
  NofWaypoints=0;
*/
  currentActivity = -1;
  abort_activity=false;
  files_sent=false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    if (_current_pose.read(pose) == RTT::NewData)
    {
        //! std::cout << "received pose: " << pose.position[0] << " " << pose.position[1] << " " << pose.position[2] << std::endl; // DEBUG
        //! new TM packet with updated pose estimation
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
          std::cout << "Error getting GNCState" << std::endl;
        }
        int aux = (int)((cos(initial_absolute_heading)*pose.position[0] - sin(initial_absolute_heading)*pose.position[1] + initial_3Dpose.position[0])*100);
        GNCState[GNC_ROVER_POSEX_INDEX]=(double)((double)aux/100.0);
	absolute_pose.position[0]=GNCState[GNC_ROVER_POSEX_INDEX];
        aux = (int)((sin(initial_absolute_heading)*pose.position[0] + cos(initial_absolute_heading)*pose.position[1] + initial_3Dpose.position[1])*100);
        GNCState[GNC_ROVER_POSEY_INDEX]=(double)((double)aux/100.0);
	absolute_pose.position[1]=GNCState[GNC_ROVER_POSEY_INDEX];
	aux = (int)((pose.position[2] + initial_3Dpose.position[2])*100);
        GNCState[GNC_ROVER_POSEZ_INDEX]=(double)((double)aux/100.0);
	absolute_pose.position[2]=GNCState[GNC_ROVER_POSEZ_INDEX];
	aux = (int)(pose.getRoll()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERX_INDEX]=(double)((double)aux/10.0);
	aux = (int)(pose.getPitch()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERY_INDEX]=(double)((double)aux/10.0);
	aux = (int)((pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG)*10);
        GNCState[GNC_ROVER_POSERZ_INDEX]=(double)((double)aux/10.0);
        //GNCState[GNC_ROVER_POSERX_INDEX]=pose.getRoll();
        //GNCState[GNC_ROVER_POSERY_INDEX]=pose.getPitch();
        //GNCState[GNC_ROVER_POSERZ_INDEX]=pose.getYaw();
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
          std::cout << "Error setting GNCState" << std::endl;
        }
    }
    if (_current_bema.read(bema) == RTT::NewData)
    {
        //! new TM packet with updated bema estimation
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
          std::cout << "Error getting GNCState" << std::endl;
        }
        int aux = (int)(bema[0].position*RAD2DEG*100);
        GNCState[GNC_ROVER_BEMA_Q1_INDEX]=(double)((double)aux/100.0);
	aux = (int)(bema[1].position*RAD2DEG*100);
        GNCState[GNC_ROVER_BEMA_Q2_INDEX]=(double)((double)aux/100.0);
	aux = (int)(bema[2].position*RAD2DEG*100);
        GNCState[GNC_ROVER_BEMA_Q3_INDEX]=(double)((double)aux/100.0);
	aux = (int)(bema[3].position*RAD2DEG*100);
        GNCState[GNC_ROVER_BEMA_Q4_INDEX]=(double)((double)aux/100.0);
	aux = (int)(bema[4].position*RAD2DEG*100);
        GNCState[GNC_ROVER_BEMA_Q5_INDEX]=(double)((double)aux/100.0);
	aux = (int)(bema[5].position*RAD2DEG*100);
        GNCState[GNC_ROVER_BEMA_Q6_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
          std::cout << "Error setting GNCState" << std::endl;
        }
    }
    if (_current_ptu.read(ptu) == RTT::NewData)
    {
        //! std::cout << "received ptu: " << ptu[0].position << " " << ptu[1].position << std::endl; // DEBUG
        //! new TM packet with updated ptu position
        if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
          std::cout << "Error getting MastState" << std::endl;
        }
	int aux = (int)(ptu[0].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q2_INDEX]=(double)((double)aux/100.0);
	aux = (int)(ptu[1].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q3_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error setting MastState" << std::endl;
        }
    }
    if (_current_imu.read(imu) == RTT::NewData)
    {
        //if (first_estimate){
        //    first_imu_estimate_yaw=imu.getYaw()*RAD2DEG;
        //    first_estimate=false;
        //}

        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
          std::cout << "Error getting GNCState" << std::endl;
        }
	int aux = (int)(imu.getRoll()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERX_INDEX]=-(double)((double)aux/10.0);
	aux = (int)(imu.getPitch()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERY_INDEX]=-(double)((double)aux/10.0);
	aux = (int)((imu.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG)*10);
        GNCState[GNC_ROVER_POSERZ_INDEX]=(double)((double)aux/10.0);
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
          std::cout << "Error setting GNCState" << std::endl;
        }
    } 
    if (_trajectory_status.read(tj_status) == RTT::NewData)
    {
        if (tj_status == 2)  //! TARGET_REACHED
            target_reached=true;
        else if (tj_status == 3) //! OUT_OF_BOUNDARIES
            abort_activity=true;
        else;
            //! Nothing to do in other status
    }

    if (_image_mast_filename.read(image_filename) == RTT::NewData)
    {
        std::cout << "check1: sending image " << image_filename << std::endl;
        std::ifstream input(image_filename.c_str(), std::ios::binary);
	//std::ifstream input("/home/exoter/Desktop/Images/FLOC_L_1.png", std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        int seq=1;
        long time=PAN_STEREO_index-1;
        std::cout << "check2: sending image" << std::endl;
        if (activemqTMSender->isConnected){
            tmComm->sendImageMessage(seq, time, size, (const unsigned char *)data, activemqTMSender->imagePanCamProducerMonitoring, transformation);
            std::cout << "check3: sending image finished" << std::endl;
        }
    }

    if (_image_front_left_filename.read(image_filename) == RTT::NewData)
    {
        std::cout << "check1: sending image " << image_filename << std::endl;
	std::ifstream input(image_filename.c_str(), std::ios::binary);
	//std::ifstream input("/home/exoter/Desktop/Images/FLOC_1_copy.png", std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        int seq=1;
        long time=FLOC_STEREO_index-1;
        std::cout << "check2: sending image" << std::endl;
        if (activemqTMSender->isConnected){
            tmComm->sendImageMessage(seq, time, size, (const unsigned char *)data, activemqTMSender->imageFLocProducerMonitoring, transformation);
            std::cout << "check3: sending image finished" << std::endl;
        }
    }

    if (_image_front_right_filename.read(image_filename) == RTT::NewData)
    {
        std::cout << "check1: sending image file " << image_filename << std::endl;
        std::ifstream input(image_filename.c_str(), std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        std::cout << "check2: sending image file with size " << size << std::endl;
        image_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(image_filename.c_str(), size, (const unsigned char *)data, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending image file finished" << std::endl;
        }
    }

    if (_image_back_left_filename.read(image_filename) == RTT::NewData)
    {
        std::cout << "check1: sending image " << image_filename << std::endl;
        std::ifstream input(image_filename.c_str(), std::ios::binary);
	//std::ifstream input("/home/exoter/Desktop/Images/FLOC_L_1.png", std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        int seq=1;
        long time=RLOC_STEREO_index-1;
        std::cout << "check2: sending image" << std::endl;
        if (activemqTMSender->isConnected){
            tmComm->sendImageMessage(seq, time, size, (const unsigned char *)data, activemqTMSender->imageRLocProducerMonitoring, transformation);
            std::cout << "check3: sending image finished" << std::endl;
        }
    }

    if (_image_back_right_filename.read(image_filename) == RTT::NewData)
    {
        std::cout << "check1: sending image file " << image_filename << std::endl;
        std::ifstream input(image_filename.c_str(), std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        std::cout << "check2: sending image file with size " << size << std::endl;
        image_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(image_filename.c_str(), size, (const unsigned char *)data, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending image file finished" << std::endl;
        }
    }

    if (_dem_mast_filename.read(dem_filename) == RTT::NewData)
    {
        std::cout << "check1: sending dem " << dem_filename << std::endl;
        std::ifstream input(dem_filename.c_str(), std::ios::binary);
	//std::ifstream input("/home/exoter/Desktop/Images/FLOC_L_1.png", std::ios::binary);
        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
        int seq=1;
        long time=(long)PAN_STEREO_index-1;
        std::cout << "check2: sending dem" << std::endl;
        std::string filename = dem_filename;
        filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendDEMMessage(filename.c_str(), seq, time, data.size(), data, activemqTMSender->demPanCamProducerMonitoring, transformation);
            std::cout << "check3: sending dem finished" << std::endl;
        }

        std::string mtl_filename = dem_filename.replace(dem_filename.find("obj"), 3, "mtl");
        std::cout << "check1: sending file " << mtl_filename << std::endl;
        char command[256];
        sprintf(command,  "sed -ie 's/\\/home\\/exoter\\/Desktop\\/Images\\///g' %s", mtl_filename.c_str());
        system(command);
        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
        auto size2 = buffer2.size();
        char* data2 = &buffer2[0];
        std::cout << "check2: sending file with size " << size2 << std::endl;
        mtl_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending file finished" << std::endl;
        }
    }

    if (_dem_front_filename.read(dem_filename) == RTT::NewData)
    {
        std::cout << "check1: sending dem " << dem_filename << std::endl;
        std::ifstream input(dem_filename.c_str(), std::ios::binary);
	//std::ifstream input("/home/exoter/Desktop/Images/FLOC_1_copy.obj", std::ios::binary);
        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
        int seq=1;
        long time=(long)FLOC_STEREO_index-1;
        std::cout << "check2: sending dem" << std::endl;
        std::string filename = dem_filename;
        filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendDEMMessage(filename.c_str(), seq, time, data.size(), data, activemqTMSender->demFLocProducerMonitoring, transformation);
            std::cout << "check3: sending dem finished" << std::endl;
        }

        std::string mtl_filename = dem_filename.replace(dem_filename.find("obj"), 3, "mtl");
        std::cout << "check1: sending file " << mtl_filename << std::endl;
        char command[256];
        sprintf(command,  "sed -ie 's/\\/home\\/exoter\\/Desktop\\/Images\\///g' %s", mtl_filename.c_str());
        system(command);
        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
        auto size2 = buffer2.size();
        char* data2 = &buffer2[0];
        std::cout << "check2: sending file with size " << size2 << std::endl;
        mtl_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending file finished" << std::endl;
        }
    }

    if (_dem_back_filename.read(dem_filename) == RTT::NewData)
    {
        std::cout << "check1: sending dem " << dem_filename << std::endl;
        std::ifstream input(dem_filename.c_str(), std::ios::binary);
	//std::ifstream input("/home/exoter/Desktop/Images/FLOC_L_1.png", std::ios::binary);
        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
        int seq=1;
        long time=(long)RLOC_STEREO_index-1;
        std::cout << "check2: sending dem" << std::endl;
        std::string filename = dem_filename;
        filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendDEMMessage(filename.c_str(), seq, time, data.size(), data, activemqTMSender->demRLocProducerMonitoring, transformation);
            std::cout << "check3: sending dem finished" << std::endl;
        }

        std::string mtl_filename = dem_filename.replace(dem_filename.find("obj"), 3, "mtl");
        std::cout << "check1: sending file " << mtl_filename << std::endl;
        char command[256];
        sprintf(command,  "sed -ie 's/\\/home\\/exoter\\/Desktop\\/Images\\///g' %s", mtl_filename.c_str());
        system(command);
        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
        auto size2 = buffer2.size();
        char* data2 = &buffer2[0];
        std::cout << "check2: sending file with size " << size2 << std::endl;
        mtl_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending file finished" << std::endl;
        }
    }

    if (_dist_mast_filename.read(dist_filename) == RTT::NewData)
    {
        std::cout << "check1: sending file " << dist_filename << std::endl;
        std::ifstream input(dist_filename.c_str(), std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        std::cout << "check2: sending file with size " << size << std::endl;
        dist_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(dist_filename.c_str(), size, (const unsigned char *)data, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending file finished" << std::endl;
        }
        files_sent=true;
    }

    if (_dist_front_filename.read(dist_filename) == RTT::NewData)
    {
        std::cout << "check1: sending file " << dist_filename << std::endl;
        std::ifstream input(dist_filename.c_str(), std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        std::cout << "check2: sending file with size " << size << std::endl;
        dist_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(dist_filename.c_str(), size, (const unsigned char *)data, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending file finished" << std::endl;
        }
        files_sent=true;
    }

    if (_dist_back_filename.read(dist_filename) == RTT::NewData)
    {
        std::cout << "check1: sending file " << dist_filename << std::endl;
        std::ifstream input(dist_filename.c_str(), std::ios::binary);
        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        auto size = buffer.size();
        char* data = &buffer[0];
        std::cout << "check2: sending file with size " << size << std::endl;
        dist_filename.replace(0, 28, "");
        if (activemqTMSender->isConnected){
            tmComm->sendFileMessage(dist_filename.c_str(), size, (const unsigned char *)data, activemqTMSender->fileProducerMonitoring);
            std::cout << "check3: sending file finished" << std::endl;
        }
        files_sent=true;
    }

    CommandInfo* cmd_info = activemqTCReceiver->extractCommandInfo();
    // CommandInfo* cmd_info = tcComm->extractCommandInfo();
    if (cmd_info != NULL)
    {
      if (!strcmp((cmd_info->activityName).c_str(), "ABORT"))
      {
          abort_activity=true;
      }
      //! Check if there is NO running activity
      else if ((currentActivity == -1) && (inPanCamActivity == 0))
      {
	if (!strcmp((cmd_info->activityName).c_str(), "GNC_LLO_ACKERMANN")) {
	  currentActivity = GNC_LLO_ACKERMANN_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf %lf %lf", &ackid, &targetPositionX, &targetPositionY, &targetSpeed);
          motionCommand();
	  travelledDistance = 0.0;
          initial_pose = pose;
	  std::cout <<  "GNC_LLO_ACKERMANN X:" << targetPositionX << " Y:" << targetPositionY << " speed:" << targetSpeed << std::endl;
          sendMotionCommand();
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          //GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
          if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error setting GNCState" << std::endl;
          }
        }
        else if (!strcmp((cmd_info->activityName).c_str(), "GNC_LLO_TURNSPOT")) {
	  currentActivity = GNC_LLO_TURNSPOT_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf", &ackid, &targetOrientationTheta);
          motionCommand();
	  travelledAngle = 0.0;
          initial_imu = pose;
	  std::cout <<  "GNC_LLO_TURNSPOT angle:" << targetOrientationTheta << std::endl;
          sendMotionCommand();
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          //GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
          if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error setting GNCState" << std::endl;
          }
        }
        else if (!strcmp((cmd_info->activityName).c_str(), "GNC_LLO_TRAJECTORY")) {
	  currentActivity = GNC_LLO_TRAJECTORY_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid; 
          char *token_str; 
          token_str = strtok((char *)(currentParams.c_str()), " ");
          ackid = atoi(token_str);
          token_str = strtok(NULL, " ");
          NofWaypoints = atoi(token_str);
          std::cout << "NofWaypoints:" << NofWaypoints << "<-" << std::endl;
          for (int i=0;i<NofWaypoints;i++){
              token_str = strtok(NULL, " ");
              waypoint.position(0)=atof(token_str);
              std::cout << "Point " << i+1 << " x:" << waypoint.position(0) << std::endl;
              token_str = strtok(NULL, " ");
              waypoint.position(1)=atof(token_str);
              std::cout << "Point " << i+1 << " y:" << waypoint.position(1) << std::endl;
              trajectory.push_back(waypoint);
          } 
          token_str = strtok(NULL, " ");
          targetOrientationTheta = atof(token_str);
          std::cout << "targetOrientationTheta: " << targetOrientationTheta << std::endl;
          trajectory.back().heading = targetOrientationTheta*DEG2RAD;
          target_reached=false;
          _trajectory.write(trajectory);
	  std::cout <<  "GNC_LLO_TRAJECTORY #ofWaypoints:" << NofWaypoints << std::endl;
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          //GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
          if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error setting GNCState" << std::endl;
          }
        }
 	else if (!strcmp((cmd_info->activityName).c_str(), "BEMA_Deploy_1")) {
	  currentActivity = BEMA_DEPLOY_1_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf", &ackid, &bema_command);
          if (bema_command>BEMALIMIT)
              bema_command=BEMALIMIT;
          else if (bema_command<-BEMALIMIT)
              bema_command=-BEMALIMIT;
	  std::cout <<  "BEMA Deploy 1: " << bema_command << std::endl;
          bema_command = bema_command*DEG2RAD;
          if (bema_command>bema[0].position){
              _bema_command.write(OMEGA);
          } else {
              _bema_command.write(-OMEGA);
          }
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          //GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
          if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error setting GNCState" << std::endl;
          }
        }
        else if (!strcmp((cmd_info->activityName).c_str(), "BEMA_Deploy_2")) {
	  currentActivity = BEMA_DEPLOY_2_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf", &ackid, &bema_command);
	  if (bema_command>BEMALIMIT)
              bema_command=BEMALIMIT;
          else if (bema_command<-BEMALIMIT)
              bema_command=-BEMALIMIT;
          std::cout <<  "BEMA Deploy 2: " << bema_command << std::endl;
          bema_command = bema_command*DEG2RAD;
          if (bema_command>bema[0].position){
              _walking_command_front.write(OMEGA);
          } else {
              _walking_command_front.write(-OMEGA);
          }
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          //GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
          if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error setting GNCState" << std::endl;
          }
        }
        else if (!strcmp((cmd_info->activityName).c_str(), "BEMA_Deploy_3")) {
	  currentActivity = BEMA_DEPLOY_3_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf", &ackid, &bema_command);
	  if (bema_command>BEMALIMIT)
              bema_command=BEMALIMIT;
          else if (bema_command<-BEMALIMIT)
              bema_command=-BEMALIMIT;
          std::cout <<  "BEMA Deploy 3: " << bema_command << std::endl;
          bema_command = bema_command*DEG2RAD;
          if (bema_command>bema[4].position){
              _walking_command_rear.write(OMEGA);
          } else {
              _walking_command_rear.write(-OMEGA);
          }
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          //GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
          if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error setting GNCState" << std::endl;
          }
        }
        else if (!strcmp((cmd_info->activityName).c_str(), "MAST_PTU_MoveTo")) {
	  currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf %lf", &ackid, &pan, &tilt);
	  std::cout <<  "MAST_PTU_MoveTo pan:" << pan << " tilt:" << tilt << std::endl;
	  pan = pan*DEG2RAD;
	  tilt = tilt*DEG2RAD;
          sendPtuCommand();
          if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
              std::cout << "Error getting MastState" << std::endl;
          }
          //MastState[0]=0.0; //! Need to check indexes and corresponding values for the Mast States
          if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
              std::cout << "Error setting MastState" << std::endl;
          }
        }
	else if (!strcmp((cmd_info->activityName).c_str(), "PanCam_WACGetImage")) {
	  currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	  currentParams = cmd_info->activityParams; //! LOC_CAM, NAV_CAM, PAN_CAM
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %s %s", &ackid, cam, dummy_param);
	  std::cout <<  "PanCam WAC Get Image from:" << cam << std::endl;
          if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error getting PanCamState" << std::endl;
          }
          PanCamState[PANCAM_ACTION_ID_INDEX]=50;
          PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
          //PanCamState[]=;
          if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error setting PanCamState" << std::endl;
          }
	}
        else if (!strcmp((cmd_info->activityName).c_str(), "FrontLocCam_GetImage")) {
	  currentActivity = LOCCAMFRONT_GET_IMAGE_ACTIVITY;
	  currentParams = cmd_info->activityParams; //! LOC_CAM, NAV_CAM, PAN_CAM
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %s", &ackid, cam);
	  std::cout <<  "LocCamFront Get Image from:" << cam << std::endl;
          if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error getting LocCamState" << std::endl;
          }
          PanCamState[PANCAM_ACTION_ID_INDEX]=51;
          PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
          //PanCamState[]=;
          if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error setting LocCamState" << std::endl;
          }
	}
        else if (!strcmp((cmd_info->activityName).c_str(), "RearLocCam_GetImage")) {
	  currentActivity = LOCCAMREAR_GET_IMAGE_ACTIVITY;
	  currentParams = cmd_info->activityParams; //! LOC_CAM, NAV_CAM, PAN_CAM
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %s", &ackid, cam);
	  std::cout <<  "LocCamRear Get Image from:" << cam << std::endl;
          if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error getting LocCamState" << std::endl;
          }
          PanCamState[PANCAM_ACTION_ID_INDEX]=52;
          PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
          //PanCamState[]=;
          if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error setting LocCamState" << std::endl;
          }
	}
	else if (!strcmp((cmd_info->activityName).c_str(), "PANCAM_WAC_RRGB")) {
	  currentActivity = PANCAM_WAC_RRGB_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
          sscanf(currentParams.c_str(), "%d %lf", &ackid, &tilt);
	  std::cout <<  "PanCam WAC RRGB at tilt: " << tilt << std::endl;
          if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error getting PanCamState" << std::endl;
          }
          //PanCamState[0]=0.0; //! Need to check indexes and corresponding values for the PanCam States
          if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error setting PanCamState" << std::endl;
          }
	}
	else {
	  RobotTask *rover_action = ( RobotTask* ) theRobotProcedure->GetRTFromName( (char*)(cmd_info->activityName).c_str());
	  if (rover_action != NULL) {
	    orcExecAct((char*)cmd_info->activityName.c_str(),(char*)cmd_info->activityParams.c_str(), 1);
	  }
	  else {
	    std::cout <<  "DEBUG: TC command not recognised!" << std::endl;
	  }
	}
      }
    }
    if (currentActivity == GNC_LLO_ACKERMANN_ACTIVITY) {
      travelledDistance = getTravelledDistance();
      if ((travelledDistance >= targetDistance) || abort_activity) {
        abort_activity=false;
      	travelledDistance = 0.0;
	targetDistance = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetSpeed=0.0;
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=32;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == GNC_LLO_TURNSPOT_ACTIVITY) {
      travelledAngle = getTravelledAngle();
      if ((travelledAngle >= targetOrientationTheta) || abort_activity) {
        abort_activity=false;
      	travelledAngle = 0.0;
	targetOrientationTheta = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=33;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == GNC_LLO_TRAJECTORY_ACTIVITY) {
      if (target_reached || abort_activity) {
        abort_activity=false;
        target_reached=false;
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetOrientationTheta=0.0;
        trajectory.clear();
        _trajectory.write(trajectory);
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=34;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == BEMA_DEPLOY_1_ACTIVITY) {
      if (bema1TargetReached() || abort_activity) {
        //bema_command = 0.0;
        abort_activity=false;
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=35;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == BEMA_DEPLOY_2_ACTIVITY) {
      if (bema2TargetReached() || abort_activity) {
        abort_activity=false;
        //bema_command = 0.0;
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=36;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == BEMA_DEPLOY_3_ACTIVITY) {
      if (bema3TargetReached() || abort_activity) {
        abort_activity=false;
        //bema_command = 0.0;
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=37;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == MAST_PTU_MOVE_TO_ACTIVITY) {
      if (ptuTargetReached() || abort_activity) {
        abort_activity=false;
        currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error getting MastState" << std::endl;
        }
        MastState[MAST_ACTION_RET_INDEX]=ACTION_RET_OK;
        MastState[MAST_ACTION_ID_INDEX]=0;
        MastState[MAST_STATUS_INDEX]=MAST_OPER_MODE_PTU_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error setting MastState" << std::endl;
        }
      }else {
	if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error getting MastState" << std::endl;
        }
        MastState[MAST_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        MastState[MAST_STATUS_INDEX]=MAST_OPER_MODE_PTU_MOVING;
        MastState[MAST_ACTION_ID_INDEX]=35;
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error setting MastState" << std::endl;
        }
      }
    }
    else if (currentActivity == PANCAM_WAC_GET_IMAGE_ACTIVITY) {
      if (!strcmp(cam, "WAC_L")) {
        char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d.png 1", cam, WACL_index++);
	_camera_mast_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_metadata.txt", cam, WACL_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << cam << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb32lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);
        
        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting PanCamState" << std::endl;
        }
        PanCamState[PANCAM_WAC_L_INDEX]=WACL_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting PanCamState" << std::endl;
        }
      }
      else if (!strcmp(cam, "WAC_R")) {
	char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d.png 2", cam, WACR_index++);
    	_camera_mast_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_metadata.txt", cam, WACR_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << cam << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb32lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting PanCamState" << std::endl;
        }
        PanCamState[PANCAM_WAC_R_INDEX]=WACR_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting PanCamState" << std::endl;
        }
      }
      else if (!strcmp(cam, "PANCAM_STEREO")) {
	char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_left.png 1", cam, PAN_STEREO_index++);
    	_camera_mast_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_left_metadata.txt", cam, PAN_STEREO_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << "WAC_L" << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
 	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb32lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        char filename2[240];
        //sprintf (filename2, "/home/exoter/Desktop/Images/%s_%d_right.png 2", cam, PAN_STEREO_index-1);
    	//_camera_mast_store_image_filename.write(filename2);
        sprintf (filename2, "/home/exoter/Desktop/Images/%s_%d_right_metadata.txt", cam, PAN_STEREO_index-1);
        std::ofstream metadata2;
        metadata2.open(filename2);
        metadata2 << "Camera ID:  " << "WAC_R" << std::endl;
        metadata2 << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata2 << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata2 << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata2 << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	if (_left_camera_bb32lab.get(time, tf, false))
	{
		metadata2 << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting PanCamState" << std::endl;
        }
        PanCamState[PANCAM_PAN_STEREO_INDEX]=PAN_STEREO_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting PanCamState" << std::endl;
        }
      }
      else {
	std::cout << "Please select one of the existing cameras WAC_L or WAC_R" << std::endl;
      }
      currentActivity = -1;
    }
    else if (currentActivity == LOCCAMFRONT_GET_IMAGE_ACTIVITY) {
      if (!strcmp(cam, "FLOC_L")) {
        char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d.png 1", cam, FLOCL_index++);
	_camera_front_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_metadata.txt", cam, FLOCL_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << cam << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb2_front2lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting LocCamState" << std::endl;
        }
        PanCamState[LOCCAM_FLOC_L_INDEX]=FLOCL_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting LocCamState" << std::endl;
        }
      }
      else if (!strcmp(cam, "FLOC_R")) {
	char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d.png 2", cam, FLOCR_index++);
	_camera_front_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_metadata.txt", cam, FLOCR_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << cam << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb2_front2lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting LocCamState" << std::endl;
        }
        PanCamState[LOCCAM_FLOC_R_INDEX]=FLOCR_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting LocCamState" << std::endl;
        }
      }
      else if (!strcmp(cam, "FLOC_STEREO")) {
	char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d 3", cam, FLOC_STEREO_index++);
	_camera_front_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_left_metadata.txt", cam, FLOC_STEREO_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << "FLOC_L" << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
 	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb2_front2lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);
       
	sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_right_metadata.txt", cam, FLOC_STEREO_index-1);
        std::ofstream metadata2;
        metadata2.open(filename);
        metadata2 << "Camera ID:  " << "FLOC_R" << std::endl;
        metadata2 << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata2 << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata2 << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata2 << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	if (_left_camera_bb2_front2lab.get(time, tf, false))
	{
		metadata2 << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting LocCamState" << std::endl;
        }
        PanCamState[LOCCAM_FLOC_STEREO_INDEX]=FLOC_STEREO_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting LocCamState" << std::endl;
        }
      }
      else {
	std::cout << "Please select one of the existing cameras LOC_L or LOC_R" << std::endl;
      }
      currentActivity = -1;
    }
    else if (currentActivity == LOCCAMREAR_GET_IMAGE_ACTIVITY) {
      if (!strcmp(cam, "RLOC_L")) {
        char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d.png 1", cam, RLOCL_index++);
	_camera_back_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_metadata.txt", cam, RLOCL_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << cam << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb2_back2lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting LocCamState" << std::endl;
        }
        PanCamState[LOCCAM_RLOC_L_INDEX]=RLOCL_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting LocCamState" << std::endl;
        }
      }
      else if (!strcmp(cam, "RLOC_R")) {
	char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d.png 2", cam, RLOCR_index++);
	_camera_back_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_metadata.txt", cam, RLOCR_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << cam << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb2_back2lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting LocCamState" << std::endl;
        }
        PanCamState[LOCCAM_RLOC_R_INDEX]=RLOCR_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting LocCamState" << std::endl;
        }
      }
      else if (!strcmp(cam, "RLOC_STEREO")) {
	char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d 3", cam, RLOC_STEREO_index++);
	_camera_back_process_image_trigger.write(true);
        sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_left_metadata.txt", cam, RLOC_STEREO_index-1);
        std::ofstream metadata;
        metadata.open(filename);
        metadata << "Camera ID:  " << "RLOC_L" << std::endl;
        metadata << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
        metadata << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
 	Eigen::Affine3d tf;
	base::Time time = base::Time::now();
	if (_left_camera_bb2_back2lab.get(time, tf, false))
	{
		metadata << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);
       
	sprintf (filename, "/home/exoter/Desktop/Images/%s_%d_right_metadata.txt", cam, RLOC_STEREO_index-1);
        std::ofstream metadata2;
        metadata2.open(filename);
        metadata2 << "Camera ID:  " << "RLOC_R" << std::endl;
        metadata2 << "Pan:  " << ptu[0].position*RAD2DEG << std::endl;
        metadata2 << "Tilt:  " << ptu[1].position*RAD2DEG << std::endl;
	metadata2 << "Position X, Y, Z:  " << absolute_pose.position[0] << ", " << absolute_pose.position[1] << ", " << absolute_pose.position[2] << std::endl;
        metadata2 << "Orientation Roll, Pitch, Yaw:  " << pose.getRoll()*RAD2DEG << ", " << pose.getPitch()*RAD2DEG << ", " << (pose.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG) << std::endl;
	if (_left_camera_bb2_back2lab.get(time, tf, false))
	{
		metadata2 << "Left Camera Transformation x,y,z, qx, qy, qz, qw: " << tf.translation().x() << ", " << tf.translation().y() << ", " << tf.translation().z() 
                    << ", " << Eigen::Quaterniond(tf.linear()).x()  << ", " << Eigen::Quaterniond(tf.linear()).y()  << ", " << Eigen::Quaterniond(tf.linear()).z() << ", " << Eigen::Quaterniond(tf.linear()).w() << std::endl;
	}
        getTransform(tf);

        if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error getting LocCamState" << std::endl;
        }
        PanCamState[LOCCAM_RLOC_STEREO_INDEX]=RLOC_STEREO_index-1;
        PanCamState[PANCAM_ACTION_ID_INDEX]=0;
        PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
        if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
           std::cout << "Error setting LocCamState" << std::endl;
        }
      }
      else {
	std::cout << "Please select one of the existing cameras LOC_L or LOC_R" << std::endl;
      }
      currentActivity = -1;
    }
    else if ((currentActivity == PANCAM_WAC_RRGB_ACTIVITY) || inPanCamActivity) {
      switch (inPanCamActivity) {
	case 0:
          currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	  pan = 45.0*DEG2RAD;
	  tilt = tilt*DEG2RAD;
          sendPtuCommand();
	  inPanCamActivity++;
	  break;
      	case 1:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
	case 2:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = 10.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 3:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
	case 4:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -25.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 5:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
	case 6:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -60.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 7:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
	case 8:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -95.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 9:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
        case 10:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -130.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 11:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
        case 12:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -165.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
        case 13:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
        case 14:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -200.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 15:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
        case 16:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -235.0*DEG2RAD;
	    //tilt = tilt*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 17:
	  if ((currentActivity == -1) && files_sent) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    strcpy(cam,"PANCAM_STEREO");
	    inPanCamActivity++;
            files_sent=false;
	  }
	  break;
	case 18:
	  if (currentActivity == -1) {
 	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
            pan = 0.0;
            tilt = 0.0;
            sendPtuCommand();
	    inPanCamActivity++;
	  }
	  break;
        case 19:
          if (currentActivity == -1) {
            inPanCamActivity=0;
          }
          break;
        default:
          break;
      }
      if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
          std::cout << "Error getting PanCamState" << std::endl;
      }
      //PanCamState[0]=0.0; //! Need to check indexes and corresponding values for the PanCam States
      if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
          std::cout << "Error setting PanCamState" << std::endl;
      }
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

double Task::getTravelledDistance()
{
    double distance = 0.0;
    distance = sqrt((pose.position[0]-initial_pose.position[0])*(pose.position[0]-initial_pose.position[0])
                   +(pose.position[1]-initial_pose.position[1])*(pose.position[1]-initial_pose.position[1])
                   +(pose.position[2]-initial_pose.position[2])*(pose.position[2]-initial_pose.position[2]));
    return distance;
}

double Task::getTravelledAngle()
{
    double angle =0.0;
    angle = std::abs(pose.getYaw()-initial_imu.getYaw());
    return (angle*RAD2DEG);
}

void Task::getTransform(Eigen::Affine3d& tf)
{
	transformation[0] = tf.translation().x();
	transformation[1] = tf.translation().y();
	transformation[2] = tf.translation().z();
	transformation[3] = Eigen::Quaterniond(tf.linear()).x();
	transformation[4] = Eigen::Quaterniond(tf.linear()).y();
	transformation[5] = Eigen::Quaterniond(tf.linear()).z();
	transformation[6] = Eigen::Quaterniond(tf.linear()).w();
}

void Task::motionCommand()
{
    if (currentActivity == GNC_LLO_ACKERMANN_ACTIVITY){
        if (targetPositionY == 0){ // Straight line command
            targetDistance = std::abs(targetPositionX);
            double sign = (targetPositionX < 0 ? -1 : 1);
            targetTranslation = targetSpeed*sign;
            targetRotation = 0;
            return;
        }
        double radius = (targetPositionX*targetPositionX + targetPositionY*targetPositionY)/(2*targetPositionY); //ToDo Check minimum radius and exit if requested command has a radius that is too small. Propose a Turnspot
        double theta = atan(targetPositionX/std::abs(radius-targetPositionY));
        double sign = (targetPositionX < 0 ? -1 : 1);
        targetTranslation = targetSpeed*sign;
        targetRotation = targetTranslation/radius;
        targetDistance = std::abs(theta*radius);
    }
    else if (currentActivity == GNC_LLO_TURNSPOT_ACTIVITY){
        targetTranslation=0.0;
        if (targetOrientationTheta>=0) {
            targetRotation=0.05; //ToDo Change this to parameter in the command
        }
        else {
            targetRotation=-0.05; //ToDo Change this to parameter in the command
            targetOrientationTheta=-targetOrientationTheta;
        }
    }
}


bool Task::ptuTargetReached()
{
    if (abort_activity)
    {
        pan = ptu[0].position;
        tilt = ptu[1].position;
        sendPtuCommand();
        return true;
    }
    double window = TARGET_WINDOW;
    if (std::abs(ptu[0].position-pan) > window)
        return false;
    if (std::abs(ptu[1].position-tilt) > window)
        return false;
    std::cout << "---- >>>> ptu target reached!" << std::endl;
    return true;
}

bool Task::bema1TargetReached()
{
    if (abort_activity)
    {
        targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
        return true;
    }
    double window = TARGET_WINDOW2;
    for (int i=0;i<6;i++)
    {
        std::cout << "bema_command: " << bema_command << " bema[].position: " << bema[i].position << std::endl;
        if (std::abs(bema[i].position-bema_command) < window)
        {
            std::cout << "---- >>>> bema1 target reached!" << std::endl;
            targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
            return true;
        }
    }
    return false;
}

bool Task::bema2TargetReached()
{
    if (abort_activity)
    {
        targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
        return true;
    }
    double window = TARGET_WINDOW2;
    for (int i=0;i<2;i++)
    {
        std::cout << "bema_command: " << bema_command << " bema[].position: " << bema[i].position << std::endl;
        if (std::abs(bema[i].position-bema_command) < window)
        {
           std::cout << "---- >>>> bema2 target reached!" << std::endl;
           targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
           return true;
        }
    }
    return false;
}

bool Task::bema3TargetReached()
{
    if (abort_activity)
    {
        targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
        return true;
    }
    double window = TARGET_WINDOW2;
    for (int i=0;i<2;i++)
    {
        std::cout << "bema_command: " << bema_command << " bema[].position: " << bema[4+i].position << std::endl;
        if (std::abs(bema[4+i].position-bema_command) < window)
        {
           std::cout << "---- >>>> bema3 target reached!" << std::endl;
           targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
           return true;
        }
    }
    return false;
}

void Task::sendPtuCommand()
{
    if (pan > PANLIMIT_LEFT)
	pan = PANLIMIT_LEFT;
    else if ( pan < PANLIMIT_RIGHT)
	pan = PANLIMIT_RIGHT;
    if (tilt > TILTLIMIT)
	tilt = TILTLIMIT;
    else if (tilt < -TILTLIMIT)
	tilt = - TILTLIMIT;
    ptu_command[0].position=pan;
    ptu_command[0].speed=base::NaN<float>();
    ptu_command[1].position=tilt;
    ptu_command[1].speed=base::NaN<float>();
    _ptu_command.write(ptu_command);
}
void Task::sendMotionCommand()
{
    motion_command.translation = targetTranslation;
    motion_command.rotation = targetRotation;
    _locomotion_command.write(motion_command);
}
