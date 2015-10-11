/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <cmath>

#include "Task.hpp"

#define TC_SERVER_PORT_NUMBER 7031
#define TM_SERVER_PORT_NUMBER 7032
#define TC_REPLY_SERVER_PORT_NUMBER 7033

const int GNC_LLO_ACTIVITY = 1;
const int PANCAM_WAC_GET_IMAGE_ACTIVITY = 2;
const int MAST_PTU_MOVE_TO_ACTIVITY = 3;
const int PANCAM_WAC_RRGB_ACTIVITY = 4;
const int BEMA_DEPLOY_1_ACTIVITY = 5;
const int BEMA_DEPLOY_2_ACTIVITY = 6;

const double DEG2RAD = 3.141592/180;

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
    inPanCamActivity=0;
    return true;
}
bool Task::startHook()
{
  if (! TaskBase::startHook())
    return false;
  //    signal(SIGPIPE, SIG_IGN);
  
  theRobotProcedure = new RobotProcedure("exoter");
  tcComm = new CommTcServer( TC_SERVER_PORT_NUMBER); 
  tmComm = new CommTmServer( TM_SERVER_PORT_NUMBER, theRobotProcedure);
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
  RobotTask* rt24 = new RobotTask("PanCam_WAC_RRGB"); // Executed (params TBD)
  RobotTask* rt25 = new RobotTask("PanCam_FilterSel"); // Simulated  

  RobotTask* rt26 = new RobotTask("MAST_DEP_Initialise"); // Simulated  
  RobotTask* rt27 = new RobotTask("MAST_DEP_Deploy"); // Simulated 
  RobotTask* rt28 = new RobotTask("MAST_PTU_Initialise"); // Simulated 
  RobotTask* rt29 = new RobotTask("MAST_PTU_MoveTo"); // Executed (params: pan, tilt (deg, deg))
  RobotTask* rt30 = new RobotTask("MAST_SwitchOff"); // Simulated 

  RobotTask* rt31 = new RobotTask("GNC_Initialise"); // Simulated 
  RobotTask* rt32 = new RobotTask("GNC_LLO"); // Executed  (params: distance, speed (m, m/hour))
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
  theRobotProcedure->insertRT(rt26);
  theRobotProcedure->insertRT(rt27);
  theRobotProcedure->insertRT(rt28);
  theRobotProcedure->insertRT(rt29);
  theRobotProcedure->insertRT(rt30);
  theRobotProcedure->insertRT(rt31);
  theRobotProcedure->insertRT(rt32);
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

  //! Send ptu and motion commands to activate the joint dispatcher
  pan = 0.0; tilt = 0.0; sendPtuCommand();
  targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();

  currentActivity = -1;
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
        GNCState[7]=7.0; //! Need to check indexes and corresponding values for the GNC States
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
        MastState[0]=0.0; //! Need to check indexes and corresponding values for the Mast States
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error setting MastState" << std::endl;
        }
    }

    //! Check list of telecommands only if there is NO running activity
    if ((currentActivity == -1) && (inPanCamActivity == 0))
    {
      CommandInfo* cmd_info = tcComm->extractCommandInfo();
      if (cmd_info != NULL)
      {
	if (!strcmp((cmd_info->activityName).c_str(), "GNC_LLO")) {
	  currentActivity = GNC_LLO_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %lf %lf", &ackid, &targetDistance, &targetTranslation);
          targetRotation= 0.0;
	  travelledDistance = 0.0;
          initial_pose = pose;
	  std::cout <<  "GNC_LLO distance:" << targetDistance << " speed:" << targetTranslation << std::endl;
          sendMotionCommand();
          if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
              std::cout << "Error getting GNCState" << std::endl;
          }
          GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
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
          sendPtuCommand();
          if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
              std::cout << "Error getting MastState" << std::endl;
          }
          MastState[0]=0.0; //! Need to check indexes and corresponding values for the Mast States
          if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
              std::cout << "Error setting MastState" << std::endl;
          }
        }
	else if (!strcmp((cmd_info->activityName).c_str(), "PanCam_WACGetImage")) {
	  currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	  currentParams = cmd_info->activityParams; //! LOC_CAM, NAV_CAM, PAN_CAM
	  int ackid;
	  sscanf(currentParams.c_str(), "%d %s", &ackid, &cam);
	  std::cout <<  "PanCam WAC Get Image from:" << cam << std::endl;
          if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error getting PanCamState" << std::endl;
          }
          PanCamState[0]=0.0; //! Need to check indexes and corresponding values for the PanCam States
          if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error setting PanCamState" << std::endl;
          }
	}
	else if (!strcmp((cmd_info->activityName).c_str(), "PanCam_WAC_RRGB")) {
	  currentActivity = PANCAM_WAC_RRGB_ACTIVITY;
	  currentParams = cmd_info->activityParams;
	  int ackid;
	  //sscanf(currentParams.c_str(), "%d %s", &ackid, &params);
	  std::cout <<  "PanCam WAC RRGB:" << std::endl; // << no params?
          if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
              std::cout << "Error getting PanCamState" << std::endl;
          }
          PanCamState[0]=0.0; //! Need to check indexes and corresponding values for the PanCam States
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
    else if (currentActivity == GNC_LLO_ACTIVITY) {
      travelledDistance = computeTravelledDistance();
      if (travelledDistance >= targetDistance) {
      	travelledDistance = 0.0;
	targetDistance = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
	currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[0]=0.0; //! Need to check indexes and corresponding values for the GNC States
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR ){
            std::cout << "Error setting GNCState" << std::endl;
        }
      }
    }
    else if (currentActivity == MAST_PTU_MOVE_TO_ACTIVITY) {
      if (ptuTargetReached()) {
        currentActivity = -1;
	if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error getting MastState" << std::endl;
        }
        MastState[0]=0.0; //! Need to check indexes and corresponding values for the Mast States
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR ){
            std::cout << "Error setting MastState" << std::endl;
        }
      }
    }
    else if (currentActivity == PANCAM_WAC_GET_IMAGE_ACTIVITY) {
      if (!strcmp(cam.c_str(), "WAC_L")) {
	_left_frame.read(frame_left);
	_store_image_filename.write("~/Desktop/Images/left.jpg");
	//! Do something with frame_left;
      }
      else if (!strcmp(cam.c_str(), "WAC_R")) {
	_right_frame.read(frame_right);
	_store_image_filename.write("~/Desktop/Images/right.jpg");
	//! Do something with frame_right;
      }
      else {
	std::cout << "Please select one of the existing cameras WAC_L or WAC_R" << std::endl;
      }
      currentActivity = -1;
      if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
          std::cout << "Error getting PanCamState" << std::endl;
      }
      PanCamState[0]=0.0; //! Need to check indexes and corresponding values for the PanCam States
      if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
          std::cout << "Error setting PanCamState" << std::endl;
      }
    }
    else if ((currentActivity == PANCAM_WAC_RRGB_ACTIVITY) || inPanCamActivity) {
      switch (inPanCamActivity) {
	case 0:
          currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	  pan = 140.0*DEG2RAD;
	  tilt = 10.0*DEG2RAD;
          sendPtuCommand();
	  inPanCamActivity++;
	  break;
      	case 1:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    cam = "CAM_L\n";
	    inPanCamActivity++;
	  }
	  break;
	case 2:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = 90.0*DEG2RAD;
	    tilt = 10.0*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 3:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    cam = "CAM_L\n";
	    inPanCamActivity++;
	  }
	  break;
	case 4:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = 30.0*DEG2RAD;
	    tilt = 10.0*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 5:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    cam = "CAM_L\n";
	    inPanCamActivity++;
	  }
	  break;
	case 6:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -30.0*DEG2RAD;
	    tilt = 10.0*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 7:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    cam = "CAM_L\n";
	    inPanCamActivity++;
	  }
	  break;
	case 8:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -90.0*DEG2RAD;
	    tilt = 10.0*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 9:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    cam = "CAM_L\n";
	    inPanCamActivity++;
	  }
	  break;
	case 10:
	  if (currentActivity == -1) {
	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
	    pan = -140.0*DEG2RAD;
	    tilt = 10.0*DEG2RAD;
	    sendPtuCommand();
            inPanCamActivity++;
	  }
	  break;
	case 11:
	  if (currentActivity == -1) {
	    currentActivity = PANCAM_WAC_GET_IMAGE_ACTIVITY;
	    cam = "CAM_L\n";
	    inPanCamActivity++;
	  }
	  break;
	case 12:
	  if (currentActivity == -1) {
 	    currentActivity = MAST_PTU_MOVE_TO_ACTIVITY;
            pan = 0.0;
            tilt = 0.0;
            sendPtuCommand();
	    inPanCamActivity++;
	  }
	  break;
        case 13:
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
      PanCamState[0]=0.0; //! Need to check indexes and corresponding values for the PanCam States
      if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR ){
          std::cout << "Error setting PanCamState" << std::endl;
      }
    }

    //! Send telemetry data
    //! orcGetTmMsg(tmmsg); // Do I need to do this here? Or should I edit the tmgeneration.cpp file and do get/set all the states?
    //! Send telemetry data
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

double Task::computeTravelledDistance()
{
    double distance = 0.0;
    distance = sqrt((pose.position[0]-initial_pose.position[0])*(pose.position[0]-initial_pose.position[0])
                   +(pose.position[1]-initial_pose.position[1])*(pose.position[1]-initial_pose.position[1])
                   +(pose.position[2]-initial_pose.position[2])*(pose.position[2]-initial_pose.position[2]));
    return distance;
}

bool Task::ptuTargetReached()
{
    double window = 0.001;
    if (std::abs(ptu[0].position-pan) > window)
        return false;
    if (std::abs(ptu[1].position-tilt) > window)
        return false;
    std::cout << "---- >>>> ptu target reached!" << std::endl;
    return true;
}

void Task::sendPtuCommand()
{
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
