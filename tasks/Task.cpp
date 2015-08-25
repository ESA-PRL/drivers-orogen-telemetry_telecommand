/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#define TC_SERVER_PORT_NBR 7031
#define TM_SERVER_PORT_NBR 7032
#define TC_REPLY_SERVER_PORT_NBR 7033

using namespace telemetry_telecommand;

static std::list<RobotTask*> RobotTasks;

RobotTask* GetRTFromName (char* name) {
  std::list<RobotTask*>::iterator pr; 
    for ( pr = RobotTasks.begin(); pr != RobotTasks.end(); pr++ ) {
    if ( !strcmp( (*pr)->GetName().c_str(), name ) ) 
      return (*pr);
  }
  return NULL;
}

extern "C" {
  
  int orcExecAct(const char* rtname, const char *rtparams, int req_id) {
    RobotTask* RT = ( RobotTask* ) GetRTFromName((char*)rtname);
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
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
//    signal(SIGPIPE, SIG_IGN);
		
    tcComm = new CommTcServer( TC_SERVER_PORT_NBR); 
  
    tmComm = new CommTmServer( TM_SERVER_PORT_NBR);
  
    tcReplyServer =  new CommTcReplyServer( TC_REPLY_SERVER_PORT_NBR );

    RobotTask* RT1 = new RobotTask ("RT1");
    RobotTasks.push_back( RT1 ); 
    RobotTask* RT2 = new RobotTask ("RT2");
    RobotTasks.push_back( RT2 );
    
    // telecommand=NULL;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    

    //! Check list of telecommands
    /*
    while (tcComm->GetTelecommandFromList(&telecommand) != 0)
    {
        if (telecommand.subsystem == LOCOMOTION) // Motion command
        */
        {
            motion_command.translation = 0.05; // telecommand.param1;
            motion_command.rotation = 0.0; // telecommand.param2;
            _locomotion_command.write(motion_command);
        }
        /*
        else if (telecommand.subsystem == PERCEPTION) // camera image acquisition
        {
            switch (telecommand.param1)
            {
            case NAV_CAM:
                break;
            case LOC_CAM:
                break;
            case PAN_IMAGE:
                break;
            }
        }
        else if (telecommand.subsystem == MAST)
        */
        {
            ptu_command[0].position=1.0;
            ptu_command[0].speed=base::NaN<float>();
            ptu_command[1].position=0.0;
            ptu_command[1].speed=base::NaN<float>();
            _ptu_command.write(ptu_command);
        }
        /*
        else if (telecommand.subsystem == SOLAR_ARRAY)
        {

        }
    }
    */
    //! Send telemetry data
    if (_current_pose.read(pose) == RTT::NewData)
    {
        std::cout << "received position: " << pose.position[0] << std::endl;
        // generate TM packet with updated pose estimation
    }
    // generate rest of TM packet
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
