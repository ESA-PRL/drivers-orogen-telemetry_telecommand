#include "Task.hpp"

const unsigned int TC_SERVER_PORT_NUMBER = 7031;
const unsigned int TM_SERVER_PORT_NUMBER = 7032;
const unsigned int TC_REPLY_SERVER_PORT_NUMBER = 7033;

const double DEG2RAD = M_PI/180;
const double RAD2DEG = 180/M_PI;

const double MIN_ACK_RADIUS = 0.6;
const double OMEGA = 0.04;                      //in Rad/s the commanded angular velocity to the walking actuators when deploying

//const double PANLIMIT_LEFT = 155*DEG2RAD;       //HDPR
//const double PANLIMIT_RIGHT = -155*DEG2RAD;     //HDPR
//const double TILTLIMIT_LOW = -25*DEG2RAD;       //HDPR
//const double TILTLIMIT_HIGH= 45*DEG2RAD;        //HDPR

const double PANLIMIT_LEFT = 90*DEG2RAD;       //ExoTeR
const double PANLIMIT_RIGHT = -180*DEG2RAD;     //ExoTeR
const double TILTLIMIT_LOW = -90*DEG2RAD;       //ExoTeR
const double TILTLIMIT_HIGH= 90*DEG2RAD;        //ExoTeR

const double DEPLOYMENTLIMIT = 110;              //ExoTeR
const double TARGET_WINDOW = 0.01;              //ExoTeR
const double TARGET_WINDOW2 = 0.01;             //ExoTeR
const double TARGET_WINDOW3 = 2.0;              //ExoTeR

using namespace telemetry_telecommand;
using namespace locomotion_switcher;

RobotProcedure*  theRobotProcedure;
ActiveMQTCReceiver* activemqTCReceiver;

RobotTask* GetRTFromName (char* rtname)
{
    RobotTask* RT = ( RobotTask* ) theRobotProcedure->GetRTFromName((char*)rtname);
    return RT;
}

extern "C"
{
    int orcExecAct(const char* rtname, const char *rtparams, int req_id)
    {
        RobotTask* RT = ( RobotTask* ) theRobotProcedure->GetRTFromName((char*)rtname);
        if (RT != NULL)
        {
            RT->SetParam((char *)rtparams);
            RT->SetTcRequestId(req_id);
            RT->Control();

            char tc_reply[80];
            sprintf(tc_reply, "%d 2 RspAck\n", RT->GetTcRequestId());
        }
        return 0;
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

void Task::deadManSwitch()
{
    lastDirectCommandTime = base::Time::now();
    deadMan = false;
    deadManSwitchRelevant = true;
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    ptu_command.resize(2);
    ptu_command.names[0]= "MAST_PAN";
    ptu_command.names[1]= "MAST_TILT";
    ptu.resize(2);
    bema.resize(6);
    joint_samples.resize(14);
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
    FHAZ_STEREO_index = 1;
    TOF_index = 1;
    LIDAR_index = 1;

    initial_3Dpose=_initial_pose;
    absolute_pose=initial_3Dpose;
    initial_absolute_heading=initial_3Dpose.getYaw();

    MastDeployed = _isMastDeployed.get();
    rover = _rover.get();
    LOG_DEBUG_S << "Configuring Rover: " << rover;

    std::function<bool(void)> trueFn = [](){return true;};

    // map telecommand strings to the corresponding enum and function
    tc_map = {
        { "GNC_AUTONAV_GOTO",     std::make_tuple( -1, 200, std::bind( &Task::exec_GNC_AUTONAV_GOTO,     this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_AUTONAV_GOTO,     this ) ) },
        { "GNC_ACKERMANN_GOTO",   std::make_tuple( -1, 200, std::bind( &Task::exec_GNC_ACKERMANN_GOTO,   this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_ACKERMANN_GOTO,   this ) ) },
        { "GNC_WHEELWALK_GOTO",   std::make_tuple( -1, 200, std::bind( &Task::exec_GNC_WHEELWALK_GOTO,   this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_WHEELWALK_GOTO,   this ) ) },
        { "GNC_LLO",              std::make_tuple( -1, 200, std::bind( &Task::exec_GNC_LLO,              this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_LLO,              this ) ) },
        { "GNC_TURNSPOT_GOTO",    std::make_tuple( -1, 200, std::bind( &Task::exec_GNC_TURNSPOT_GOTO,    this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_TURNSPOT_GOTO,    this ) ) },
        { "GNC_TRAJECTORY",       std::make_tuple( -1, 600, std::bind( &Task::exec_GNC_TRAJECTORY,       this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_TRAJECTORY,       this ) ) },
        { "GNC_TRAJECTORY_WISDOM",std::make_tuple( -1, 600, std::bind( &Task::exec_GNC_TRAJECTORY_WISDOM,this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_TRAJECTORY_WISDOM,this ) ) },
        { "MAST_PTU_MoveTo",      std::make_tuple( -1, 300, std::bind( &Task::exec_MAST_PTU_MOVE_TO,     this, std::placeholders::_1), std::bind( &Task::ctrl_MAST_PTU_MOVE_TO,     this ) ) },
        { "Deploy_Mast",          std::make_tuple( -1, 50, std::bind( &Task::exec_DEPLOY_MAST,           this, std::placeholders::_1), trueFn ) },
        { "PANCAM_PANORAMA",      std::make_tuple( -1, 500, std::bind( &Task::exec_PANCAM_PANORAMA,      this, std::placeholders::_1), std::bind( &Task::ctrl_PANCAM_PANORAMA,      this ) ) },
        { "TOF_ACQ",              std::make_tuple( -1, 200, std::bind( &Task::exec_TOF_ACQ,              this, std::placeholders::_1), std::bind( &Task::ctrl_TOF_ACQ,              this ) ) },
        { "LIDAR_ACQ",            std::make_tuple( -1, 200, std::bind( &Task::exec_LIDAR_ACQ,            this, std::placeholders::_1), std::bind( &Task::ctrl_LIDAR_ACQ,            this ) ) },
        { "FRONT_ACQ",            std::make_tuple( -1, 200, std::bind( &Task::exec_FRONT_ACQ,            this, std::placeholders::_1), std::bind( &Task::ctrl_FRONT_ACQ,            this ) ) },
        { "NAVCAM_ACQ",           std::make_tuple( -1, 200, std::bind( &Task::exec_NAVCAM_ACQ,           this, std::placeholders::_1), std::bind( &Task::ctrl_NAVCAM_ACQ,           this ) ) },
        { "MAST_ACQ",             std::make_tuple( -1, 200, std::bind( &Task::exec_MAST_ACQ,             this, std::placeholders::_1), std::bind( &Task::ctrl_MAST_ACQ,             this ) ) },
        { "PANCAM_ACQ",           std::make_tuple( -1, 200, std::bind( &Task::exec_PANCAM_ACQ,           this, std::placeholders::_1), std::bind( &Task::ctrl_PANCAM_ACQ,           this ) ) },
        { "REAR_ACQ",             std::make_tuple( -1, 200, std::bind( &Task::exec_REAR_ACQ,             this, std::placeholders::_1), std::bind( &Task::ctrl_REAR_ACQ,             this ) ) },
        { "HAZCAM_ACQ",           std::make_tuple( -1, 200, std::bind( &Task::exec_HAZCAM_ACQ,           this, std::placeholders::_1), std::bind( &Task::ctrl_HAZCAM_ACQ,           this ) ) },
        { "LOCCAM_ACQ",           std::make_tuple( -1, 200, std::bind( &Task::exec_LOCCAM_ACQ,           this, std::placeholders::_1), std::bind( &Task::ctrl_LOCCAM_ACQ,           this ) ) },
        { "Deployment_All",       std::make_tuple( -1, 500, std::bind( &Task::exec_DEPLOYMENT_ALL,      this, std::placeholders::_1), std::bind( &Task::ctrl_DEPLOYMENT_ALL,       this ) ) },
        { "Deployment_Front",     std::make_tuple( -1, 500, std::bind( &Task::exec_DEPLOYMENT_FRONT,    this, std::placeholders::_1), std::bind( &Task::ctrl_DEPLOYMENT_FRONT,     this ) ) },
        { "Deployment_Rear",      std::make_tuple( -1, 500, std::bind( &Task::exec_DEPLOYMENT_REAR,     this, std::placeholders::_1), std::bind( &Task::ctrl_DEPLOYMENT_REAR,      this ) ) },
        { "GNC_Update",           std::make_tuple( -1, 50, std::bind( &Task::exec_GNC_UPDATE,           this, std::placeholders::_1), trueFn ) },
        { "GNC_ACKERMANN_DIRECT", std::make_tuple( -1, 50, std::bind( &Task::exec_GNC_ACKERMANN_DIRECT, this, std::placeholders::_1), trueFn ) },
        { "GNC_TURNSPOT_DIRECT",  std::make_tuple( -1, 50, std::bind( &Task::exec_GNC_TURNSPOT_DIRECT,  this, std::placeholders::_1), trueFn ) },
        { "ALL_ACQ",              std::make_tuple( -1, 50, std::bind( &Task::exec_ALL_ACQ,              this, std::placeholders::_1), std::bind( &Task::ctrl_ALL_ACQ,              this ) ) },
        { "GNCG",                 std::make_tuple( -1, 50, std::bind( &Task::exec_GNCG,                 this, std::placeholders::_1), trueFn ) },
        { "Activity_Plan",        std::make_tuple( -1, 50, std::bind( &Task::exec_ACTIVITY_PLAN,        this, std::placeholders::_1), trueFn ) },
        { "GET_NEW_HK_TM",        std::make_tuple( -1, 50, std::bind( &Task::exec_GET_NEW_HK_TM,        this, std::placeholders::_1), trueFn ) },
        { "ABORT",                std::make_tuple( -1, 50, std::bind( &Task::exec_ABORT,                this, std::placeholders::_1), trueFn ) }
    };

    // we're not waiting for these products without receiving the command first
    sent_image_left = true;
    sent_image_right = true;
    sent_distance_image = true;
    sent_point_cloud = true;
    sent_dem = true;

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    taskPeriod = TaskContext::getPeriod();

    activemq::library::ActiveMQCPP::initializeLibrary();
    bool useTopics = true;
    bool sessionTransacted = false;
    int numMessages = 2000;

    std::string brokerURI = _activeMQ_brokerURI.value();
    //std::string brokerURI = "failover:(tcp://192.168.200.241:9009)";

    activemqTMSender = new ActiveMQTMSender(brokerURI, numMessages, useTopics, false, "");
    activemqAdmin = new ActiveMQAdmin(brokerURI, numMessages, useTopics);
    activemqTCReceiver = new ActiveMQTCReceiver(brokerURI, numMessages, useTopics, sessionTransacted);

    theRobotProcedure = new RobotProcedure("exoter");
    tcComm = new CommTcServer( TC_SERVER_PORT_NUMBER);
    tmComm = new CommTmServer( TM_SERVER_PORT_NUMBER, theRobotProcedure, activemqTMSender,rover);
    tcReplyServer =  new CommTcReplyServer( TC_REPLY_SERVER_PORT_NUMBER );

    theRobotProcedure->insertRT(new RobotTask("Deploy_Mast"));                  //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_DEP_Initialise"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_DEP_MoveTo"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_DEP_SwitchOff"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_PAN_Initialise"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_PAN_MoveTo"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_PAN_SwitchOff"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_PTU_MoveTo"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_TILT_Initialise"));         //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_TILT_MoveTo"));             //Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_TILT_SwitchOff"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_HighPwr2Reduced"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_LowPwr2Reduced"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_Nominal2Reduced"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_Reduced2HighPwr"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_Reduced2LowPwr"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_Reduced2Nominal"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("COMMS_SwitchOn"));               //Simulated
    theRobotProcedure->insertRT(new RobotTask("COMMS_LST2WH"));                 //Simulated
    theRobotProcedure->insertRT(new RobotTask("COMMS_WH2LST"));                 //Simulated
    theRobotProcedure->insertRT(new RobotTask("COMMS_SwitchOff"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("Deploy_LEFT_SA"));               //Simulated
    theRobotProcedure->insertRT(new RobotTask("Deploy_RIGHT_SA"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Primary_Initialise"));   //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Primary_MoveTo"));       //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Primary_SwitchOff"));    //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Secondary_Initialise")); //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Secondary_MoveTo"));     //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Secondary_SwitchOff"));  //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Primary_Initialise"));  //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Primary_MoveTo"));      //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Primary_SwitchOff"));   //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Secondary_Initialise"));//Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Secondary_MoveTo"));    //Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Secondary_SwitchOff")); //Simulated
    theRobotProcedure->insertRT(new RobotTask("ADEs_Activate"));                //Simulated
    theRobotProcedure->insertRT(new RobotTask("ADEs_DeActivate"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_Operational2Standby"));      //Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_Standby2Operational"));      //Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_SwitchOn"));                 //Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_SwitchOff"));                //Simulated
    theRobotProcedure->insertRT(new RobotTask("GNC_Initialise"));               //Simulated
    theRobotProcedure->insertRT(new RobotTask("GNC_MonitoringOnly"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("GNC_SwitchOff"));                //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_Initialise"));            //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_InitWACs"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_SwitchOn"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_WACAcqImage"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_WACGetImage"));           //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_SwitchOff"));             //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_PIUSwitchOff"));          //Simulated
    theRobotProcedure->insertRT(new RobotTask("PANCAM_WAC_RRGB"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_FilterSel"));             //Simulated
    theRobotProcedure->insertRT(new RobotTask("BEMA_Deploy_1"));                //Simulated
    theRobotProcedure->insertRT(new RobotTask("BEMA_Deploy_2"));                //Simulated
    theRobotProcedure->insertRT(new RobotTask("Release_Umbilical"));            //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_WakeUp"));                    //Simulated
    theRobotProcedure->insertRT(new RobotTask("MMS_WaitAbsTime"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("MMS_WaitRelTime"));              //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Comms"));             //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_SwitchOffMobility"));         //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_PostComms"));                 //Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_Go2Nominal"));               //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Travel"));            //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Night"));             //Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Dozing"));            //Simulated



    theRobotProcedure->insertRT(new RobotTask("MAST_ACQ"));                 // Executed (params WAC_L, WAC_R)
    theRobotProcedure->insertRT(new RobotTask("PANCAM_ACQ"));               // Executed (params WAC_L, WAC_R)
    theRobotProcedure->insertRT(new RobotTask("HAZCAM_ACQ"));               // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("LOCCAM_ACQ"));               // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("LIDAR_ACQ"));                // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("TOF_ACQ"));                  // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("PANCAM_PANORAMA"));          // Executed (params tilt angle in deg)
    theRobotProcedure->insertRT(new RobotTask("NAVCAM_ACQ"));               // Executed
    theRobotProcedure->insertRT(new RobotTask("FRONT_ACQ"));                // Executed
    theRobotProcedure->insertRT(new RobotTask("REAR_ACQ"));                 // Executed
    theRobotProcedure->insertRT(new RobotTask("ALL_ACQ"));                  // Executed
    theRobotProcedure->insertRT(new RobotTask("MAST_PTU_MoveTo"));          // Executed (params: pan, tilt (deg, deg))
    theRobotProcedure->insertRT(new RobotTask("GNC_Update"));               // Executed  (params: x,y,z in meters rx,ry,rz in degrees)
    theRobotProcedure->insertRT(new RobotTask("GNCG"));                     // Executed  (params: fixed ActivityPlan.txt)
    theRobotProcedure->insertRT(new RobotTask("Activity_Plan"));            // Executed  (params: ActivityPlan file name)
    theRobotProcedure->insertRT(new RobotTask("GET_NEW_HK_TM"));            // Executed  (params: )
    theRobotProcedure->insertRT(new RobotTask("GNC_AUTONAV_GOTO"));         // Executed  (params: x,y global coordinates (m, m))
    theRobotProcedure->insertRT(new RobotTask("GNC_ACKERMANN_GOTO"));       // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_WHEELWALK_GOTO"));       // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_LLO"));                  // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_TURNSPOT_GOTO"));        // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_ACKERMANN_DIRECT"));     // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_TURNSPOT_DIRECT"));      // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_TRAJECTORY"));           // Executed  (params: number of waypoints, vector of waypoints(x,y,h))
    theRobotProcedure->insertRT(new RobotTask("GNC_TRAJECTORY_WISDOM"));    // Executed  (params: number of waypoints, vector of waypoints(x,y,h))
    theRobotProcedure->insertRT(new RobotTask("Deployment_All"));           // Executed  (params: deploy angle in deg)
    theRobotProcedure->insertRT(new RobotTask("Deployment_Front"));         // Executed  (params: deploy angle in deg)
    theRobotProcedure->insertRT(new RobotTask("Deployment_Rear"));          // Executed  (params: deploy angle in deg)
    theRobotProcedure->insertRT(new RobotTask("ABORT"));                    // Executed

    //! ToDo: Check if this fix is still necessary. It might be fixed at ptu_control component configure/start hook.
    //! Send ptu and motion commands to activate the joint dispatcher. Otherwise stays waiting.
    locomotion_mode = LocomotionMode::DRIVING;
    pan = 0.0; tilt = 0.0; sendPtuCommand();
    targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();

    WisdomDistance = 0.0;
    WisdomAcqTime = 0;
    WisdomTimer = 0;
    WisdomAcquiring = false;

    /**
     * Routine to send the bema command to the rover stowed position (beggining of Egress in ExoTeR)
     */
    /*
    bema_command=-90.0;
    LOG_INFO_S <<  "Deployment All: " << bema_command;
    bema_command = bema_command*DEG2RAD;
    locomotion_mode = LocomotionMode::DEPLOYMENT;
    _locomotion_mode.write(locomotion_mode);
    _bema_command.write(-2.0*OMEGA);
    setTimeout("Deployment_All", 600);
    */
    // target reached is only false when we are in trajectory_following activity
    // without having reached the target
    target_reached = true;

    abort_activity=false;
    files_sent=false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    checkDeadManSwitch();
    reactToInputPorts();
    getAndExecTelecommand();
    controlRunningActivities();
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
    return sqrt((pose.position[0]-initial_pose.position[0])*(pose.position[0]-initial_pose.position[0])
            +(pose.position[1]-initial_pose.position[1])*(pose.position[1]-initial_pose.position[1])
            +(pose.position[2]-initial_pose.position[2])*(pose.position[2]-initial_pose.position[2]));
}

bool Task::angleReached()
{
    double angle = std::abs(pose.getYaw()*RAD2DEG-targetOrientationTheta);
    return angle < TARGET_WINDOW3;
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

bool Task::ptuTargetReached()
{
    if (abort_activity)
    {
        abort_activity=false;
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
    LOG_INFO_S << "PTU target reached!";
    return true;
}

bool Task::bema1TargetReached()
{
    if (abort_activity)
    {
        abort_activity=false;
        targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
        return true;
    }
    double window = TARGET_WINDOW2;
    for (int i=0;i<6;i++)
    {
        LOG_DEBUG_S << "bema_command: " << bema_command << " bema[].position: " << bema[i].position;
        if (std::abs(bema[i].position-bema_command) < window)
        {
            LOG_INFO_S << "BEMA1 target reached!";
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
        abort_activity=false;
        targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
        return true;
    }
    double window = TARGET_WINDOW2;
    for (int i=0;i<2;i++)
    {
        LOG_DEBUG_S << "bema_command: " << bema_command << " bema[].position: " << bema[i].position;
        if (std::abs(bema[i].position-bema_command) < window)
        {
             LOG_INFO_S << "BEMA2 target reached!";
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
        abort_activity=false;
        targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
        return true;
    }
    double window = TARGET_WINDOW2;
    for (int i=0;i<2;i++)
    {
        LOG_DEBUG_S << "bema_command: " << bema_command << " bema[].position: " << bema[4+i].position;
        if (std::abs(bema[4+i].position-bema_command) < window)
        {
            LOG_INFO_S << "BEMA3 target reached!";
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
    if (tilt > TILTLIMIT_HIGH)
        tilt = TILTLIMIT_HIGH;
    else if (tilt < TILTLIMIT_LOW)
        tilt = TILTLIMIT_LOW;
    _mast_pan.write(pan);
    if (rover == HDPR)
        _mast_tilt.write(tilt*4);
    if (rover == ExoTeR)
        _mast_tilt.write(tilt);
}

void Task::sendMotionCommand()
{
    _locomotion_mode.write(locomotion_mode);
    motion_command.translation = targetTranslation;
    motion_command.rotation = targetRotation;
    _locomotion_command.write(motion_command);
}

void Task::sendProduct(messages::Telemetry tm)
{
    const int seq = 1;  // sequence number is not necessary any more.
                        // TODO adjust send..() signature
    switch (tm.productSource)
    {
        case messages::Producer::PANCAM:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Pancam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            int res = tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgPancamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size << " and result " << res;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_LEFT:
                    {
                        LOG_INFO_S << "Sending stereo left image from Pancam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgPancamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent stereo left image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_RIGHT:
                    {
                        LOG_INFO_S << "Sending stereo right image from Pancam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_right_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgPancamRightProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_right = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Pancam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distPancamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Pancam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcPancamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Pancam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demPancamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::MAST:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Mast " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgMastProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Mast " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distMastProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Mast " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcMastProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Mast " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_pancam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demMastProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::LIDAR:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Lidar " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_lidar2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgLidarProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Lidar " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_lidar2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distLidarProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Lidar " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_lidar2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcLidarProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Lidar " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_lidar2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demLidarProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::NAVCAM:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Navcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (MastDeployed)
                        {
                            if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        else
                        {
                            if (_left_camera_navcam_back2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgNavcamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_LEFT:
                    {
                        LOG_INFO_S << "Sending stereo left image from Navcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (MastDeployed)
                        {
                            if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        else
                        {
                            if (_left_camera_navcam_back2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgNavcamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_RIGHT:
                    {
                        LOG_INFO_S << "Sending stereo right image from Navcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (MastDeployed)
                        {
                            if (_right_camera_navcam2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        else
                        {
                            if (_right_camera_navcam_back2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgNavcamRightProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_right = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (MastDeployed)
                        {
                            if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        else
                        {
                            if (_left_camera_navcam_back2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distNavcamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (MastDeployed)
                        {
                            if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        else
                        {
                            if (_left_camera_navcam_back2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcNavcamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Navcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (MastDeployed)
                        {
                            if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        else
                        {
                            if (_left_camera_navcam_back2lab.get(tm.timestamp, tf, false))
                            {
                                getTransform(tf);
                            }
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demNavcamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::FRONT:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Front " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb32lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgFrontProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Front " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb32lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distFrontProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Front " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb32lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcFrontProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Front " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb32lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demFrontProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::TOF:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Tof " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_tof2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgTofProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Tof " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_tof2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distTofProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Tof " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_tof2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcTofProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Tof " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_tof2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demTofProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::LOCCAM:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_loccam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgLoccamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_LEFT:
                    {
                        LOG_INFO_S << "Sending stereo left image from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_loccam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgLoccamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_RIGHT:
                    {
                        LOG_INFO_S << "Sending stereo right image from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_right_camera_loccam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgLoccamRightProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_right = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_loccam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distLoccamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_loccam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcLoccamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Loccam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_loccam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demLoccamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::HAZCAM:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Hazcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgHazcamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_LEFT:
                    {
                        LOG_INFO_S << "Sending stereo left from Hazcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgHazcamLeftProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent stereo left with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::STEREO_RIGHT:
                    {
                        LOG_INFO_S << "Sending stereo right from Hazcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_right_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgHazcamRightProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent stereo right with size " << size;
                        }
                        sent_image_right = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Hazcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distHazcamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Hazcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcHazcamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Hazcam " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demHazcamProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::REAR:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Rear " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgRearProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        sent_image_left = true;
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Rear " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distRearProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        sent_distance_image = true;
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        LOG_INFO_S << "Sending point cloud from Rear " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(tm.productPath.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->pcRearProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent point cloud file with size " << data.size();
                        }
                        sent_point_cloud = true;
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Rear " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> fileContents((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        std::vector<unsigned char> data =  std::vector<unsigned char>(fileContents.begin(), fileContents.end());
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        std::string filename = tm.productPath;
                        filename.replace(0, 21, "");
                        Eigen::Affine3d tf;
                        if (_left_camera_bb22lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendDEMMessage(filename.c_str(), seq, time, date.c_str(), data.size(), data, activemqTMSender->demRearProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << data.size();
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        LOG_INFO_S << "Sending mtl file " << mtl_filename;
                        char command[256];
                        std::string folder = _productsFolder.value();
                        sprintf(command,  "sed -ie 's/%s//g' %s", folder.c_str(), mtl_filename.c_str());
                        system(command);
                        std::ifstream input2(mtl_filename.c_str(), std::ios::binary);
                        std::vector<char> buffer2((std::istreambuf_iterator<char>(input2)), (std::istreambuf_iterator<char>()));
                        auto size2 = buffer2.size();
                        char* data2 = &buffer2[0];
                        mtl_filename.replace(0, 21, "");
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendFileMessage(mtl_filename.c_str(), size2, (const unsigned char *)data2, activemqTMSender->fileProducerMonitoring);
                            LOG_INFO_S << "Sent mtl file with size " << size2;
                        }
                        sent_dem = true;
                        break;
                    }
                default:
                    break;
            }
            break;
        case messages::Producer::AUTONAV:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        LOG_INFO_S << "Sending image from Autonav " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time = tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath = date + "_AUTONAV_IMAGE.png";
                        Eigen::Affine3d tf;
                        if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgAutonavProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent image with size " << size;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        LOG_INFO_S << "Sending distance from Autonav " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath = date + "_AUTONAV_DISPARITY.png";
                        Eigen::Affine3d tf;
                        if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->distAutonavProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent distance file with size " << size;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        LOG_INFO_S << "Sending dem from Autonav " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath = date + "_AUTONAV_DEM.png";
                        Eigen::Affine3d tf;
                        if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->demAutonavProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent dem with size " << size;
                        }
                        break;
                    }
                case messages::ProductType::PARTIAL_NAVMAP:
                    {
                        LOG_INFO_S << "Sending mavmap from Autonav " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath = date + "_AUTONAV_PARTIAL_NAVMAP.png";
                        Eigen::Affine3d tf;
                        if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->partialNavmapAutonavProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent partial navmap with size " << size;
                        }
                        break;
                    }
                case messages::ProductType::NAVMAP:
                    {
                        LOG_INFO_S << "Sending mavmap from Autonav " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath = date + "_AUTONAV_NAVMAP.png";
                        Eigen::Affine3d tf;
                        if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->navmapAutonavProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent navmap with size " << size;
                        }
                        break;
                    }
                case messages::ProductType::TRAJMAP:
                    {
                        LOG_INFO_S << "Sending trajmap from Autonav " << tm.productPath;
                        std::ifstream input(tm.productPath.c_str(), std::ios::binary);
                        std::vector<char> buffer((std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
                        auto size = buffer.size();
                        char* data = &buffer[0];
                        long time=tm.timestamp.toMilliseconds();
                        std::string date = tm.timestamp.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_");
                        date.erase(std::remove(date.begin(),date.end(), ':' ), date.end() ) ;
                        tm.productPath = date + "_AUTONAV_TRAJMAP.png";
                        Eigen::Affine3d tf;
                        if (_left_camera_navcam2lab.get(tm.timestamp, tf, false))
                        {
                            getTransform(tf);
                        }
                        if (activemqTMSender->isConnected)
                        {
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->trajmapAutonavProducerMonitoring, transformation);
                            LOG_INFO_S << "Sent trajmap with size " << size;
                        }
                        break;
                    }
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void Task::exec_GNC_AUTONAV_GOTO(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf %lf %lf %lf %d %d", &targetPositionX, &targetPositionY, &targetOrientationTheta, &max_obstacle, &max_slope, &cold_start, &timeout_motions);
    LOG_INFO_S <<  "GNC_AUTONAV_GOTO X:" << targetPositionX << ", Y:" << targetPositionY << ", Heading: " << targetOrientationTheta;
    base::Waypoint goal_pose2D(base::Position(targetPositionX,targetPositionY,0),targetOrientationTheta,0,0);
    _autonav_obstacle.write(max_obstacle);
    _autonav_slope.write(max_slope);
    _autonav_coldstart.write((bool)cold_start);
    _autonav_goal.write(goal_pose2D);
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_ACKERMANN_GOTO(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf %lf %d", &targetPositionX, &targetPositionY, &targetSpeed, &timeout_motions);
    // Calculate the parameters to be sent as motion commands (2D): Translation (m/s) and Rotation (rad/s)
    if (targetPositionY == 0) // Straight line command
    {
        targetDistance = std::abs(targetPositionX);
        double sign = (targetPositionX < 0 ? -1 : 1);
        targetTranslation = targetSpeed*sign;
        targetRotation = 0;
    }
    else
    {
    double radius = (targetPositionX*targetPositionX + targetPositionY*targetPositionY)/(2*targetPositionY);
    if (std::abs(radius)<MIN_ACK_RADIUS)
    {
        LOG_INFO_S << "Telemetry_Telecommand: Aborting Ackerman activity. Radius of curvature too small. Try Point Turn first.";
        targetTranslation=0.0;
        targetRotation=0.0;
        setTimeout(cmd_info->activityName, 0);
        return;
    }
    double theta = atan(targetPositionX/std::abs(radius-targetPositionY));
    double sign = (targetPositionX < 0 ? -1 : 1);
    targetTranslation = targetSpeed*sign;
    targetRotation = targetTranslation/radius;
    targetDistance = std::abs(theta*radius);
    }
    travelledDistance = 0.0;
    initial_pose = pose;
    LOG_INFO_S <<  "GNC_ACKERMANN_GOTO X:" << targetPositionX << " Y:" << targetPositionY << " speed:" << targetSpeed;
    locomotion_mode = LocomotionMode::DRIVING;
    sendMotionCommand();
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_WHEELWALK_GOTO(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %d", &targetPositionX, &timeout_motions);
    targetDistance = std::abs(targetPositionX);
    travelledDistance = 0.0;
    initial_pose = pose;
    LOG_INFO_S <<  "GNC_WHEELWALK_GOTO PositionX:" << targetPositionX;
    locomotion_mode = LocomotionMode::WHEEL_WALKING;
    targetTranslation = targetDistance; // just needs to be non zero
    targetRotation = 0;                 // any value
    sendMotionCommand();
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_LLO(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf %d", &targetPositionX, &targetSpeed, &timeout_motions); 
    // Calculate the parameters to be sent as motion commands (2D): Translation (m/s) and Rotation (rad/s)
    targetDistance = std::abs(targetPositionX);
    double sign = (targetPositionX < 0 ? -1 : 1);
    targetTranslation = targetSpeed*sign;
    targetRotation = 0;
    //travelledDistance = 0.0;
    //initial_pose = pose;
    LOG_INFO_S <<  "GNC_LLO Distance:" << targetPositionX << " speed:" << targetSpeed;
    locomotion_mode = LocomotionMode::DRIVING;
    sendMotionCommand();
    setTimeout(cmd_info->activityName, (int)(targetDistance/targetSpeed/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_TURNSPOT_GOTO(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf %d", &targetOrientationTheta, &targetRotation, &timeout_motions);
    targetRotation *= DEG2RAD;
    initial_imu = pose;

    // Calculate the parameters to be sent as motion commands (2D): Translation (m/s) and Rotation (rad/s)
    targetTranslation=0.0;
    if (targetOrientationTheta>=0)
    {
        targetOrientationTheta+=(initial_imu.getYaw()*RAD2DEG);
        if (targetOrientationTheta>180.0)
        {
            targetOrientationTheta-=360.0;
        }
    }
    else
    {
        targetRotation=-targetRotation;
        targetOrientationTheta+=(initial_imu.getYaw()*RAD2DEG);
        if (targetOrientationTheta<-180.0)
        {
            targetOrientationTheta+=360.0;
        }
    }

    travelledAngle = 0.0;
    LOG_INFO_S <<  "GNC_TURNSPOT_GOTO angle:" << targetOrientationTheta;
    locomotion_mode = LocomotionMode::DRIVING;
    sendMotionCommand();
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_TRAJECTORY(CommandInfo* cmd_info)
{
    target_reached=false;
    char *token_str = strtok((char *)(cmd_info->activityParams.c_str()), " ");
    token_str = strtok(NULL, " ");
    int NofWaypoints = atoi(token_str);
    LOG_INFO_S << "NofWaypoints:" << NofWaypoints << "<-";
    for (int i=0;i<NofWaypoints;i++)
    {
        token_str = strtok(NULL, " ");
        waypoint.position(0)=atof(token_str);
        LOG_INFO_S << "Point " << i+1 << " x:" << waypoint.position(0);
        token_str = strtok(NULL, " ");
        waypoint.position(1)=atof(token_str);
        LOG_INFO_S << "Point " << i+1 << " y:" << waypoint.position(1);
        trajectory.push_back(waypoint);
    }
    token_str = strtok(NULL, " ");
    targetOrientationTheta = atof(token_str);
    LOG_INFO_S << "targetOrientationTheta: " << targetOrientationTheta;
    trajectory.back().heading = targetOrientationTheta*DEG2RAD;
    token_str = strtok(NULL, " ");
    targetSpeed = atof(token_str);
    token_str = strtok(NULL, " ");
    timeout_motions = atof(token_str);
    locomotion_mode = LocomotionMode::DRIVING;
    _locomotion_mode.write(locomotion_mode);
    _trajectory.write(trajectory);
    if (targetSpeed>0)
    {
        _trajectory_speed.write(targetSpeed);
    }
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    LOG_INFO_S <<  "GNC_TRAJECTORY #ofWaypoints:" << NofWaypoints;
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_TRAJECTORY_WISDOM(CommandInfo* cmd_info)
{
    target_reached=false;
    char *token_str = strtok((char *)(cmd_info->activityParams.c_str()), " ");
    token_str = strtok(NULL, " ");
    int NofWaypoints = atoi(token_str);
    LOG_INFO_S << "NofWaypoints:" << NofWaypoints << "<-";
    for (int i=0;i<NofWaypoints;i++)
    {
        token_str = strtok(NULL, " ");
        waypoint.position(0)=atof(token_str);
        LOG_INFO_S << "Point " << i+1 << " x:" << waypoint.position(0);
        token_str = strtok(NULL, " ");
        waypoint.position(1)=atof(token_str);
        LOG_INFO_S << "Point " << i+1 << " y:" << waypoint.position(1);
        trajectory.push_back(waypoint);
    }
    token_str = strtok(NULL, " ");
    targetOrientationTheta = atof(token_str);
    LOG_INFO_S << "targetOrientationTheta: " << targetOrientationTheta;
    trajectory.back().heading = targetOrientationTheta*DEG2RAD;
    token_str = strtok(NULL, " ");
    targetSpeed = atof(token_str);
    LOG_INFO_S << "targetSpeed: " << targetSpeed;
    token_str = strtok(NULL, " ");
    WisdomDistance = atof(token_str);
    LOG_INFO_S << "WisdomDistance: " << WisdomDistance;
    token_str = strtok(NULL, " ");
    WisdomTimer = atof(token_str);
    LOG_INFO_S << "WisdomTimer: " << WisdomTimer;
    token_str = strtok(NULL, " ");
    timeout_motions = atof(token_str);
    travelledDistance = 0.0;
    initial_pose = pose;
    locomotion_mode = LocomotionMode::DRIVING;
    _locomotion_mode.write(locomotion_mode);
    _trajectory.write(trajectory);
    if (targetSpeed>0)
    {
        _trajectory_speed.write(targetSpeed);
    }
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    LOG_INFO_S <<  "GNC_TRAJECTORY #ofWaypoints:" << NofWaypoints;
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_MAST_PTU_MOVE_TO(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf %d", &pan, &tilt, &timeout_motions);
    LOG_INFO_S <<  "MAST_PTU_MoveTo pan:" << pan << " tilt:" << tilt;
    pan = pan*DEG2RAD;
    tilt = tilt*DEG2RAD;
    sendPtuCommand();
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
    {
        LOG_WARN_S << "Error getting MastState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
    {
        LOG_WARN_S << "Error setting MastState";
    }
}

void Task::exec_DEPLOY_MAST(CommandInfo* cmd_info)
{
    LOG_INFO_S <<  "DEPLOY_MAST: Mast Deployed.";
    MastDeployed=true;
    if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
    {
        LOG_WARN_S << "Error getting MastState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
    {
        LOG_WARN_S << "Error setting MastState";
    }
}

void Task::exec_DEPLOYMENT_ALL(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %d", &bema_command, &timeout_motions);
    if (bema_command>DEPLOYMENTLIMIT)
        bema_command=DEPLOYMENTLIMIT;
    else if (bema_command<-DEPLOYMENTLIMIT)
        bema_command=-DEPLOYMENTLIMIT;
    LOG_INFO_S <<  "Deployment All: " << bema_command;
    bema_command = bema_command*DEG2RAD;
    locomotion_mode = LocomotionMode::DEPLOYMENT;
    _locomotion_mode.write(locomotion_mode);
    if (bema_command>bema[0].position)
    {
        _bema_command.write(OMEGA);
        //targetTranslation = OMEGA;
        //targetRotation = 0.0;
        //sendMotionCommand();
    }
    else
    {
        _bema_command.write(-OMEGA);
        //targetTranslation = -OMEGA;
        //targetRotation = 0.0;
        //sendMotionCommand();

    }
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_DEPLOYMENT_FRONT(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %d", &bema_command, &timeout_motions);
    if (bema_command>DEPLOYMENTLIMIT)
        bema_command=DEPLOYMENTLIMIT;
    else if (bema_command<-DEPLOYMENTLIMIT)
        bema_command=-DEPLOYMENTLIMIT;
    LOG_INFO_S <<  "Deployment Front: " << bema_command;
    bema_command = bema_command*DEG2RAD;
    locomotion_mode = LocomotionMode::DEPLOYMENT;
    _locomotion_mode.write(locomotion_mode);
    if (bema_command>bema[0].position)
    {
        _walking_command_front.write(OMEGA);
    }
    else
    {
        _walking_command_front.write(-OMEGA);
    }
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_DEPLOYMENT_REAR(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %d", &bema_command, &timeout_motions);
    if (bema_command>DEPLOYMENTLIMIT)
        bema_command=DEPLOYMENTLIMIT;
    else if (bema_command<-DEPLOYMENTLIMIT)
        bema_command=-DEPLOYMENTLIMIT;
    LOG_INFO_S <<  "Deployment Rear: " << bema_command;
    bema_command = bema_command*DEG2RAD;
    locomotion_mode = LocomotionMode::DEPLOYMENT;
    _locomotion_mode.write(locomotion_mode);
    if (bema_command>bema[4].position)
    {
        _walking_command_rear.write(OMEGA);
    }
    else
    {
        _walking_command_rear.write(-OMEGA);
    }
    setTimeout(cmd_info->activityName, (int)(timeout_motions/taskPeriod));
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_UPDATE(CommandInfo* cmd_info)
{
    double update_pose_x;
    double update_pose_y;
    double update_pose_z;
    double update_pose_rx;
    double update_pose_ry;
    double update_pose_rz;
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf %lf %lf %lf %lf", &update_pose_x, &update_pose_y, &update_pose_z, &update_pose_rx, &update_pose_ry, &update_pose_rz);
    absolute_pose.position[0]=update_pose_x;
    absolute_pose.position[1]=update_pose_y;
    absolute_pose.position[2]=update_pose_z;
    Eigen::Quaternion <double> orientation(Eigen::AngleAxisd(update_pose_rz*DEG2RAD, Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(update_pose_ry*DEG2RAD, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(update_pose_rx*DEG2RAD, Eigen::Vector3d::UnitX()));
    absolute_pose.orientation = orientation;
    _update_pose.write(absolute_pose);
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
}

void Task::exec_GNC_ACKERMANN_DIRECT(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf %lf", &targetTranslation, &targetRotation);
    LOG_INFO_S <<  "GNC_ACKERMANN_DIRECT Translation:" << targetTranslation << " Rotation:" << targetRotation;
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        LOG_WARN_S << "Error getting GNCState";
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        LOG_WARN_S << "Error setting GNCState";
    deadManSwitch();

    // check if we are in direct or in path following mode
    if (target_reached)
    {
        // if not in trajectory following, send complete (direct) command
        locomotion_mode = LocomotionMode::DRIVING;
        sendMotionCommand();
    }
    else
    {
        // trajectory following cannot go into reverse
        // TODO don't write speeds but change PID parameters
        // Speed needs to be positive. Waypoint navigation does not accept non-positive speeds.
        if (targetTranslation >= 0) 
        {
            _trajectory_speed.write(targetTranslation);
        }
    }
}

void Task::exec_GNC_TURNSPOT_DIRECT(CommandInfo* cmd_info)
{
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf", &targetRotation);
    // TODO does the targetRotation(Speed) need to be multiplied by DEG2RAD here as well?
    targetTranslation=0.0;
    LOG_INFO_S <<  "GNC_TURNSPOT_DIRECT Rotation:" << targetRotation;
    locomotion_mode = LocomotionMode::DRIVING;
    sendMotionCommand();
    if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error getting GNCState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        LOG_WARN_S << "Error setting GNCState";
    }
    deadManSwitch();
}

void Task::exec_PANCAM_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "PanCam Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting PanCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=50;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting PanCamState";
    }

    // set flags to wait for correct products
    setProductWaitFlags(static_cast<messages::ProductType>(productType));

    _pancam_trigger.write(tc_out);
    PAN_STEREO_index++;
}


void Task::exec_MAST_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "PanCam Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting PanCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=50;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting PanCamState";
    }

    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _mast_trigger.write(tc_out);
    PAN_STEREO_index++;
}

void Task::exec_GET_NEW_HK_TM(CommandInfo* cmd_info)
{
    tmComm->sendStatesTM();
}

void Task::exec_ACTIVITY_PLAN(CommandInfo* cmd_info)
{
    char activityPlan[1024];
    sscanf(cmd_info->activityParams.c_str(), "%*d %s", activityPlan);
    LOG_INFO_S << "Activity Plan Name: " << activityPlan;
    TaskLib* taskLib = new TaskLib("");
//    taskLib->insertSol(std::string("/home/marta/rock/bundles/rover/config/orogen/ActivityPlan.txt"));
    taskLib->insertSol(activityPlan);
    taskLib->ExecuteActivityPlan();
}

void Task::exec_GNCG(CommandInfo* cmd_info)
{
//    char activityPlan[1024];
//    sscanf(cmd_info->activityParams.c_str(), "%*d %s", activityPlan);
//    LOG_INFO_S << "Activity Plan Name: " << activityPlan;
    TaskLib* taskLib = new TaskLib("");
    taskLib->insertSol(std::string("/home/marta/rock/bundles/rover/config/orogen/ActivityPlan.txt"));
//    taskLib->insertSol(activityPlan);
    taskLib->ExecuteActivityPlan();
}

void Task::exec_NAVCAM_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "LocCamFront Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=51;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _navcam_trigger.write(tc_out);
    FLOC_STEREO_index++;
}

void Task::exec_FRONT_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "LocCamFront Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=51;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _front_trigger.write(tc_out);
    FLOC_STEREO_index++;
}

void Task::exec_REAR_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "LocCamRear Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=52;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _rear_trigger.write(tc_out);
    RLOC_STEREO_index++;
}

void Task::exec_LOCCAM_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "LocCam Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _loccam_trigger.write(tc_out);
    FHAZ_STEREO_index++;
}

void Task::exec_HAZCAM_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "HazCamFront Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }

    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _haz_front_trigger.write(tc_out);
    FHAZ_STEREO_index++;
}

void Task::exec_TOF_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "Tof Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }

    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _tof_trigger.write(tc_out);
    TOF_index++;
}

void Task::exec_LIDAR_ACQ(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "Lidar Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }

    setProductWaitFlags(static_cast<messages::ProductType>(productType));
    _lidar_trigger.write(tc_out);
    LIDAR_index++;
}

void Task::exec_ALL_ACQ(CommandInfo* cmd_info)
{
    // TODO:
    // only allow execution of stop all
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %d %d", &productType, &productMode);
    tc_out.productType = static_cast<messages::ProductType>(productType);
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = static_cast<messages::Mode>(productMode);
    }
    LOG_INFO_S <<  "All sensors Get Image type " << productType << " and mode: " << productMode;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }

    _lidar_trigger.write(tc_out);
    LIDAR_index++;

    _tof_trigger.write(tc_out);
    TOF_index++;

    _haz_front_trigger.write(tc_out);
    FHAZ_STEREO_index++;

    _rear_trigger.write(tc_out);
    RLOC_STEREO_index++;

    _front_trigger.write(tc_out);
    FLOC_STEREO_index++;

    _mast_trigger.write(tc_out);
    PAN_STEREO_index++;
}

void Task::exec_PANCAM_PANORAMA(CommandInfo* cmd_info)
{
    messages::Telecommand tc_out;
    sscanf(cmd_info->activityParams.c_str(), "%*d %lf", &panorama_tilt);
    tc_out.productType = messages::ProductType::DEM;
    tc_out.productMode = messages::Mode::CONTINUOUS;
    LOG_INFO_S <<  "PanCam Panorama at tilt: " << panorama_tilt;
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting PanCamState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting PanCamState";
    }
    _pancam_360_trigger.write(tc_out);
    _panorama_tilt.write(panorama_tilt);
}

void Task::exec_ABORT(CommandInfo* cmd_info)
{
    abort_activity=true;
    LOG_INFO_S << "Abort message received!";
}

void Task::reactToInputPorts()
{
    if (_current_pose.read(pose) == RTT::NewData)
    {
        //! new TM packet with updated pose estimation
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
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
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
    }
    if (_current_bema.read(bema) == RTT::NewData)
    {
        //! new TM packet with updated bema estimation
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        int aux = (int)(bema[0].position*RAD2DEG*100);
        GNCState[GNC_ROVER_DEPLOYMENT_Q1_INDEX]=(double)((double)aux/100.0);
        aux = (int)(bema[1].position*RAD2DEG*100);
        GNCState[GNC_ROVER_DEPLOYMENT_Q2_INDEX]=(double)((double)aux/100.0);
        aux = (int)(bema[2].position*RAD2DEG*100);
        GNCState[GNC_ROVER_DEPLOYMENT_Q3_INDEX]=(double)((double)aux/100.0);
        aux = (int)(bema[3].position*RAD2DEG*100);
        GNCState[GNC_ROVER_DEPLOYMENT_Q4_INDEX]=(double)((double)aux/100.0);
        aux = (int)(bema[4].position*RAD2DEG*100);
        GNCState[GNC_ROVER_DEPLOYMENT_Q5_INDEX]=(double)((double)aux/100.0);
        aux = (int)(bema[5].position*RAD2DEG*100);
        GNCState[GNC_ROVER_DEPLOYMENT_Q6_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
    }
    if (_joint_samples.read(joint_samples) == RTT::NewData)
    {
        //! new TM packet with updated joint samples
        if ( theRobotProcedure->GetParameters()->get( (char*)"LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            LOG_WARN_S << "Error getting LOCOMState";
        }
        if (rover == HDPR)
        {
            int aux = (int)(joint_samples[0].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL1_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[1].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL2_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[2].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL3_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[3].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL4_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[4].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL5_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[5].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL6_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[6].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER1_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[7].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER2_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[8].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER5_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[9].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER6_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[0].raw*100);
            LOCOMState[GNC_ROVER_WHEEL1_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[1].raw*100);
            LOCOMState[GNC_ROVER_WHEEL2_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[2].raw*100);
            LOCOMState[GNC_ROVER_WHEEL3_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[3].raw*100);
            LOCOMState[GNC_ROVER_WHEEL4_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[4].raw*100);
            LOCOMState[GNC_ROVER_WHEEL5_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[5].raw*100);
            LOCOMState[GNC_ROVER_WHEEL6_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[6].raw*100);
            LOCOMState[GNC_ROVER_STEER1_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[7].raw*100);
            LOCOMState[GNC_ROVER_STEER2_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[8].raw*100);
            LOCOMState[GNC_ROVER_STEER5_CURRENT_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[9].raw*100);
            LOCOMState[GNC_ROVER_STEER6_CURRENT_INDEX]=(double)((double)aux/100.0);

            // joint samples for rocker and bogies are already in degrees
            // rounding values from float to closest integer. This is done by adding 0.5 to the floating value and then casting, i.e truncating, to INT
            aux = (int)(joint_samples[10].position+0.5);
            LOCOMState[GNC_ROVER_LEFT_ROCKER_INDEX]=(double)(aux);
            aux = (int)(joint_samples[11].position+0.5);
            LOCOMState[GNC_ROVER_RIGHT_ROCKER_INDEX]=(double)(aux);
            aux = (int)(joint_samples[12].position+0.5);
            LOCOMState[GNC_ROVER_LEFT_BOGIE_INDEX]=(double)(aux);
            aux = (int)(joint_samples[13].position+0.5);
            LOCOMState[GNC_ROVER_RIGHT_BOGIE_INDEX]=(double)(aux);
        }
        else if (rover == ExoTeR)
        {
            int aux = (int)(joint_samples[0].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL1_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[1].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL2_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[2].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL3_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[3].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL4_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[4].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL5_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[5].speed*RAD2DEG*100);
            LOCOMState[GNC_ROVER_WHEEL6_SPEED_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[6].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER1_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[7].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER2_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[8].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER5_POSITION_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[9].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_STEER6_POSITION_INDEX]=(double)((double)aux/100.0);

            aux = (int)(joint_samples[10].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_DEPLOYMENT_Q1_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[11].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_DEPLOYMENT_Q2_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[12].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_DEPLOYMENT_Q3_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[13].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_DEPLOYMENT_Q4_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[14].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_DEPLOYMENT_Q5_INDEX]=(double)((double)aux/100.0);
            aux = (int)(joint_samples[15].position*RAD2DEG*100);
            LOCOMState[GNC_ROVER_DEPLOYMENT_Q6_INDEX]=(double)((double)aux/100.0);

            aux = (int)(joint_samples[0].raw*10);
            LOCOMState[GNC_ROVER_WHEEL1_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[1].raw*10);
            LOCOMState[GNC_ROVER_WHEEL2_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[2].raw*10);
            LOCOMState[GNC_ROVER_WHEEL3_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[3].raw*10);
            LOCOMState[GNC_ROVER_WHEEL4_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[4].raw*10);
            LOCOMState[GNC_ROVER_WHEEL5_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[5].raw*10);
            LOCOMState[GNC_ROVER_WHEEL6_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[6].raw*10);
            LOCOMState[GNC_ROVER_STEER1_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[7].raw*10);
            LOCOMState[GNC_ROVER_STEER2_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[8].raw*10);
            LOCOMState[GNC_ROVER_STEER5_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[9].raw*10);
            LOCOMState[GNC_ROVER_STEER6_CURRENT_INDEX]=(double)((double)aux/10.0);

            aux = (int)(joint_samples[10].raw*10);
            LOCOMState[GNC_ROVER_DEPLOYMENT1_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[11].raw*10);
            LOCOMState[GNC_ROVER_DEPLOYMENT2_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[12].raw*10);
            LOCOMState[GNC_ROVER_DEPLOYMENT3_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[13].raw*10);
            LOCOMState[GNC_ROVER_DEPLOYMENT4_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[14].raw*10);
            LOCOMState[GNC_ROVER_DEPLOYMENT5_CURRENT_INDEX]=(double)((double)aux/10.0);
            aux = (int)(joint_samples[15].raw*10);
            LOCOMState[GNC_ROVER_DEPLOYMENT6_CURRENT_INDEX]=(double)((double)aux/10.0);

            // joint samples for rocker and bogies are in RAD in ExoTeR
            // rounding values from float to closest integer. This is done by adding 0.5 to the floating value and then casting, i.e truncating, to INT
            aux = (int)(joint_samples[16].position*RAD2DEG +0.5);
            LOCOMState[GNC_ROVER_LEFT_BOGIE_INDEX]=(double)(aux);
            aux = (int)(joint_samples[17].position*RAD2DEG +0.5);
            LOCOMState[GNC_ROVER_RIGHT_BOGIE_INDEX]=(double)(aux);
            aux = (int)(joint_samples[18].position*RAD2DEG +0.5);
            LOCOMState[GNC_ROVER_REAR_BOGIE_INDEX]=(double)(aux);

            LOCOMState[GNC_ROVER_LEFT_ROCKER_INDEX]=0.0;
            LOCOMState[GNC_ROVER_RIGHT_ROCKER_INDEX]=0.0;
        }
        if ( theRobotProcedure->GetParameters()->set( (char*)"LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            LOG_WARN_S << "Error setting LOCOMState";
        }
    }
    if (_motor_temperatures.read(motor_temperatures) == RTT::NewData)
    {
        //! new TM packet with updated motor temperature values
        if ( theRobotProcedure->GetParameters()->get( (char*)"LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            LOG_WARN_S << "Error getting LOCOMState";
        }
        int aux = (int)(motor_temperatures.temp[0].getCelsius()*10);
        LOCOMState[GNC_ROVER_WHEEL1_TEMPERATURE_INDEX]=(double)((double)aux/10.0);
        aux = (int)(motor_temperatures.temp[1].getCelsius()*10);
        LOCOMState[GNC_ROVER_WHEEL2_TEMPERATURE_INDEX]=(double)((double)aux/10.0);
        aux = (int)(motor_temperatures.temp[2].getCelsius()*10);
        LOCOMState[GNC_ROVER_WHEEL3_TEMPERATURE_INDEX]=(double)((double)aux/10.0);
        aux = (int)(motor_temperatures.temp[3].getCelsius()*10);
        LOCOMState[GNC_ROVER_WHEEL4_TEMPERATURE_INDEX]=(double)((double)aux/10.0);
        aux = (int)(motor_temperatures.temp[4].getCelsius()*10);
        LOCOMState[GNC_ROVER_WHEEL5_TEMPERATURE_INDEX]=(double)((double)aux/10.0);
        aux = (int)(motor_temperatures.temp[5].getCelsius()*10);
        LOCOMState[GNC_ROVER_WHEEL6_TEMPERATURE_INDEX]=(double)((double)aux/10.0);
        if ( theRobotProcedure->GetParameters()->set( (char*)"LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            LOG_WARN_S << "Error setting LOCOMState";
        }
    }
    if (_current_ptu.read(ptu) == RTT::NewData)
    {
        //! new TM packet with updated ptu position
        if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error getting MastState";
        }
        int aux = (int)(ptu[0].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q2_INDEX]=(double)((double)aux/100.0);
        aux = (int)(ptu[1].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q3_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error setting MastState";
        }
    }
    if (_current_pan.read(ptu[0].position) == RTT::NewData)
    {
        //! new TM packet with updated pan position (HDPR)
        //ptu[0].position=pan;
        if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error getting MastState";
        }
        int aux = (int)(ptu[0].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q2_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error setting MastState";
        }
    }
    if (_current_tilt.read(ptu[1].position) == RTT::NewData)
    {
        if (rover == HDPR) 
            //! new TM packet with updated tilt position (HDPR)
            ptu[1].position/=4.0; // HDPR tilt value comes with a factor of 4
        if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error getting MastState";
        }
        int aux = (int)(ptu[1].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q3_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error setting MastState";
        }
    }
    if (_current_imu.read(imu) == RTT::NewData)
    {
        //! new TM packet with updated imu pose
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        int aux = (int)(imu.getRoll()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERX_INDEX]=-(double)((double)aux/10.0);
        aux = (int)(imu.getPitch()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERY_INDEX]=-(double)((double)aux/10.0);
        // commenting out the yaw from the imu reading.
        //aux = (int)((imu.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG)*10);
        //GNCState[GNC_ROVER_POSERZ_INDEX]=(double)((double)aux/10.0);
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
    }

    if (_autonav_state.read(an_state) == RTT::NewData)
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_AUTONAV_STATUS_INDEX]=(double)(an_state);
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
    }

    if (_trajectory_status.read(tj_status) == RTT::NewData)
    {
        if (tj_status == 2)  //! TARGET_REACHED
        {
            target_reached=true;
            LOG_INFO_S << "Trajectory target reached (segment or final)";
        }
        else if (tj_status == 3) //! OUT_OF_BOUNDARIES
        {
            abort_activity=true;
            target_reached=true;
            LOG_INFO_S << "Rover out of trajectory boundaries. Aborting trajectory!";
        }
        //! Nothing to do in other status

        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_TRAJECTORY_STATUS_INDEX]=(double)(tj_status);
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
    }

    if (_fdir_state.read(fdir_state) == RTT::NewData)
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_FDIR_STATUS_INDEX]=(double)(fdir_state);
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
    }

    if (_telemetry_product.read(tm_in) == RTT::NewData)
    {
        sendProduct(tm_in);
    }
}

void Task::getAndExecTelecommand()
{
    CommandInfo* cmd_info = activemqTCReceiver->extractCommandInfo();  // ActiveMQ messaging Protocol
    // CommandInfo* cmd_info = tcComm->extractCommandInfo();           // Server/Client TCP-IP sockets messaging Protocol
    if (cmd_info != NULL)
    {
        auto cmd_str = cmd_info->activityName;

        try
        {
            getExecFunction(cmd_str)(cmd_info);
            LOG_DEBUG_S << "Found and executed exec function";
            initializeTimer(cmd_str);
        }
        catch (std::exception& e)
        {
            RobotTask *rover_action = ( RobotTask* ) theRobotProcedure->GetRTFromName( (char*)(cmd_str).c_str());
            if (rover_action != NULL)
            {
                orcExecAct((char*)cmd_info->activityName.c_str(),(char*)cmd_info->activityParams.c_str(), 1);
            }
            else
            {
                LOG_WARN_S <<  "TC command not recognized!";
            }
        }
    }
}

int Task::getTimer(const string cmd_str)
{
    return std::get<0>(tc_map[cmd_str]);
}

void Task::setTimer(const string cmd_str, const int val)
{
    std::get<0>(tc_map[cmd_str]) = val;
}

void Task::setTimeout(const string cmd_str, const int val)
{
    std::get<1>(tc_map[cmd_str]) = val;
}

void Task::initializeTimer(const string cmd_str)
{
    std::get<0>(tc_map[cmd_str]) = std::get<1>(tc_map[cmd_str]);
}

std::function< void(CommandInfo*) > Task::getExecFunction(const string cmd_str)
{
    if (tc_map.find(cmd_str)!=tc_map.end())
        return std::get<2>(tc_map[cmd_str]);
    else
        throw std::exception();
}

std::function< bool(void) > Task::getControlFunction(const string cmd_str)
{
    return std::get<3>(tc_map[cmd_str]);
}

void Task::controlRunningActivities()
{
    for (auto &map_entry : tc_map)
    {
        auto cmd_str = map_entry.first;
        auto timer = getTimer(cmd_str);
        if (timer > 0)
        {
            bool activity_finished = getControlFunction(cmd_str)();
            if (activity_finished)
            {
                setTimer(cmd_str, -1);
                LOG_INFO_S << "Completed activity: " << cmd_str;
                theRobotProcedure->GetRTFromName((char*)(cmd_str).c_str())->post_cond=1;
            }
            else
            {
                setTimer(cmd_str, timer-1);
            }
        }
        else if (timer == 0)
        {
            LOG_INFO_S << "Timeout in activity: " << cmd_str;
            abort_activity=true;
            getControlFunction(cmd_str)();
            setTimer(cmd_str, -1);
            // ToDo when a Timeout is reached a different notification to post_cond=1 should be used.
            theRobotProcedure->GetRTFromName((char*)(cmd_str).c_str())->post_cond=1;
        }
    }
}

void Task::checkDeadManSwitch()
{
    // only send stop command once. when emergency stop command is sent, deadMan is true
    if (!deadMan && deadManSwitchRelevant)
    {
        base::Time elapsedTime = base::Time::now() - lastDirectCommandTime;
        if (elapsedTime.toMicroseconds() > deadManTime)
        {
            targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();
            deadMan = true;
            LOG_INFO_S << "Dead man switch stopped motion";
        }
    }
}

bool Task::ctrl_REAR_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[LOCCAM_RLOC_STEREO_INDEX]=RLOC_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_PANCAM_PANORAMA()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting PanCamState";
    }
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting PanCamState";
    }
    //TODO need input port from pancam_panorama to signal "finished"
    return true;
}

bool Task::ctrl_TOF_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[TOF_INDEX]=TOF_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_LIDAR_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[LIDAR_INDEX]=LIDAR_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_ALL_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[LIDAR_INDEX]=LIDAR_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return true;
}

bool Task::ctrl_LOCCAM_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[HAZCAM_FHAZ_STEREO_INDEX]=FHAZ_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_HAZCAM_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[HAZCAM_FHAZ_STEREO_INDEX]=FHAZ_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_MAST_PTU_MOVE_TO()
{
    if (ptuTargetReached())
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error getting MastState";
        }
        MastState[MAST_ACTION_RET_INDEX]=ACTION_RET_OK;
        MastState[MAST_ACTION_ID_INDEX]=0;
        if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error setting MastState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error getting MastState";
        }
        MastState[MAST_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        MastState[MAST_ACTION_ID_INDEX]=35;
        if ( theRobotProcedure->GetParameters()->set( (char*)"MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            LOG_WARN_S << "Error setting MastState";
        }
        return false;
    }
}

bool Task::ctrl_DEPLOYMENT_REAR()
{
    if (bema3TargetReached())
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=37;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_DEPLOYMENT_FRONT()
{
    if (bema2TargetReached())
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=36;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_DEPLOYMENT_ALL()
{
    if (bema1TargetReached())
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=35;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_TRAJECTORY()
{
    if (target_reached || abort_activity)
    {
        abort_activity=false;
        target_reached=true;
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetOrientationTheta=0.0;
        trajectory.clear();
        _trajectory.write(trajectory);
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=34;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_TRAJECTORY_WISDOM()
{
    if (target_reached || abort_activity)
    {
        abort_activity=false;
        target_reached=true;
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetOrientationTheta=0.0;
        trajectory.clear();
        _trajectory.write(trajectory);
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if (!WisdomAcquiring) // It is traversing
        {
            travelledDistance = getTravelledDistance();
            if (travelledDistance >= WisdomDistance)
            {
                WisdomAcquiring=true;
                travelledDistance = 0.0;
                initial_pose = pose;
                //signal command_arbiter
                double speed = 0.00001; // speed needs to be positive. Speed zero is not accepted by waypoint navigation are correct config parameter
                _trajectory_speed.write(speed);
            }
        }
        else // It is stopped to Acquire
        {
            WisdomAcqTime++;
            if (WisdomAcqTime >= (int)(WisdomTimer/taskPeriod))
            {
                WisdomAcquiring=false;
                WisdomAcqTime=0;
                //signal command_arbiter
                _trajectory_speed.write(targetSpeed);
            }
        }
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=34;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_TURNSPOT_GOTO()
{
    if (angleReached() || abort_activity)
    {
        LOG_INFO_S << "Finish Turnspot";
        abort_activity=false;
        travelledAngle = 0.0;
        targetOrientationTheta = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=33;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_AUTONAV_GOTO()
{
    bool autonav_finished = false;
    if (abort_activity || (_autonav_finished.read(autonav_finished) == RTT::NewData))
    {
        abort_activity=false;
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetOrientationTheta=0.0;
        max_obstacle = 0.0;
        max_slope = 0.0;
        cold_start = 0;
        _autonav_reset.write(true);
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=32;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_ACKERMANN_GOTO()
{
    travelledDistance = getTravelledDistance();
    if ((travelledDistance >= targetDistance) || abort_activity)
    {
        abort_activity=false;
        travelledDistance = 0.0;
        targetDistance = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetSpeed=0.0;
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=32;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_WHEELWALK_GOTO()
{
    travelledDistance = getTravelledDistance();
    if ((travelledDistance >= targetDistance) || abort_activity)
    {
        abort_activity=false;
        travelledDistance = 0.0;
        targetDistance = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetSpeed=0.0;
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=32;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_GNC_LLO()
{
    //travelledDistance = getTravelledDistance();
    //if ((travelledDistance >= targetDistance) || abort_activity)
    if (abort_activity)
    {
        abort_activity=false;
        travelledDistance = 0.0;
        targetDistance = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
        targetPositionX=0.0;
        targetPositionY=0.0;
        targetSpeed=0.0;
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error getting GNCState";
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=32;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( (char*)"GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            LOG_WARN_S << "Error setting GNCState";
        }
        return false;
    }
}

bool Task::ctrl_PANCAM_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting PanCamState";
    }
    PanCamState[PANCAM_PAN_STEREO_INDEX]=PAN_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting PanCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_MAST_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting PanCamState";
    }
    PanCamState[PANCAM_PAN_STEREO_INDEX]=PAN_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting PanCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_NAVCAM_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[LOCCAM_FLOC_STEREO_INDEX]=FLOC_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

bool Task::ctrl_FRONT_ACQ()
{
    if ( theRobotProcedure->GetParameters()->get( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error getting LocCamState";
    }
    PanCamState[LOCCAM_FLOC_STEREO_INDEX]=FLOC_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( (char*)"PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        LOG_WARN_S << "Error setting LocCamState";
    }
    return getProductWaitStatus();
}

void Task::setProductWaitFlags(messages::ProductType productType)
{
    switch (productType)
    {
        case messages::ProductType::IMAGE:
            sent_image_left = false;
            break;
        case messages::ProductType::STEREO:
            sent_image_left = false;
            sent_image_right = false;
            break;
        case messages::ProductType::DISTANCE:
            sent_image_left = false;
            sent_distance_image = false;
            break;
        case messages::ProductType::POINT_CLOUD:
            sent_point_cloud = false;
            break;
        case messages::ProductType::DEM:
            sent_image_left = false;
            sent_dem = false;
            break;
        default:
            break;
    }
}

bool Task::getProductWaitStatus()
{
    return sent_image_left &&
        sent_image_right &&
        sent_distance_image &&
        sent_point_cloud &&
        sent_dem;
}
