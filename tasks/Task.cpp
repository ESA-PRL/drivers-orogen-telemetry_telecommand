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

const double DEG2RAD = 3.141592/180;
const double RAD2DEG = 180/3.141592;

const double MIN_ACK_RADIUS = 0.6;
const double OMEGA = 0.02;                      //in Rad/s the commanded angular velocity to the walking actuators when deploying

const double PANLIMIT_LEFT = 155*DEG2RAD;       //HDPR
const double PANLIMIT_RIGHT = -155*DEG2RAD;     //HDPR
const double TILTLIMIT_LOW = -25*DEG2RAD;       //HDPR
const double TILTLIMIT_HIGH= 45*DEG2RAD;        //HDPR
const double DEPLOYMENTLIMIT = 95;              //ExoTeR
const double TARGET_WINDOW = 0.01;              //ExoTeR
const double TARGET_WINDOW2 = 0.01;             //ExoTeR
const double TARGET_WINDOW3 = 2.0;              //ExoTeR

using namespace telemetry_telecommand;

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

    // map telecommand strings to the corresponding enum and function
    tc_map = {
        { "GNC_ACKERMANN_GOTO",   std::make_tuple( -1, 3, std::bind( &Task::exec_GNC_ACKERMANN_GOTO,   this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_ACKERMANN_GOTO,   this ) ) },
        { "GNC_TURNSPOT_GOTO",    std::make_tuple( -1, 3, std::bind( &Task::exec_GNC_TURNSPOT_GOTO,    this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_TURNSPOT_GOTO,    this ) ) },
        { "GNC_TRAJECTORY",       std::make_tuple( -1, 3, std::bind( &Task::exec_GNC_TRAJECTORY,       this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_TRAJECTORY,       this ) ) },
        { "MAST_PTU_MoveTo",      std::make_tuple( -1, 3, std::bind( &Task::exec_MAST_PTU_MOVE_TO,     this, std::placeholders::_1), std::bind( &Task::ctrl_MAST_PTU_MOVE_TO,     this ) ) },
        { "PANCAM_PANORAMA",      std::make_tuple( -1, 3, std::bind( &Task::exec_PANCAM_PANORAMA,      this, std::placeholders::_1), std::bind( &Task::ctrl_PANCAM_PANORAMA,      this ) ) },
        { "TOF_ACQ",              std::make_tuple( -1, 3, std::bind( &Task::exec_TOF_ACQ,              this, std::placeholders::_1), std::bind( &Task::ctrl_TOF_ACQ,              this ) ) },
        { "LIDAR_ACQ",            std::make_tuple( -1, 3, std::bind( &Task::exec_LIDAR_ACQ,            this, std::placeholders::_1), std::bind( &Task::ctrl_LIDAR_ACQ,            this ) ) },
        { "FRONT_ACQ",            std::make_tuple( -1, 3, std::bind( &Task::exec_FRONT_ACQ,            this, std::placeholders::_1), std::bind( &Task::ctrl_FRONT_ACQ,            this ) ) },
        { "MAST_ACQ",             std::make_tuple( -1, 3, std::bind( &Task::exec_MAST_ACQ,             this, std::placeholders::_1), std::bind( &Task::ctrl_MAST_ACQ,             this ) ) },
        { "REAR_ACQ",             std::make_tuple( -1, 3, std::bind( &Task::exec_REAR_ACQ,             this, std::placeholders::_1), std::bind( &Task::ctrl_REAR_ACQ,             this ) ) },
        { "HAZCAM_ACQ",           std::make_tuple( -1, 3, std::bind( &Task::exec_HAZCAM_ACQ,           this, std::placeholders::_1), std::bind( &Task::ctrl_HAZCAM_ACQ,           this ) ) },
        { "Deployment_All",       std::make_tuple( -1, 3, std::bind( &Task::exec_DEPLOYMENT_ALL,       this, std::placeholders::_1), std::bind( &Task::ctrl_DEPLOYMENT_ALL,       this ) ) },
        { "Deployment_Front",     std::make_tuple( -1, 3, std::bind( &Task::exec_DEPLOYMENT_FRONT,     this, std::placeholders::_1), std::bind( &Task::ctrl_DEPLOYMENT_FRONT,     this ) ) },
        { "Deployment_Rear",      std::make_tuple( -1, 3, std::bind( &Task::exec_DEPLOYMENT_REAR,      this, std::placeholders::_1), std::bind( &Task::ctrl_DEPLOYMENT_REAR,      this ) ) },
        { "GNC_Update",           std::make_tuple( -1, 3, std::bind( &Task::exec_GNC_UPDATE,           this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_UPDATE,           this ) ) },
        { "GNC_ACKERMANN_DIRECT", std::make_tuple( -1, 3, std::bind( &Task::exec_GNC_ACKERMANN_DIRECT, this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_ACKERMANN_DIRECT, this ) ) },
        { "GNC_TURNSPOT_DIRECT",  std::make_tuple( -1, 3, std::bind( &Task::exec_GNC_TURNSPOT_DIRECT,  this, std::placeholders::_1), std::bind( &Task::ctrl_GNC_TURNSPOT_DIRECT,  this ) ) },
        { "ALL_ACQ",              std::make_tuple( -1, 3, std::bind( &Task::exec_ALL_ACQ,              this, std::placeholders::_1), std::bind( &Task::ctrl_ALL_ACQ,              this ) ) },
        { "GNCG",                 std::make_tuple( -1, 3, std::bind( &Task::exec_GNCG,                 this, std::placeholders::_1), std::bind( &Task::ctrl_GNCG,                 this ) ) },
        { "ABORT",                std::make_tuple( -1, 3, std::bind( &Task::exec_ABORT,                this, std::placeholders::_1), std::bind( &Task::ctrl_ABORT,                this ) ) }
    };

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

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
    tmComm = new CommTmServer( TM_SERVER_PORT_NUMBER, theRobotProcedure, activemqTMSender);
    tcReplyServer =  new CommTcReplyServer( TC_REPLY_SERVER_PORT_NUMBER );

    theRobotProcedure->insertRT(new RobotTask("ADE_LEFT_Initialise"));      // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_LEFT_conf"));            // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_LEFT_ReleaseHDRM"));     // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_LEFT_SwitchOff"));       // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_RIGHT_Initialise"));     // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_RIGHT_conf"));           // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_RIGHT_ReleaseHDRM"));    // Simulated
    theRobotProcedure->insertRT(new RobotTask("ADE_RIGHT_SwitchOff"));      // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_Initialise"));       // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_PrimaryMoveTo"));    // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_SecondaryMoveTo"));  // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_LEFT_SwitchOff"));        // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_Initialise"));      // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_PrimaryMoveTo"));   // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_SecondaryMoveTo")); // Simulated
    theRobotProcedure->insertRT(new RobotTask("SA_RIGHT_SwitchOff"));       // Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_Initialise"));        // Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_InitWACs"));          // Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_SwitchOn"));          // Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_WACAcqImage"));       // Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_ACQ"));                 // Executed (params WAC_L, WAC_R)
    theRobotProcedure->insertRT(new RobotTask("HAZCAM_ACQ"));               // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("LIDAR_ACQ"));                // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("TOF_ACQ"));                  // Executed in HDPR (params TBD)
    theRobotProcedure->insertRT(new RobotTask("PanCam_SwitchOff"));         // Simulated
    theRobotProcedure->insertRT(new RobotTask("PanCam_PIUSwitchOff"));      // Simulated
    theRobotProcedure->insertRT(new RobotTask("PANCAM_PANORAMA"));          // Executed (params tilt angle in deg)
    theRobotProcedure->insertRT(new RobotTask("PanCam_FilterSel"));         // Simulated
    theRobotProcedure->insertRT(new RobotTask("FRONT_ACQ"));                // Executed
    theRobotProcedure->insertRT(new RobotTask("REAR_ACQ"));                 // Executed
    theRobotProcedure->insertRT(new RobotTask("ALL_ACQ"));                  // Executed
    theRobotProcedure->insertRT(new RobotTask("MAST_DEP_Initialise"));      // Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_DEP_Deploy"));          // Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_PTU_Initialise"));      // Simulated
    theRobotProcedure->insertRT(new RobotTask("MAST_PTU_MoveTo"));          // Executed (params: pan, tilt (deg, deg))
    theRobotProcedure->insertRT(new RobotTask("MAST_SwitchOff"));           // Simulated
    theRobotProcedure->insertRT(new RobotTask("GNC_Initialise"));           // Simulated
    theRobotProcedure->insertRT(new RobotTask("GNC_Update"));               // Executed  (params: x,y,z in meters rx,ry,rz in degrees)
    theRobotProcedure->insertRT(new RobotTask("GNC_ACKERMANN_GOTO"));       // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_TURNSPOT_GOTO"));        // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_ACKERMANN_DIRECT"));     // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_TURNSPOT_DIRECT"));      // Executed  (params: distance, speed (m, m/hour))
    theRobotProcedure->insertRT(new RobotTask("GNC_TRAJECTORY"));           // Executed  (params: number of waypoints, vector of waypoints(x,y,h))
    theRobotProcedure->insertRT(new RobotTask("GNC_SwitchOff"));            // Simulated
    theRobotProcedure->insertRT(new RobotTask("Deployment_All"));           // Executed  (params: deploy angle in deg)
    theRobotProcedure->insertRT(new RobotTask("Deployment_Front"));         // Executed  (params: deploy angle in deg)
    theRobotProcedure->insertRT(new RobotTask("Deployment_Rear"));          // Executed  (params: deploy angle in deg)
    theRobotProcedure->insertRT(new RobotTask("RV_WakeUp"));                // Simulated
    theRobotProcedure->insertRT(new RobotTask("MMS_WaitAbsTime"));          // Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Comms"));         // Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_PostComms"));             // Simulated
    theRobotProcedure->insertRT(new RobotTask("DHS_Go2Nominal"));           // Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Travel"));        // Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Night"));         // Simulated
    theRobotProcedure->insertRT(new RobotTask("RV_Prepare4Dozing"));        // Simulated

    //! ToDo: Check if this fix is still necessary. It might be fixed at ptu_control component configure/start hook.
    //! Send ptu and motion commands to activate the joint dispatcher. Otherwise stays waiting.
    pan = 0.0; tilt = 0.0; sendPtuCommand();
    targetTranslation = 0.0; targetRotation = 0.0; sendMotionCommand();

    /**
     * Routine to send the bema command to the rover stowed position (beggining of Egress in ExoTeR)
     */
    /*
       currentActivity = DEPLOYMENT_ALL_ACTIVITY;
       bema_command=-90.0;
       std::cout <<  "Deployment All: " << bema_command << std::endl;
       bema_command = bema_command*DEG2RAD;
       _bema_command.write(-2.0*OMEGA);

       target_reached=false;
       NofWaypoints=0;
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
    double distance = 0.0;
    distance = sqrt((pose.position[0]-initial_pose.position[0])*(pose.position[0]-initial_pose.position[0])
            +(pose.position[1]-initial_pose.position[1])*(pose.position[1]-initial_pose.position[1])
            +(pose.position[2]-initial_pose.position[2])*(pose.position[2]-initial_pose.position[2]));
    return distance;
}

bool Task::angleReached()
{
    double angle =0.0;
    angle = std::abs(pose.getYaw()*RAD2DEG-targetOrientationTheta);
    return angle<TARGET_WINDOW3;
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
    if (tilt > TILTLIMIT_HIGH)
        tilt = TILTLIMIT_HIGH;
    else if (tilt < TILTLIMIT_LOW)
        tilt = TILTLIMIT_LOW;
    _mast_pan.write(pan);
    _mast_tilt.write(tilt*4);
}

void Task::sendMotionCommand()
{
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
        case messages::Producer::MAST:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        std::cout << "Telemetry: sending image from Mast " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent image with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        std::cout << "Telemetry: sending distance from Mast " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent distance file with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        std::cout << "Telemetry: sending point cloud from Mast " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent point cloud file with size " << data.size() << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        std::cout << "Telemetry: sending dem from Mast " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent dem with size " << data.size() << std::endl;
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        std::cout << "Telemetry: sending mtl file " << mtl_filename << std::endl;
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
                            std::cout << "Telemetry: sent mtl file with size " << size2 << std::endl;
                        }
                        break;
                    }
            }
            break;
        case messages::Producer::LIDAR:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        std::cout << "Telemetry: sending image from Lidar " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent image with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        std::cout << "Telemetry: sending distance from Lidar " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent distance file with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        std::cout << "Telemetry: sending point cloud from Lidar " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent point cloud file with size " << data.size() << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        std::cout << "Telemetry: sending dem from Lidar " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent dem with size " << data.size() << std::endl;
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        std::cout << "Telemetry: sending mtl file " << mtl_filename << std::endl;
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
                            std::cout << "Telemetry: sent mtl file with size " << size2 << std::endl;
                        }
                        break;
                    }
            }
            break;
        case messages::Producer::FRONT:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        std::cout << "Telemetry: sending image from Front " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent image with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        std::cout << "Telemetry: sending distance from Front " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent distance file with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        std::cout << "Telemetry: sending point cloud from Front " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent point cloud file with size " << data.size() << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        std::cout << "Telemetry: sending dem from Front " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent dem with size " << data.size() << std::endl;
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        std::cout << "Telemetry: sending mtl file " << mtl_filename << std::endl;
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
                            std::cout << "Telemetry: sent mtl file with size " << size2 << std::endl;
                        }
                        break;
                    }
            }
            break;
        case messages::Producer::TOF:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        std::cout << "Telemetry: sending image from Tof " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent image with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        std::cout << "Telemetry: sending distance from Tof " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent distance file with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        std::cout << "Telemetry: sending point cloud from Tof " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent point cloud file with size " << data.size() << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        std::cout << "Telemetry: sending dem from Tof " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent dem with size " << data.size() << std::endl;
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        std::cout << "Telemetry: sending mtl file " << mtl_filename << std::endl;
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
                            std::cout << "Telemetry: sent mtl file with size " << size2 << std::endl;
                        }
                        break;
                    }
            }
            break;
        case messages::Producer::HAZCAM:
            switch (tm.type) {
                case messages::ProductType::IMAGE:
                    {
                        std::cout << "Telemetry: sending image from Hazcam " << tm.productPath << std::endl;
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
                            tmComm->sendImageMessage(tm.productPath.c_str(), seq, time, date.c_str(), size, (const unsigned char *)data, activemqTMSender->imgHazcamProducerMonitoring, transformation);
                            std::cout << "Telemetry: sent image with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        std::cout << "Telemetry: sending distance from Hazcam " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent distance file with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        std::cout << "Telemetry: sending point cloud from Hazcam " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent point cloud file with size " << data.size() << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        std::cout << "Telemetry: sending dem from Hazcam " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent dem with size " << data.size() << std::endl;
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        std::cout << "Telemetry: sending mtl file " << mtl_filename << std::endl;
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
                            std::cout << "Telemetry: sent mtl file with size " << size2 << std::endl;
                        }
                        break;
                    }
            }
            break;
        case messages::Producer::REAR:
            switch (tm.type)
            {
                case messages::ProductType::IMAGE:
                    {
                        std::cout << "Telemetry: sending image from Rear " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent image with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DISTANCE:
                    {
                        std::cout << "Telemetry: sending distance from Rear " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent distance file with size " << size << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::POINT_CLOUD:
                    {
                        std::cout << "Telemetry: sending point cloud from Rear " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent point cloud file with size " << data.size() << std::endl;
                        }
                        break;
                    }
                case messages::ProductType::DEM:
                    {
                        std::cout << "Telemetry: sending dem from Rear " << tm.productPath << std::endl;
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
                            std::cout << "Telemetry: sent dem with size " << data.size() << std::endl;
                        }
                        std::string mtl_filename = tm.productPath.replace(tm.productPath.find("obj"), 3, "mtl");
                        mtl_filename.replace(mtl_filename.find(".gz"),3,"");
                        std::cout << "Telemetry: sending mtl file " << mtl_filename << std::endl;
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
                            std::cout << "Telemetry: sent mtl file with size " << size2 << std::endl;
                        }
                        break;
                    }
            }
            break;
    }
}

void Task::exec_GNC_ACKERMANN_GOTO(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf %lf %lf", &ackid, &targetPositionX, &targetPositionY, &targetSpeed);

    // Calculate the parameters to be sent as motion commands (2D): Translation (m/s) and Rotation (rad/s)
    if (targetPositionY == 0) // Straight line command
    {
        targetDistance = std::abs(targetPositionX);
        double sign = (targetPositionX < 0 ? -1 : 1);
        targetTranslation = targetSpeed*sign;
        targetRotation = 0;
        return;
    }
    double radius = (targetPositionX*targetPositionX + targetPositionY*targetPositionY)/(2*targetPositionY);
    if (std::abs(radius)<MIN_ACK_RADIUS)
    {
        std::cout << "Telemetry_Telecommand: Aborting Ackerman activity. Radius of curvature too small. Try Point Turn first." << std::endl;
        targetTranslation=0.0;
        targetRotation=0.0;
        setTimeout(cmd_info->activityName, -1);
        return;
    }
    double theta = atan(targetPositionX/std::abs(radius-targetPositionY));
    double sign = (targetPositionX < 0 ? -1 : 1);
    targetTranslation = targetSpeed*sign;
    targetRotation = targetTranslation/radius;
    targetDistance = std::abs(theta*radius);

    travelledDistance = 0.0;
    initial_pose = pose;
    std::cout <<  "GNC_ACKERMANN_GOTO X:" << targetPositionX << " Y:" << targetPositionY << " speed:" << targetSpeed << std::endl;
    sendMotionCommand();
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_GNC_TURNSPOT_GOTO(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf %lf", &ackid, &targetOrientationTheta, &targetRotation);
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
    std::cout <<  "GNC_TURNSPOT_GOTO angle:" << targetOrientationTheta << std::endl;
    sendMotionCommand();
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_GNC_TRAJECTORY(CommandInfo* cmd_info)
{
    target_reached=false;
    currentParams = cmd_info->activityParams;
    char *token_str = strtok((char *)(currentParams.c_str()), " ");
    token_str = strtok(NULL, " ");
    int NofWaypoints = atoi(token_str);
    std::cout << "NofWaypoints:" << NofWaypoints << "<-" << std::endl;
    for (int i=0;i<NofWaypoints;i++)
    {
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
    token_str = strtok(NULL, " ");
    targetSpeed = atof(token_str);
    _trajectory.write(trajectory);
    if (targetSpeed>0)
    {
        _trajectory_speed.write(targetSpeed);
    }
    std::cout <<  "GNC_TRAJECTORY #ofWaypoints:" << NofWaypoints << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_MAST_PTU_MOVE_TO(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf %lf", &ackid, &pan, &tilt);
    std::cout <<  "MAST_PTU_MoveTo pan:" << pan << " tilt:" << tilt << std::endl;
    pan = pan*DEG2RAD;
    tilt = tilt*DEG2RAD;
    sendPtuCommand();
    if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
    {
        std::cout << "Error getting MastState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
    {
        std::cout << "Error setting MastState" << std::endl;
    }
}

void Task::exec_DEPLOYMENT_ALL(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf", &ackid, &bema_command);
    if (bema_command>DEPLOYMENTLIMIT)
        bema_command=DEPLOYMENTLIMIT;
    else if (bema_command<-DEPLOYMENTLIMIT)
        bema_command=-DEPLOYMENTLIMIT;
    std::cout <<  "Deployment All: " << bema_command << std::endl;
    bema_command = bema_command*DEG2RAD;
    if (bema_command>bema[0].position)
    {
        _bema_command.write(OMEGA);
    }
    else
    {
        _bema_command.write(-OMEGA);
    }
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_DEPLOYMENT_FRONT(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf", &ackid, &bema_command);
    if (bema_command>DEPLOYMENTLIMIT)
        bema_command=DEPLOYMENTLIMIT;
    else if (bema_command<-DEPLOYMENTLIMIT)
        bema_command=-DEPLOYMENTLIMIT;
    std::cout <<  "Deployment Front: " << bema_command << std::endl;
    bema_command = bema_command*DEG2RAD;
    if (bema_command>bema[0].position)
    {
        _walking_command_front.write(OMEGA);
    }
    else
    {
        _walking_command_front.write(-OMEGA);
    }
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_DEPLOYMENT_REAR(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf", &ackid, &bema_command);
    if (bema_command>DEPLOYMENTLIMIT)
        bema_command=DEPLOYMENTLIMIT;
    else if (bema_command<-DEPLOYMENTLIMIT)
        bema_command=-DEPLOYMENTLIMIT;
    std::cout <<  "Deployment Rear: " << bema_command << std::endl;
    bema_command = bema_command*DEG2RAD;
    if (bema_command>bema[4].position)
    {
        _walking_command_rear.write(OMEGA);
    }
    else
    {
        _walking_command_rear.write(-OMEGA);
    }
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_GNC_UPDATE(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    double update_pose_x;
    double update_pose_y;
    double update_pose_z;
    double update_pose_rx;
    double update_pose_ry;
    double update_pose_rz;
    sscanf(currentParams.c_str(), "%d %lf %lf %lf %lf %lf %lf", &ackid, &update_pose_x, &update_pose_y, &update_pose_z, &update_pose_rx, &update_pose_ry, &update_pose_rz);
    absolute_pose.position[0]=update_pose_x;
    absolute_pose.position[1]=update_pose_y;
    absolute_pose.position[2]=update_pose_z;
    Eigen::Quaternion <double> orientation(Eigen::AngleAxisd(update_pose_rz*DEG2RAD, Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(update_pose_ry*DEG2RAD, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(update_pose_rx*DEG2RAD, Eigen::Vector3d::UnitX()));
    absolute_pose.orientation = orientation;
    _update_pose.write(absolute_pose);
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
}

void Task::exec_GNC_ACKERMANN_DIRECT(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf %lf", &ackid, &targetTranslation, &targetRotation);
    std::cout <<  "GNC_ACKERMANN_DIRECT Translation:" << targetTranslation << " Rotation:" << targetRotation << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        std::cout << "Error getting GNCState" << std::endl;
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        std::cout << "Error setting GNCState" << std::endl;
    deadManSwitch();

    // check if we are in direct or in path following mode
    if (target_reached)
    {
        // if not in trajectory following, send complete (direct) command
        sendMotionCommand();
    }
    else
    {
        // trajectory following cannot go into reverse
        // TODO don't write speeds but change PID parameters
        if (targetTranslation >= 0)
        {
            _trajectory_speed.write(targetTranslation);
        }
        else
        {
            _trajectory_speed.write(0.0);
        }
    }
}

void Task::exec_GNC_TURNSPOT_DIRECT(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf", &ackid, &targetRotation);
    // TODO does the targetRotation(Speed) need to be multiplied by DEG2RAD here as well?
    targetTranslation=0.0;
    std::cout <<  "GNC_TURNSPOT_DIRECT Rotation:" << targetRotation << std::endl;
    sendMotionCommand();
    if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error getting GNCState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
    {
        std::cout << "Error setting GNCState" << std::endl;
    }
    deadManSwitch();
}

void Task::exec_MAST_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "PanCam Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting PanCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=50;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting PanCamState" << std::endl;
    }
}

void Task::exec_GNCG(CommandInfo* cmd_info)
{
    TaskLib* taskLib = new TaskLib("");
    taskLib->insertSol(std::string("/home/marta/rock/bundles/rover/config/orogen/ActivityPlan.txt"));
    taskLib->ExecuteActivityPlan();
}

void Task::exec_FRONT_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "LocCamFront Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=51;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
}

void Task::exec_REAR_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "LocCamRear Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=52;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
}

void Task::exec_HAZCAM_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "HazCamFront Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
}

void Task::exec_TOF_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "Tof Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
}

void Task::exec_LIDAR_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "Lidar Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
}

void Task::exec_ALL_ACQ(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %d %d", &ackid, &productType, &productMode);
    tc_out.productType = productType;
    if (productMode>0)
    {
        tc_out.productMode=messages::Mode::PERIODIC;
        tc_out.usecPeriod=productMode*1000;
    }
    else
    {
        tc_out.productMode = productMode;
    }
    std::cout <<  "All sensors Get Image type " << productType << " and mode: " << productMode << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[PANCAM_ACTION_ID_INDEX]=53;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
}

void Task::exec_PANCAM_PANORAMA(CommandInfo* cmd_info)
{
    currentParams = cmd_info->activityParams;
    int ackid;
    sscanf(currentParams.c_str(), "%d %lf", &ackid, &panorama_tilt);
    tc_out.productType = messages::ProductType::DEM;
    tc_out.productMode = messages::Mode::CONTINUOUS;
    std::cout <<  "PanCam Panorama at tilt: " << panorama_tilt << std::endl;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting PanCamState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting PanCamState" << std::endl;
    }
}

void Task::exec_ABORT(CommandInfo* cmd_info)
{
    abort_activity=true;
    std::cout << "Abort message received!" << std::endl;
}

void Task::reactToInputPorts()
{
    if (_current_pose.read(pose) == RTT::NewData)
    {
        //! new TM packet with updated pose estimation
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
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
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
    }
    if (_current_bema.read(bema) == RTT::NewData)
    {
        //! new TM packet with updated bema estimation
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
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
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
    }
    if (_joint_samples.read(joint_samples) == RTT::NewData)
    {
        //! new TM packet with updated joint samples
        if ( theRobotProcedure->GetParameters()->get( "LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            std::cout << "Error getting LOCOMState" << std::endl;
        }
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
        aux = (int)(joint_samples[0].raw*100);
        LOCOMState[GNC_ROVER_STEER1_CURRENT_INDEX]=(double)((double)aux/100.0);
        aux = (int)(joint_samples[1].raw*100);
        LOCOMState[GNC_ROVER_STEER2_CURRENT_INDEX]=(double)((double)aux/100.0);
        aux = (int)(joint_samples[4].raw*100);
        LOCOMState[GNC_ROVER_STEER5_CURRENT_INDEX]=(double)((double)aux/100.0);
        aux = (int)(joint_samples[5].raw*100);
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

        if ( theRobotProcedure->GetParameters()->set( "LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            std::cout << "Error setting LOCOMState" << std::endl;
        }
    }
    if (_motor_temperatures.read(motor_temperatures) == RTT::NewData)
    {
        //! new TM packet with updated motor temperature values
        if ( theRobotProcedure->GetParameters()->get( "LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            std::cout << "Error getting LOCOMState" << std::endl;
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
        if ( theRobotProcedure->GetParameters()->set( "LOCOMState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) LOCOMState ) == ERROR )
        {
            std::cout << "Error setting LOCOMState" << std::endl;
        }
    }
    if (_current_ptu.read(ptu) == RTT::NewData)
    {
        //! new TM packet with updated ptu position
        if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error getting MastState" << std::endl;
        }
        int aux = (int)(ptu[0].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q2_INDEX]=(double)((double)aux/100.0);
        aux = (int)(ptu[1].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q3_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error setting MastState" << std::endl;
        }
    }
    if (_current_pan.read(pan) == RTT::NewData)
    {
        //! new TM packet with updated pan position (HDPR)
        ptu[0].position=pan;
        if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error getting MastState" << std::endl;
        }
        int aux = (int)(ptu[0].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q2_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error setting MastState" << std::endl;
        }
    }
    if (_current_tilt.read(tilt) == RTT::NewData)
    {
        //! new TM packet with updated tilt position (HDPR)
        ptu[1].position=tilt/4; // HDPR tilt value comes with a factor of 4
        if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error getting MastState" << std::endl;
        }
        int aux = (int)(ptu[1].position*RAD2DEG*100);
        MastState[MAST_CURRENT_Q3_INDEX]=(double)((double)aux/100.0);
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error setting MastState" << std::endl;
        }
    }
    if (_current_imu.read(imu) == RTT::NewData)
    {
        //! new TM packet with updated imu pose
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        int aux = (int)(imu.getRoll()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERX_INDEX]=-(double)((double)aux/10.0);
        aux = (int)(imu.getPitch()*RAD2DEG*10);
        GNCState[GNC_ROVER_POSERY_INDEX]=-(double)((double)aux/10.0);
        aux = (int)((imu.getYaw()*RAD2DEG + initial_absolute_heading*RAD2DEG)*10);
        GNCState[GNC_ROVER_POSERZ_INDEX]=(double)((double)aux/10.0);
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
    }

    if (_trajectory_status.read(tj_status) == RTT::NewData)
    {
        if (tj_status == 2)  //! TARGET_REACHED
        {
            target_reached=true;
            std::cout << "Final trajectory target reached" << std::endl;
        }
        else if (tj_status == 3) //! OUT_OF_BOUNDARIES
        {
            abort_activity=true;
            target_reached=true;
            std::cout << "Rover out of trajectory boundaries. Aborting trajectory!" << std::endl;
        }
        //! Nothing to do in other status

        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_TRAJECTORY_STATUS_INDEX]=(double)(tj_status);
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
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
            initializeTimeout(cmd_str);
            getExecFunction(cmd_str)(cmd_info);
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
                std::cout <<  "DEBUG: TC command not recognised!" << std::endl;
            }
        }
    }
}

int Task::getTimeout(const string cmd_str)
{
    return std::get<0>(tc_map[cmd_str]);
}

void Task::setTimeout(const string cmd_str, const int val)
{
    std::get<0>(tc_map[cmd_str]) = val;
}

void Task::initializeTimeout(const string cmd_str)
{
    std::get<0>(tc_map[cmd_str]) = std::get<1>(tc_map[cmd_str]);
}

std::function< void(CommandInfo*) > Task::getExecFunction(const string cmd_str)
{
    return std::get<2>(tc_map[cmd_str]);
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
        auto timeout = getTimeout(cmd_str);
        if (timeout > 0)
        {
            bool activity_finished = getControlFunction(cmd_str)();
            if (activity_finished)
            {
                setTimeout(cmd_str, -1);
            }
            else
            {
                setTimeout(cmd_str, timeout-1);
            }
        }
        else if (timeout == 0)
        {
            //TODO signal timeout happened
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
            std::cout << "Dead man switch stopped motion" << std::endl;
        }
    }
}

bool Task::ctrl_REAR_ACQ()
{
    _rear_trigger.write(tc_out);
    RLOC_STEREO_index++;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[LOCCAM_RLOC_STEREO_INDEX]=RLOC_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_PANCAM_PANORAMA()
{
    _panoramica_trigger.write(tc_out);
    _panorama_tilt.write(panorama_tilt);
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting PanCamState" << std::endl;
    }
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting PanCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_TOF_ACQ()
{
    _tof_trigger.write(tc_out);
    TOF_index++;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[TOF_INDEX]=TOF_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_LIDAR_ACQ()
{
    _lidar_trigger.write(tc_out);
    LIDAR_index++;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[LIDAR_INDEX]=LIDAR_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_ALL_ACQ()
{
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
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[LIDAR_INDEX]=LIDAR_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_HAZCAM_ACQ()
{
    _haz_front_trigger.write(tc_out);
    FHAZ_STEREO_index++;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[HAZCAM_FHAZ_STEREO_INDEX]=FHAZ_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_MAST_PTU_MOVE_TO()
{
    if (ptuTargetReached() || abort_activity)
    {
        abort_activity=false;
        if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error getting MastState" << std::endl;
        }
        MastState[MAST_ACTION_RET_INDEX]=ACTION_RET_OK;
        MastState[MAST_ACTION_ID_INDEX]=0;
        MastState[MAST_STATUS_INDEX]=MAST_OPER_MODE_PTU_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error setting MastState" << std::endl;
        }
        theRobotProcedure->GetRTFromName("MAST_PTU_MoveTo")->post_cond=1;
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error getting MastState" << std::endl;
        }
        MastState[MAST_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        MastState[MAST_STATUS_INDEX]=MAST_OPER_MODE_PTU_MOVING;
        MastState[MAST_ACTION_ID_INDEX]=35;
        if ( theRobotProcedure->GetParameters()->set( "MastState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) MastState ) == ERROR )
        {
            std::cout << "Error setting MastState" << std::endl;
        }
        return false;
    }
}

bool Task::ctrl_DEPLOYMENT_REAR()
{
    if (bema3TargetReached() || abort_activity)
    {
        abort_activity=false;
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=37;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return false;
    }
}

bool Task::ctrl_DEPLOYMENT_FRONT()
{
    if (bema2TargetReached() || abort_activity)
    {
        abort_activity=false;
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=36;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return false;
    }
}

bool Task::ctrl_GNC_UPDATE()
{
    //TODO Add logic to handle an Update command. Probably need to trigger a restart of the odometry related component.
    return false;
}

bool Task::ctrl_DEPLOYMENT_ALL()
{
    if (bema1TargetReached() || abort_activity)
    {
        abort_activity=false;
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=35;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
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
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=34;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return false;
    }
}

bool Task::ctrl_GNC_TURNSPOT_GOTO()
{
    if ( angleReached() || abort_activity)
    {
        std::cout << "Finish Turnspot" << std::endl;
        abort_activity=false;
        travelledAngle = 0.0;
        targetOrientationTheta = 0.0;
        targetTranslation = 0.0;
        targetRotation = 0.0;
        sendMotionCommand();
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=33;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
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
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_OK;
        GNCState[GNC_ACTION_ID_INDEX]=0;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_STNDBY;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return true;
    }
    else
    {
        if ( theRobotProcedure->GetParameters()->get( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error getting GNCState" << std::endl;
        }
        GNCState[GNC_ACTION_RET_INDEX]=ACTION_RET_RUNNING;
        GNCState[GNC_ACTION_ID_INDEX]=32;
        GNCState[GNC_STATUS_INDEX]=GNC_OPER_MODE_LLO;
        if ( theRobotProcedure->GetParameters()->set( "GNCState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) GNCState ) == ERROR )
        {
            std::cout << "Error setting GNCState" << std::endl;
        }
        return false;
    }
}

bool Task::ctrl_MAST_ACQ()
{
    _mast_trigger.write(tc_out);
    PAN_STEREO_index++;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting PanCamState" << std::endl;
    }
    PanCamState[PANCAM_PAN_STEREO_INDEX]=PAN_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting PanCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_FRONT_ACQ()
{
    _front_trigger.write(tc_out);
    FLOC_STEREO_index++;
    if ( theRobotProcedure->GetParameters()->get( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error getting LocCamState" << std::endl;
    }
    PanCamState[LOCCAM_FLOC_STEREO_INDEX]=FLOC_STEREO_index-1;
    PanCamState[PANCAM_ACTION_ID_INDEX]=0;
    PanCamState[PANCAM_ACTION_RET_INDEX]=ACTION_RET_OK;
    if ( theRobotProcedure->GetParameters()->set( "PanCamState", DOUBLE, MAX_STATE_SIZE, 0, ( char * ) PanCamState ) == ERROR )
    {
        std::cout << "Error setting LocCamState" << std::endl;
    }
    return true;
}

bool Task::ctrl_GNC_ACKERMANN_DIRECT(){return true;} //TODO DELETE ME, direct command needs no controlling
bool Task::ctrl_GNC_TURNSPOT_DIRECT(){return true;}  //TODO DELETE ME, direct command needs no controlling
bool Task::ctrl_GNCG() { return true; } //TODO delete me

bool Task::ctrl_ABORT()
{
    //TODO adjust all timeouts
    return true;
}
