#ifndef TELEMETRY_TELECOMMAND_TASK_TASK_HPP
#define TELEMETRY_TELECOMMAND_TASK_TASK_HPP

#include <base/commands/Motion2D.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

#include "telemetry_telecommand/TaskBase.hpp"
#include "telemetry_telecommand/comm.h"
#include "telemetry_telecommand/rt.h"
#include "telemetry_telecommand/prr.h"

#include "telemetry_telecommand/ActiveMQTMSender.h"
#include "telemetry_telecommand/ActiveMQAdmin.h"
#include "telemetry_telecommand/ActiveMQTCReceiver.h"
#include "telemetry_telecommand/tasklib.h"

#include "telemetry_telecommand/Messages.hpp"
#include "temperature/temperatureTypes.hpp"

namespace telemetry_telecommand
{
    enum Telecommand
    {
        GNC_ACKERMANN_GOTO,
        GNC_TURNSPOT_GOTO,
        GNC_TRAJECTORY,
        MAST_PTU_MOVE_TO,
        PANCAM_WAC_ACQ,
        PANCAM_PANORAMA,
        LOCCAMFRONT_ACQ,
        LOCCAMREAR_ACQ,  //ExoTeR
        HAZCAMFRONT_ACQ, //HDPR
        TOF_ACQ,         //HDPR
        LIDAR_ACQ,       //HDPR
        FRONT_ACQ,
        MAST_ACQ,
        REAR_ACQ,
        HAZCAM_ACQ,
        DEPLOYMENT_ALL,  //ExoTeR
        DEPLOYMENT_FRONT,//ExoTeR
        DEPLOYMENT_REAR, //ExoTeR
        GNC_UPDATE,
        GNC_ACKERMANN_DIRECT,
        GNC_TURNSPOT_DIRECT,
        ALL_ACQ,
        ABORT,
        GNCG
    };

    class Task : public TaskBase
    {
        friend class TaskBase;
        protected:

        double taskPeriod;
        RoverName rover;

        int currentActivity;
        bool abort_activity;
        bool files_sent;
        int WACL_index, WACR_index, PAN_STEREO_index;
        int FLOCL_index, FLOCR_index, FLOC_STEREO_index;
        int RLOCL_index, RLOCR_index, RLOC_STEREO_index;
        int TOF_index, LIDAR_index, FHAZ_STEREO_index;

        // State variables definition
        double State[MAX_STATE_SIZE];
        double ADEState[MAX_STATE_SIZE];
        double SAState[MAX_STATE_SIZE];
        double PanCamState[MAX_STATE_SIZE];
        double LOCOMState[MAX_STATE_SIZE];
        double MastState[MAX_STATE_SIZE];
        double GNCState[MAX_STATE_SIZE];

        // TM message
        std::string tmmsg;

        // GNC_LLO parameters
        double totaltravelledDistance;
        double travelledDistance;
        double travelledAngle;
        double targetDistance;
        double targetPositionX; //meters
        double targetPositionY; //meters
        double targetOrientationTheta; //degrees
        double targetSpeed; //meters per second
        double targetTranslation;
        double targetRotation;
        double transformation[7];
        std::vector<base::Waypoint> trajectory;
        base::Waypoint waypoint;
        bool target_reached;
        base::samples::RigidBodyState initial_pose;
        base::samples::RigidBodyState initial_imu;
        int timeout_motions;
        double WisdomDistance;
        int WisdomAcqTime;
        int WisdomTimer;
        bool WisdomAcquiring;
        locomotion_switcher::LocomotionMode locomotion_mode;

        // MAST_PTU_MoveTo parameters
        double pan;
        double tilt;
        double panorama_tilt;
        bool MastDeployed;

        // PanCam_WACGetImage parameters
        char cam[80], dummy_param[80];
        std::string image_filename;
        std::string dem_filename;
        std::string dist_filename;
        messages::Telemetry tm_in;
        messages::ProductType productType;
        messages::Mode productMode;

        // PanCam_WAC_RRGB parameters
        int inPanCamActivity;

        //ACTIVE MQ
        ActiveMQTMSender* activemqTMSender;
        ActiveMQAdmin* activemqAdmin;

        CommTmServer* tmComm;
        CommTcServer* tcComm;
        CommTcReplyServer* tcReplyServer;

        base::commands::Motion2D motion_command;
        base::commands::Joints ptu_command;
        double bema_command;
        base::samples::RigidBodyState pose;
        base::samples::Joints ptu;
        base::samples::Joints bema;
        base::samples::Joints joint_samples;
        temperature::samples::Temperature motor_temperatures;
        base::samples::RigidBodyState imu;
        base::samples::RigidBodyState initial_3Dpose;
        base::samples::RigidBodyState absolute_pose;
        double initial_absolute_heading;
        int tj_status;

        //FIXME Dirty fix to allow executing commands while executing motion commands
        bool isActiveACKERMANNGOTO = false;
        bool isActiveTURNSPOTGOTO = false;
        bool isActiveTRAJECTORY = false;

        // Dead Man Switch
        // if false, the time since the last direct command will be compared to the dead man time.
        // if too much time has elapsed, a stop motion command will be sent and deadMan will be set to true.
        bool deadMan = false;
        base::Time lastDirectCommandTime;
        int deadManTime = 2e6; //[usec]
        // ignore dead man switch if direct activities were never used
        bool deadManSwitchRelevant = false;
        // deadManSwitch updates the lastDirectCommandTime, resets deadMan to false, and sets dmsRelevant to true
        // this function is called after each direct command
        void deadManSwitch();

        void execTelecommand(std::string);

        public:
        Task(std::string const& name = "telemetry_telecommand::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
        ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

        /** this method computes the absolute travelled distance in 3d. initial position
         * from which the distance is calculated is reset every time a locomotion command
         * is sent. the initial position is compared to the current position to calculate the
         * travelled distance.
         */
        double getTravelledDistance();

        /** this method computes the absolute travelled angle (in radians). initial orientation
         * from which the motion is calculated is reset every time a locomotion command
         * is sent. the initial orientation is compared to the current orientation to calculate the
         * travelled angle.
         */
        bool angleReached();

        void getTransform(Eigen::Affine3d& tf);

        /** Checks if a bema move command has reached its target.
         */
        bool bema1TargetReached();

        /** Checks if a walking (egress front) move command has reached its target.
         */
        bool bema2TargetReached();

        /** Checks if a walking (egress rear) move command has reached its target.
         */
        bool bema3TargetReached();

        /** Checks if a ptu move command has reached its target.
         */
        bool ptuTargetReached();

        /** Sends the PTU command through the ptu_command port. The values of pan and tilt
         *  need to be previously set
         */
        void sendPtuCommand();

        /** Sends the Motion command through the motion_command port. The values of translation and
         *  rotation speed need to be previously set
         */
        void sendMotionCommand();

        /** Sends the file (image, distance, etc.) specified in the tm message.
         *  It clasifies the file depending its type and source
         */
        void sendProduct(messages::Telemetry tm_in);

        /** Stop rover in case the direct command can be assumed to have crashed.
         * This is mainly used when controlling the rover's velocity with a
         * joystick while in trajectory following mode.
         */
        void checkDeadManSwitch();

        /** Reads input ports to update odometry, robot state, ... */
        void reactToInputPorts();

        /** Reads incoming telecommands (if any) and executes (or simulates) them */
        void getAndExecTelecommand();

        /** Checks which activities are running and continues executing them */
        void controlRunningActivities();

        private:

        void exec_GNC_ACKERMANN_GOTO(CommandInfo*);
        void exec_GNC_WHEELWALK_GOTO(CommandInfo*);
        void exec_GNC_LLO(CommandInfo*);
        void exec_GNC_TURNSPOT_GOTO(CommandInfo*);
        void exec_GNC_TRAJECTORY(CommandInfo*);
        void exec_GNC_TRAJECTORY_WISDOM(CommandInfo*);
        void exec_MAST_PTU_MOVE_TO(CommandInfo*);
        void exec_DEPLOY_MAST(CommandInfo*);
        void exec_PANCAM_PANORAMA(CommandInfo*);
        void exec_TOF_ACQ(CommandInfo*);
        void exec_LIDAR_ACQ(CommandInfo*);
        void exec_DEPLOYMENT_ALL(CommandInfo*);
        void exec_DEPLOYMENT_FRONT(CommandInfo*);
        void exec_DEPLOYMENT_REAR(CommandInfo*);
        void exec_GNC_UPDATE(CommandInfo*);
        void exec_GNC_ACKERMANN_DIRECT(CommandInfo*);
        void exec_GNC_TURNSPOT_DIRECT(CommandInfo*);
        void exec_ALL_ACQ(CommandInfo*);
        void exec_HAZCAM_ACQ(CommandInfo*);
        void exec_LOCCAM_ACQ(CommandInfo*);
        void exec_GNCG(CommandInfo*);
        void exec_ABORT(CommandInfo*);
        void exec_MAST_ACQ(CommandInfo*);
        void exec_PANCAM_ACQ(CommandInfo*);
        void exec_FRONT_ACQ(CommandInfo*);
        void exec_NAVCAM_ACQ(CommandInfo*);
        void exec_REAR_ACQ(CommandInfo*);

        bool ctrl_GNC_ACKERMANN_GOTO();
        bool ctrl_GNC_WHEELWALK_GOTO();
        bool ctrl_GNC_LLO();
        bool ctrl_GNC_TURNSPOT_GOTO();
        bool ctrl_GNC_TRAJECTORY();
        bool ctrl_GNC_TRAJECTORY_WISDOM();
        bool ctrl_MAST_PTU_MOVE_TO();
        bool ctrl_DEPLOY_MAST();
        bool ctrl_PANCAM_PANORAMA();
        bool ctrl_TOF_ACQ();
        bool ctrl_LIDAR_ACQ();
        bool ctrl_DEPLOYMENT_ALL();
        bool ctrl_DEPLOYMENT_FRONT();
        bool ctrl_DEPLOYMENT_REAR();
        bool ctrl_GNC_UPDATE();
        bool ctrl_ALL_ACQ();
        bool ctrl_HAZCAM_ACQ();
        bool ctrl_LOCCAM_ACQ();
        bool ctrl_MAST_ACQ();
        bool ctrl_PANCAM_ACQ();
        bool ctrl_NAVCAM_ACQ();
        bool ctrl_FRONT_ACQ();
        bool ctrl_REAR_ACQ();

        std::map<const std::string, std::tuple<int, int, std::function<void(CommandInfo*)>, std::function<bool(void)> > > tc_map;
        int getTimer(const string cmd_str);
        void setTimer(const string cmd_str, const int val);
        void setTimeout(const string cmd_str, const int val);
        void initializeTimer(const string cmd_str);
        std::function< void(CommandInfo*) > getExecFunction(const string cmd_str);
        std::function< bool(void) > getControlFunction(const string cmd_str);
    };
}

#endif
