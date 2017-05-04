/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

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

#include "telemetry_telecommand/Messages.hpp"
#include "temperature/temperatureTypes.hpp"
//#include "frame_helper/FrameHelper.h"

namespace telemetry_telecommand {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the telemetry_telecommand namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','telemetry_telecommand::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        int currentActivity;
        bool abort_activity;
        bool files_sent;
        std::string currentParams;
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
        int NofWaypoints;
        base::samples::RigidBodyState initial_pose;
        base::samples::RigidBodyState initial_imu;
        
        // MAST_PTU_MoveTo parameters
        double pan;
        double tilt;

        // PanCam_WACGetImage parameters
        char cam[80], dummy_param[80];
        std::string image_filename;
        std::string dem_filename;
        std::string dist_filename;
        messages::Telemetry tm_in;
        messages::Telecommand tc_out;
        messages::ProductType productType;
        messages::Mode productMode;

        // PanCam_WAC_RRGB parameters
        int inPanCamActivity;
	
	//ACTIVE MQ
        ActiveMQTMSender* activemqTMSender;
        ActiveMQTCReceiver* activemqTCReceiver;
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
        //bool first_estimate;
        //double first_imu_estimate_yaw;

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

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "telemetry_telecommand::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
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
        double getTravelledAngle();
        bool angleReached();

        void getTransform(Eigen::Affine3d& tf);

        /** Calculates the parameters to be sent as motion commands (2D). Translation (m/s) and Rotation (rad/s)
         * are calculated.
         */
        void motionCommand();


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

    };
}

#endif

