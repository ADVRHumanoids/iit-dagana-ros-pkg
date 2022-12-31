#ifndef _DAGANA_PLUGIN_H_
#define _DAGANA_PLUGIN_H_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
    /**
    * 
    */
    class DaganaPlugin : public ModelPlugin {
    
    public: 
        DaganaPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        
     
    private:
        
        physics::ModelPtr model;
        sdf::ElementPtr sdf;
        
        std::string scopedJointName;
        double pos_min;
        double pos_max;
        double vel_max;
        std::string gripper_name;
        std::string joint_name;
        double rate;
        double p, i, d, cmdMax, cmdMin;
        
        std::unique_ptr<ros::NodeHandle> nh;
        ros::CallbackQueue rosQueue;
        void QueueThread(double rate);
        std::thread rosQueueThread;
        
        void pubJointState();
        bool setReference();
        double pos_ref_interpolated;
        double dt;
        
        ros::Publisher joint_state_pub;
        sensor_msgs::JointState jointStateMsg;

        ros::Subscriber joint_command_sub;
        void jointCommandClbk ( const sensor_msgs::JointStateConstPtr &msg );
        sensor_msgs::JointState jointCommandMsg;
        
        bool getSdfElements();
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DaganaPlugin)
}

#endif // _DAGANA_PLUGIN_H_

