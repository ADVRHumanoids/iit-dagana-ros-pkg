#include <dagana_gazebo/DaganaPlugin.h>

void gazebo::DaganaPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("ROS not initialized");
        return;
    }
    
    if (sdf->HasElement("gripperName")) 
    {
        sdf->GetElement("gripperName")->GetValue()->Get(gripper_name);

    } else {
        
        ROS_ERROR("sdf element 'gripperName' not found\n");
        return;
    }
    
    if (sdf->HasElement("jointName")) 
    {
        sdf->GetElement("jointName")->GetValue()->Get(joint_name);

    } else {
        
        ROS_ERROR("sdf element 'gripperName' not found\n");
        return;
        
    }
    if (sdf->HasElement("rate")) 
    {
        sdf->GetElement("rate")->GetValue()->Get(rate);

    } else {
        
        ROS_ERROR("sdf element 'gripperName' not found\n");
        return;
    }
    dt = 1.0/rate;
    
    // Store the model pointer for convenience.
    this->model = model;
    
    // Create a ROS node.
    this->nh.reset(new ros::NodeHandle(""));
    
    // Create a named topic, and subscribe to it.
    //TODO take namespace from somewhere
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/xbotcore/gripper/" + gripper_name + "/command",
        10,
        boost::bind(&DaganaPlugin::jointCommandClbk, this, _1),
        ros::VoidPtr(), &this->rosQueue);
        
    joint_command_sub = nh->subscribe(so);
    
    joint_state_pub = nh->advertise<sensor_msgs::JointState>("/xbotcore/gripper/" + gripper_name + "/state", 10);

    this->rosQueueThread = std::thread(std::bind(&DaganaPlugin::QueueThread, this, rate));
    
    scopedJointName = model->GetName() + "::" + joint_name; //necessary for Set of gazebo
    pos_ref_interpolated = model->GetJoint(scopedJointName)->Position(0);
    
    //init message
    jointStateMsg.name.push_back(joint_name);
    jointStateMsg.position.push_back(model->GetJoint(scopedJointName)->Position(0));
    jointStateMsg.velocity.push_back(model->GetJoint(scopedJointName)->GetVelocity(0));
    jointStateMsg.effort.push_back(model->GetJoint(scopedJointName)->GetForce(0));
    
    pos_min = model->GetJoint(scopedJointName)->LowerLimit(0);
    pos_max = model->GetJoint(scopedJointName)->UpperLimit(0);
    vel_max = model->GetJoint(scopedJointName)->GetVelocityLimit(0);
    
    
}

void gazebo::DaganaPlugin::QueueThread(double rate) {
    
    ros::Rate r(rate); 
    while (nh->ok()) {

        pubJointState();
        // see if some messages for subs have arrived
        this->rosQueue.callAvailable(ros::WallDuration());        

        if (command_arrived)
        {
            setReference();
            command_arrived = false;
        }
        r.sleep();
    }
}

void gazebo::DaganaPlugin::pubJointState ( ) {

    jointStateMsg.header.stamp = ros::Time::now();

#if GAZEBO_MAJOR_VERSION >= 8
    jointStateMsg.position[0] = model->GetJoint(scopedJointName)->Position(0); //index 0 is the "right" axis
#else
    jointStateMsg.position[0] = model->GetJoint(scopedJointName)->GetAngle(0).Radian(); //index 0 is the "right" axis
#endif
    jointStateMsg.velocity[0] = model->GetJoint(scopedJointName)->GetVelocity(0); //index 0 is the "right" axis
    jointStateMsg.effort[0] = model->GetJoint(scopedJointName)->GetForce(0); //index 0 is the "right" axis
    
    joint_state_pub.publish(jointStateMsg);
}

bool gazebo::DaganaPlugin::setReference (  )
{        
    
    if(jointCommandMsg.position.size() == 1)
    {
        
        double pos_ref_candidate = pos_min + jointCommandMsg.position[0] * (pos_max - pos_min);
        pos_ref_interpolated = std::max(std::min(pos_ref_interpolated + vel_max*dt,
                                     pos_ref_candidate),
                                     pos_ref_interpolated - vel_max*dt);
        
        //model->getJoint(scopedJointName).setForce();
        model->GetJoint(scopedJointName)->SetPosition(0, pos_ref_interpolated);

    }
    else
    {
        ROS_ERROR("Joint command: wrong size for position field! Expected: 1, Actual %ld\n", jointCommandMsg.position.size());
        return false;
    }
    
    return true;

}

void gazebo::DaganaPlugin::jointCommandClbk ( const sensor_msgs::JointStateConstPtr &msg ) {
    
    jointCommandMsg = *msg;
    command_arrived = true;
}
