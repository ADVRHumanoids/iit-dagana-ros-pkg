#include <dagana_gazebo/DaganaPlugin.h>

void gazebo::DaganaPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("ROS not initialized");
        return;
    }
    
    this->sdf = sdf;
    
    if (! getSdfElements()) {
        return;
    }
    
    // Store the model pointer for convenience.
    this->model = model;
    scopedJointName = model->GetName() + "::" + joint_name; //necessary for Set of gazebo

    dt = 1.0/rate;
    model->GetJointController()->SetPositionPID(
                scopedJointName, common::PID(p, i, d, 1, -1, cmdMax, cmdMin));
    
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
    
    if (! model->GetJoint(scopedJointName)) {
        ROS_ERROR("DAGANA PLUGIN ERROR: '%s' joint not found\n", scopedJointName.c_str());
        return;
    }
    pos_ref_interpolated = model->GetJoint(scopedJointName)->Position(0);
    
    //init message
    jointStateMsg.name.push_back(joint_name);
    jointStateMsg.position.push_back(model->GetJoint(scopedJointName)->Position(0));
    jointStateMsg.velocity.push_back(model->GetJoint(scopedJointName)->GetVelocity(0));
    jointStateMsg.effort.push_back(model->GetJoint(scopedJointName)->GetForce(0));
    
    jointCommandMsg = jointStateMsg;
    
    pos_min = model->GetJoint(scopedJointName)->LowerLimit(0);
    pos_max = model->GetJoint(scopedJointName)->UpperLimit(0);
    vel_max = model->GetJoint(scopedJointName)->GetVelocityLimit(0);
    
    this->rosQueueThread = std::thread(std::bind(&DaganaPlugin::QueueThread, this, rate));
}

void gazebo::DaganaPlugin::QueueThread(double rate) {
    
    ros::Rate r(rate); 
    while (nh->ok()) {

        pubJointState();
        // see if some messages for subs have arrived
        this->rosQueue.callAvailable(ros::WallDuration());        

        setReference();

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

/************************************************************************************************** ******************************* */
          // this will not consider collision and will make everyhing explode
//         model->GetJoint(scopedJointName)->SetPosition(0, pos_ref_interpolated);
 
 /************************************************************************************************** ******************************* */
        model->GetJointController()->SetPositionTarget (scopedJointName, pos_ref_interpolated );
        //model->GetJointController()->SetVelocityTarget (scopedJointName, vel_max );
        
//         auto pid = model->GetJointController()->GetPositionPIDs().at(scopedJointName);
//         std::cout << "PID" << pid.GetPGain() << " " << pid.GetIGain() << " " << pid.GetDGain() << " " <<
//             "cmd " << pid.GetCmdMin() << " " << pid.GetCmdMax() <<
//             "int " << pid.GetIMin() << " " << pid.GetIMax() 
//             << std::endl;

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
}

bool gazebo::DaganaPlugin::getSdfElements() {
    
    if (sdf->HasElement("gripperName")) 
    {
        sdf->GetElement("gripperName")->GetValue()->Get(gripper_name);

    } else {
        
        ROS_ERROR("sdf element 'gripperName' not found\n");
        return false;
    }
    
    if (sdf->HasElement("jointName")) 
    {
        sdf->GetElement("jointName")->GetValue()->Get(joint_name);

    } else {
        
        ROS_ERROR("sdf element 'jointName' not found\n");
        return false;
        
    }
    
    if (sdf->HasElement("p")) 
    {
        sdf->GetElement("p")->GetValue()->Get(p);

    } else {
        
        ROS_ERROR("sdf element 'p' (for pid) not found\n");
        return false;
        
    }
    if (sdf->HasElement("i")) 
    {
        sdf->GetElement("i")->GetValue()->Get(i);

    } else {
        
        ROS_ERROR("sdf element 'i' (for pid) not found\n");
        return false;
        
    }
    if (sdf->HasElement("d")) 
    {
        sdf->GetElement("d")->GetValue()->Get(d);

    } else {
        
        ROS_ERROR("sdf element 'd' (for pid) not found\n");
        return false;
        
    }
    
    if (sdf->HasElement("cmdMax")) 
    {
        sdf->GetElement("cmdMax")->GetValue()->Get(cmdMax);

    } else {
        
        ROS_ERROR("sdf element 'cmdMax' (for pid) not found\n");
        return false;
        
    }
    
    if (sdf->HasElement("cmdMin")) 
    {
        sdf->GetElement("cmdMin")->GetValue()->Get(cmdMin);

    } else {
        
        ROS_ERROR("sdf element 'cmdMin' (for pid) not found\n");
        return false;
        
    }
    
    if (sdf->HasElement("rate")) 
    {
        sdf->GetElement("rate")->GetValue()->Get(rate);

    } else {
        
        ROS_ERROR("sdf element 'gripperName' not found\n");
        return false;
    }
    
    return true;

}
