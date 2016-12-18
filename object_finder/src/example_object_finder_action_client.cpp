// example_object_finder_action_client: 
// wsn, April, 2016
// illustrates use of object_finder action server called "objectFinderActionServer"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <coordinator/ManipTaskAction.h>
#include <object_grabber/object_grabberAction.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

geometry_msgs::PoseStamped g_perceived_object_pose;
ros::Publisher *g_pose_publisher;
bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code = 0;
int g_object_finder_return_code = 0;
int g_fdbk_count = 0;

geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
geometry_msgs::PoseStamped g_object_pose;
coordinator::ManipTaskResult g_result;

int g_found_object_code;
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
        g_perceived_object_pose= result->object_pose;
        ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                g_perceived_object_pose.pose.position.y,
                g_perceived_object_pose.pose.position.z);

        ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                g_perceived_object_pose.pose.orientation.y,
                g_perceived_object_pose.pose.orientation.z,
                g_perceived_object_pose.pose.orientation.w);
        g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}


//manipulation callbacks
void manipDoneCb(const actionlib::SimpleClientGoalState& state,
        const coordinator::ManipTaskResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_goal_done = true;
    g_result = *result;
    g_callback_status = result->manip_return_code;

    switch (g_callback_status) {
        case coordinator::ManipTaskResult::MANIP_SUCCESS:
            ROS_INFO("returned MANIP_SUCCESS");

            break;

        case coordinator::ManipTaskResult::FAILED_PERCEPTION:
            ROS_WARN("returned FAILED_PERCEPTION");
            g_object_finder_return_code = result->object_finder_return_code;
            break;
        case coordinator::ManipTaskResult::FAILED_PICKUP:
            ROS_WARN("returned FAILED_PICKUP");
            g_object_grabber_return_code = result->object_grabber_return_code;
            g_object_pose = result->object_pose;
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
            break;
        case coordinator::ManipTaskResult::FAILED_DROPOFF:
            ROS_WARN("returned FAILED_DROPOFF");
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;          
            break;
    }
}

//optional feedback; output has been suppressed (commented out) below
void manipFeedbackCb(const coordinator::ManipTaskFeedbackConstPtr& fdbk_msg) {
    g_fdbk_count++;
    if (g_fdbk_count > 1000) { //slow down the feedback publications
        g_fdbk_count = 0;
        //suppress this feedback output
        ROS_INFO("feedback status = %d", fdbk_msg->feedback_status);
    }
    //g_fdbk = fdbk_msg->feedback_status; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but possibly useful for diagnostics
void manipActiveCb() {
    ROS_INFO("Goal just went active");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_finder_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("object_finder_action_service", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for object finder server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    object_finder::objectFinderGoal object_finder_goal;
    //object_finder::objectFinderResult object_finder_result;

    object_finder_goal.object_id = ObjectIdCodes::TABLE_SURFACE;
    object_finder_goal.known_surface_ht = false; //require find table height
    //object_finder_goal.object_id=object_finder::objectFinderGoal::COKE_CAN_UPRIGHT;
    //object_finder_goal.object_id=object_finder::objectFinderGoal::TOY_BLOCK;
    //object_finder_goal.known_surface_ht=true;
    //object_finder_goal.known_surface_ht=false; //require find table height
    //object_finder_goal.surface_ht = 0.05;
    double surface_height;
    ROS_INFO("sending goal: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        
        bool finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND) {
                        ROS_INFO("surface-finder success");
                        surface_height = g_perceived_object_pose.pose.position.z; // table-top height, as found by object_finder 
                        ROS_INFO("found table ht = %f",surface_height);   }
    else {
        ROS_WARN("did not find table height; quitting:");
        return 1;
    }



    //define sweep endpoint poses; 
    //right sweep
    geometry_msgs::PoseStamped right_sweep_pose;
    right_sweep_pose.header.frame_id = "torso";
    right_sweep_pose.pose.position.x = 0.65;
    right_sweep_pose.pose.position.y = -0.35; //-0.35;
    right_sweep_pose.pose.position.z = -0.3;
    right_sweep_pose.pose.orientation.x = 0.707;
    right_sweep_pose.pose.orientation.y = 0.707;
    right_sweep_pose.pose.orientation.z = 0;
    right_sweep_pose.pose.orientation.w = 0;
    right_sweep_pose.header.stamp = ros::Time::now(); 

    //left sweep
    geometry_msgs::PoseStamped left_sweep_pose;
    left_sweep_pose.header.frame_id = "torso";
    left_sweep_pose.pose.position.x = 0.6;
    left_sweep_pose.pose.position.y = 0.25; //-0.35;
    left_sweep_pose.pose.position.z = -0.35;
    left_sweep_pose.pose.orientation.x = 0.507;
    left_sweep_pose.pose.orientation.y = 0.407;
    left_sweep_pose.pose.orientation.z = 0;
    left_sweep_pose.pose.orientation.w = 0;
    left_sweep_pose.header.stamp = ros::Time::now(); 

    //make an action client for the manipulation task
    coordinator::ManipTaskGoal manip_goal;
    coordinator::ManipTaskGoal cube_goal;
    coordinator::ManipTaskGoal sweep_goal;
    coordinator::ManipTaskGoal lsweep_goal;

    actionlib::SimpleActionClient<coordinator::ManipTaskAction> manipulation_ac("manip_task_action_service", true);
    ROS_INFO("waiting for manipulation server: ");
    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = manipulation_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    //record surface height
    object_finder_goal.known_surface_ht = true;
    object_finder_goal.surface_ht = surface_height;
    bool found_cube = false;
    bool found_block = false;

    ROS_INFO("using surface ht = %f",surface_height);     
    geometry_msgs::PoseStamped cube_pose;
    geometry_msgs::PoseStamped block_pose;

    int ans = 0;
    while(ros::ok()){

        ROS_INFO("ready to go to wait pose; enter 1 when you are ready, or 0 to stop: ");
        std::cin>>ans;
        if (ans == 0) return 0; 

        //send arm to waiting pose
        ROS_INFO("sending a goal: move to pre-pose");
        g_goal_done = false;
        manip_goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
        manipulation_ac.sendGoal(manip_goal, &manipDoneCb, &manipActiveCb, &manipFeedbackCb);
        while (!g_goal_done) {
            ros::Duration(0.1).sleep();
        }
        if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS) {
            ROS_ERROR("failed to move quitting");
            return 0;
        }

        ROS_INFO("ready to look for block; enter 1 when you are ready, or 0 to stop: ");
        std::cin>>ans;
        if (ans == 0) return 0;   

        object_finder_goal.object_id=ObjectIdCodes::CUBE_ID;
        ROS_INFO("sending goal to find CUBE: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }       
        
        if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)   {
            ROS_INFO("found cube!");
            cube_pose = g_perceived_object_pose;
            found_cube = true;
        }    
         else {
            ROS_WARN("cube not found!:");
        }

        if(!found_cube){
            object_finder_goal.object_id=ObjectIdCodes::TOY_BLOCK_ID;
            ROS_INFO("sending goal to find TOY_BLOCK: ");
            object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
            finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result ");
                return 1;
            }       
            
            if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)   {
                ROS_INFO("found block!");
                block_pose = g_perceived_object_pose;
                found_block = true;
            }    
             else {
                ROS_WARN("block not found!:");
            }
        }

        if(found_cube){
            //sweep it right

            //print some information for debugging
            ROS_INFO_STREAM("cube pose w/rt frame-id " << cube_pose.header.frame_id << endl);
            ROS_INFO_STREAM("object origin: (x,y,z) = (" << cube_pose.pose.position.x << ", " << cube_pose.pose.position.y << ", "
                    << cube_pose.pose.position.z << ")" << endl);
            ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = (" << cube_pose.pose.orientation.x << ","
                    << cube_pose.pose.orientation.y << ","
                    << cube_pose.pose.orientation.z << ","
                    << cube_pose.pose.orientation.w << ")" << endl);

            //send command to acquire block:
            ROS_INFO("sending a goal: straddle cube");
            g_goal_done = false;
            cube_goal.action_code = coordinator::ManipTaskGoal::STRADDLE_OBJECT;
            cube_pose.pose.position.z -= 0.05;
            cube_goal.pickup_frame = cube_pose;
            cube_goal.object_code = ObjectIdCodes::TOY_BLOCK_ID;
            manipulation_ac.sendGoal(cube_goal, &manipDoneCb, &manipActiveCb, &manipFeedbackCb);
            while (!g_goal_done) {
                ros::Duration(0.1).sleep();
            }
            if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS) {
                ROS_ERROR("failed to straddle cube; ");
            }

            ROS_INFO("sending a goal: sweep cube");
            g_goal_done = false;
            lsweep_goal.action_code = coordinator::ManipTaskGoal::CART_MOVE_TO_GRIPPER_POSE;
            lsweep_goal.gripper_goal_frame = left_sweep_pose;
            manipulation_ac.sendGoal(lsweep_goal, &manipDoneCb, &manipActiveCb, &manipFeedbackCb);
            while (!g_goal_done) {
                ros::Duration(0.1).sleep();
            }
            if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS) {
                ROS_ERROR("failed to sweep cube; ");
            }
            else{
                ROS_INFO("Swept!!!!");
            }

        } else if(found_block) {
            //sweep it left

            //print some information for debugging
            ROS_INFO_STREAM("block pose w/rt frame-id " << block_pose.header.frame_id << endl);
            ROS_INFO_STREAM("object origin: (x,y,z) = (" << block_pose.pose.position.x << ", " << block_pose.pose.position.y << ", "
                    << block_pose.pose.position.z << ")" << endl);
            ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = (" << cube_pose.pose.orientation.x << ","
                    << block_pose.pose.orientation.y << ","
                    << block_pose.pose.orientation.z << ","
                    << block_pose.pose.orientation.w << ")" << endl);

            //send command to acquire block:
            ROS_INFO("sending a goal: straddle block");
            g_goal_done = false;
            manip_goal.action_code = coordinator::ManipTaskGoal::STRADDLE_OBJECT;
            block_pose.pose.position.z -= 0.08;
            manip_goal.pickup_frame = block_pose;
            manip_goal.object_code = ObjectIdCodes::TOY_BLOCK_ID;
            manipulation_ac.sendGoal(manip_goal, &manipDoneCb, &manipActiveCb, &manipFeedbackCb);
            while (!g_goal_done) {
                ros::Duration(0.1).sleep();
            }
            if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS) {
                ROS_ERROR("failed to straddle block; ");
            }

            ROS_INFO("sending a goal: sweep block");
            g_goal_done = false;
            sweep_goal.action_code = coordinator::ManipTaskGoal::CART_MOVE_TO_GRIPPER_POSE;
            sweep_goal.gripper_goal_frame = right_sweep_pose;
            manipulation_ac.sendGoal(sweep_goal, &manipDoneCb, &manipActiveCb, &manipFeedbackCb);
            while (!g_goal_done) {
                ros::Duration(0.1).sleep();
            }
            if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS) {
                ROS_ERROR("failed to sweep cube; ");
            }
            else{
                ROS_INFO("Swept!!!!");
            }

        } else {
            //dab
        }

        found_cube = false;
        found_block = false;
    }
}

