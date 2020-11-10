#include "calibrator.h"

// Constructor
Calibrator::Calibrator(ros::NodeHandle nh)
    : nh_(nh), listener_(new(tf::TransformListener)), broadcaster_(new(tf::TransformBroadcaster)), refresh_rate_(default_refresh_rate), calib_pos_count_(0), arm_position_(1), transform_set_(0), sealer_transform_set_(0), fridge_transform_set_(0), cumulative_rot_x_(0.0), cumulative_rot_y_ (0.0), cumulative_rot_z_(0.0), cumulative_rot_w_(0.0), cumulative_trans_x_(0.0), cumulative_trans_y_ (0.0), cumulative_trans_z_(0.0)
{
    sub1_ = nh_.subscribe("ar_pose_marker", 1000, &Calibrator::trackerCallback, this);


}

// Deconstructor
Calibrator::~Calibrator()
{

}

// AR Tag Tracking Node Callback
void Calibrator::trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{

    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

    // Update member variable with most recent tag pose data
    marker_pose_ = *msg;

    marker_pose_lock.unlock();
    
}
    
// Function run by another thread to broadcast the Base to Camera Transform continuously upon setting the transform - this makes it easier to visualise the output from the calibration process in real time
void Calibrator::broadcastBaseCameraTransform()
{
    // Set rate limiter to broadcast transform every 2 seconds
    ros::Rate rate_limiter(0.5);

    while(ros::ok()){
        // Wait until the transform has been set the first time
	if(transform_set_ < 1){
	

        } else {
            // Create unique lock for safe access to member variables
            std::unique_lock<std::mutex> base_camera_transform_lock(base_camera_mutex_);

	    // Store transform in temp variable
            tf::Transform broadcastTransform = average_base_camera_transform_;

            base_camera_transform_lock.unlock();

	    // Broadcast transform
            broadcaster_->sendTransform(tf::StampedTransform(broadcastTransform, ros::Time::now(), "base_link", "camera_base_link"));
        }
	rate_limiter.sleep();

    }
    

}

// Function run by another thread to broadcast the Tag to Target transform continuously upon setting the transform - this makes it easier to visualise the output from the calibration process in real-time
void Calibrator::broadcastTagTargetTransform()
{
    // Set rate limiter to broadcast transform every 2 seconds
    ros::Rate rate_limiter(0.5);

    while(ros::ok()){

	// Create unique mutex lock to update shared member variables
	std::unique_lock<std::mutex> camera_tag_transform_lock(camera_tag_transform_mutex_);

	tf::Transform broadcastTransform1 = camera_tag_transform_;
	camera_tag_transform_lock.unlock();

        // Wait until the transform has been set the first time
	if(sealer_transform_set_ < 1){
	

        } else {
            // Create unique lock for safe access to member variables
            std::unique_lock<std::mutex> tag_target_transform_lock(tag_target_mutex_);

	    // Store transform in temp variable
            tf::Transform broadcastTransform2 = average_tag_target_transform_;

            tag_target_transform_lock.unlock();

	    broadcaster_->sendTransform(tf::StampedTransform(broadcastTransform1, ros::Time::now(), "camera_color_optical_frame", "sealer_tag"));
	    broadcaster_->sendTransform(tf::StampedTransform(broadcastTransform2, ros::Time::now(), "sealer_tag", "sealer_link"));
        }


        if(fridge_transform_set_ < 1){


        } else {

	    // Create unique lock for safe access to member variables
            std::unique_lock<std::mutex> tag_target_transform_lock(tag_target_mutex_);

	    // Store transform in temp variable
            tf::Transform broadcastTransform2 = average_tag_target_transform_;

            tag_target_transform_lock.unlock();

	    broadcaster_->sendTransform(tf::StampedTransform(broadcastTransform1, ros::Time::now(), "camera_color_optical_frame", "fridge_tag"));
	    broadcaster_->sendTransform(tf::StampedTransform(broadcastTransform2, ros::Time::now(), "fridge_tag", "fridge_link"));


        }
	rate_limiter.sleep();

    }

}


 
// Main Calibrate Camera Function
/*
 * Function obtains the relative camera to calibration tag transform from the Alvar ROS package.
 * Using the kinematic loop, the function then mutiplies through the kinematic loop to calculate the relative base to camera transform
 * Both the calculated base to camera transform and the rolling average base to camera transforms are printed to the terminal for the user's analysis and interpretation
 * The rolling average base to camera transform is then broadcasted to the TF tree for visualisation purposes.
 * Upon completing the calibration process, the values obtained for the average base to camera transform should be added to the tf_broadcaster.cpp file.
*/
void Calibrator::calibrateCamera()
{
    while(ros::ok()) {
        // While the number of calibration positions is less than 10, loop.
        if(calib_pos_count_ < 10) {
            

/*
            // Ask user what position to drive arm to
            std::cout << "Select which Arm Position to Drive KUKA Arm to: ";
            while(!(std::cin >> arm_position_)){
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input, Try again: ";
            }
           
  */                 
	    // Display output to user
            std::cout << "Driving Robot Arm to Position: " << (calib_pos_count_ + 1) << std::endl;     
            // Drive robot arm to position
            
	    //**********************************************************************//
	    //*		NEED TO WORK OUT HOW TO IMPLEMENT THE ARM DRIVE FUNCTION   *//
	    //**********************************************************************//
            

            
            // Prompt user to hit enter once robot arm has stopped moving
            std::cout << "Press Space Bar then Enter when Robot Arm Stops Moving to Continue Calibration... ";
	    std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
            

            // Get position of tag w.r.t. camera from Alvar package
	    // Create unique lock for safe access to member variables
            std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

            // Update member variable with most recent pose data
            marker_pose_for_calc_ = marker_pose_;

            marker_pose_lock.unlock();

            geometry_msgs::PoseStamped sensor_pose;
            float timestamp = 0.0;
	    tf::Transform camera_tag_transform;
          
	    // Iterate through all the markers that can be seen by the camera
            for (int i = 0; i < marker_pose_for_calc_.markers.size(); i++) {
                if (marker_pose_for_calc_.markers.at(i).id == 8) {
		    // Debugging line
                    ROS_INFO("I see the calibration marker!");

		    // Store the pose data in variable
		    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
		    // Store reference frame id of camera in variable
		    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

		    // Create transform variable of camera to tag transform using the data from the Alvar node
		    camera_tag_transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                    camera_tag_transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));


    		    // Create a transform of the camera_base_link to the camera's reference frame used in Alvar node
		    tf::StampedTransform camera_internal_transform;
		    listener_->lookupTransform("camera_base_link", sensor_pose.header.frame_id, ros::Time(0), camera_internal_transform);


		    // Get position of tag w.r.t. end effector from kinematic chain
		    tf::StampedTransform tag_arm_transform;
	    	    listener_->lookupTransform("link_7", "tool0", ros::Time(0), tag_arm_transform);
		


	   	    // Get position of link_1 w.r.t. base from kinematic chain
	    	    tf::StampedTransform arm_base1_transform;
	    	    listener_->lookupTransform("base_link", "link_1", ros::Time(0), arm_base1_transform);

		    // Get position of link_2 w.r.t. link_1 from kinematic chain
		    tf::StampedTransform arm_base2_transform;
	    	    listener_->lookupTransform("link_1", "link_2", ros::Time(0), arm_base2_transform);
		    
		    // Get position of link_3 w.r.t. link_2 from kinematic chain
		    tf::StampedTransform arm_base3_transform;
	    	    listener_->lookupTransform("link_2", "link_3", ros::Time(0), arm_base3_transform);

		    // Get position of link_4 w.r.t. link_3 from kinematic chain
		    tf::StampedTransform arm_base4_transform;
	    	    listener_->lookupTransform("link_3", "link_4", ros::Time(0), arm_base4_transform);

		    // Get position of link_5 w.r.t. link_4 from kinematic chain
		    tf::StampedTransform arm_base5_transform;
	    	    listener_->lookupTransform("link_4", "link_5", ros::Time(0), arm_base5_transform);

		    // Get position of link_6 w.r.t. link_5 from kinematic chain
		    tf::StampedTransform arm_base6_transform;
	    	    listener_->lookupTransform("link_5", "link_6", ros::Time(0), arm_base6_transform);

		    // Get position of link_7 w.r.t. link_6 from kinematic chain
		    tf::StampedTransform arm_base7_transform;
	    	    listener_->lookupTransform("link_6", "link_7", ros::Time(0), arm_base7_transform);


	            // Calculate relative transform between base and camera by multiplying along the kinematic chain from base through to the camera
		    tf::Transform calc_transform;
	  	    calc_transform.mult(arm_base1_transform, arm_base2_transform);
		    calc_transform *= arm_base3_transform;
		    calc_transform *= arm_base4_transform;
		    calc_transform *= arm_base5_transform;
		    calc_transform *= arm_base6_transform;
		    calc_transform *= arm_base7_transform;
		    calc_transform *= tag_arm_transform;
   
		    // Create temp transform to store the result from camera_base_link -> tag
		    tf::Transform temp_transform;
	 	    temp_transform.mult(camera_internal_transform, camera_tag_transform);

		    // Inverse transform as we want tag -> camera_base_link
	            temp_transform = temp_transform.inverse();

                    calc_transform *= temp_transform;

		    tf::Transform base_camera_transform;

		    // Store relative transform from base->camera in variable
		    base_camera_transform = calc_transform;


		    // Solely used in Debugging Printing Statements, not needed for functionality
		    tf::Matrix3x3 base_camera_rotation = base_camera_transform.getBasis();
		    tf::Vector3 base_camera_translation = base_camera_transform.getOrigin();
	    
		    

		    // DEBUGGING PRINTING STATEMENTS - PRINTING HOMOGENEOUS TRANSFORMATION MATRIX TO TERMINAL
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Base to Camera Transform: " << std::endl;
		    std::cout << base_camera_rotation[0][0] << " " <<  base_camera_rotation[0][1] << " " << base_camera_rotation[0][2] << " " << base_camera_translation[0] << std::endl;
		    std::cout << base_camera_rotation[1][0] << " " <<  base_camera_rotation[1][1] << " " << base_camera_rotation[1][2] << " " << base_camera_translation[1] << std::endl;
		    std::cout << base_camera_rotation[2][0] << " " <<  base_camera_rotation[2][1] << " " << base_camera_rotation[2][2] << " " << base_camera_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    // DEBUGGING PRINTING STATEMENTS - PRINTING QUATERNION TO TERMINAL
		    tf::Quaternion base_camera_rotation_quat = base_camera_transform.getRotation();
		    std::cout << "Base to Camera Quaternion: " << std::endl;
		    std::cout << base_camera_rotation_quat[0] << " " <<  base_camera_rotation_quat[1] << " " << base_camera_rotation_quat[2] << " " << base_camera_rotation_quat[3] << std::endl;




		    // Increment calibration position counter
		    calib_pos_count_++;


		    // Check to ensure all quaternions are facing the same direction, if not inverse the quaternion
		    if(base_camera_rotation_quat[3] < 0){
			// Calculate cumulative results for quaternion and translation between base->camera transforms
			cumulative_rot_x_ += -base_camera_rotation_quat[0];
			cumulative_rot_y_ += -base_camera_rotation_quat[1];
			cumulative_rot_z_ += -base_camera_rotation_quat[2];
			cumulative_rot_w_ += -base_camera_rotation_quat[3];

			cumulative_trans_x_ += base_camera_translation[0];
			cumulative_trans_y_ += base_camera_translation[1];
			cumulative_trans_z_ += base_camera_translation[2];
		    } else {
			// Calculate cumulative results for quaternion and translation between base->camera transforms
			cumulative_rot_x_ += base_camera_rotation_quat[0];
			cumulative_rot_y_ += base_camera_rotation_quat[1];
			cumulative_rot_z_ += base_camera_rotation_quat[2];
			cumulative_rot_w_ += base_camera_rotation_quat[3];

			cumulative_trans_x_ += base_camera_translation[0];
			cumulative_trans_y_ += base_camera_translation[1];
			cumulative_trans_z_ += base_camera_translation[2];
		    }

		    // Calculate average position and rotation by dividing the cumulative results by the number of calibration positions visited
		    tf::Vector3 average_base_camera_translation(cumulative_trans_x_/calib_pos_count_, cumulative_trans_y_/calib_pos_count_, cumulative_trans_z_/calib_pos_count_);
	 	    tf::Quaternion average_base_camera_rotation_quat(cumulative_rot_x_/calib_pos_count_, cumulative_rot_y_/calib_pos_count_, cumulative_rot_z_/calib_pos_count_, cumulative_rot_w_/calib_pos_count_);

		    // Normalise quaternion to avoid errors
		    average_base_camera_rotation_quat = average_base_camera_rotation_quat.normalize();

		    // Create average base->camera transform and store results in the transform
		    tf::Transform average_base_camera_transform;
		    average_base_camera_transform.setOrigin(average_base_camera_translation);
                    average_base_camera_transform.setRotation(average_base_camera_rotation_quat);

		    // Create unique mutex lock to update shared member variables
		    std::unique_lock<std::mutex> base_camera_transform_lock(base_camera_mutex_);
		    // Store average base->camera transform in member variable so that it can be broadcasted by parallel running thread
           	    average_base_camera_transform_ = average_base_camera_transform;
            	    base_camera_transform_lock.unlock();

		    // Set flag to activate parallel running thread
		    if(transform_set_ < 1){
			std::unique_lock<std::mutex> transform_set_lock(transform_set_mutex_);
           	    
		        transform_set_++;
            	    
		        transform_set_lock.unlock();
		    }
		
    
		    // Solely used for debugging printing statements, not needed in the functionality
		    tf::Matrix3x3 average_base_camera_rotation = average_base_camera_transform.getBasis();

		    // DEBUGGING PRINTING STATEMENTS - PRINTING HOMOGENEOUS TRANSFORMATION MATRIX TO TERMINAL
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Average Base to Camera Transform: " << std::endl;
		    std::cout << average_base_camera_rotation[0][0] << " " <<  average_base_camera_rotation[0][1] << " " << average_base_camera_rotation[0][2] << " " << average_base_camera_translation[0] << std::endl;
		    std::cout << average_base_camera_rotation[1][0] << " " <<  average_base_camera_rotation[1][1] << " " << average_base_camera_rotation[1][2] << " " << average_base_camera_translation[1] << std::endl;
		    std::cout << average_base_camera_rotation[2][0] << " " <<  average_base_camera_rotation[2][1] << " " << average_base_camera_rotation[2][2] << " " << average_base_camera_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    // DEBUGGING PRINTING STATEMENTS - PRINTING QUATERNION TO TERMINAL
		    std::cout << "Average Base to Camera Quaternion: " << std::endl;
		    std::cout << average_base_camera_rotation_quat[0] << " " <<  average_base_camera_rotation_quat[1] << " " << average_base_camera_rotation_quat[2] << " " << average_base_camera_rotation_quat[3] << std::endl;

		    // Broadcast transform to be shown in TF
		    broadcaster_->sendTransform(tf::StampedTransform(average_base_camera_transform, ros::Time::now(), "base_link", "camera_base_link"));

		   
                } else{
		    ROS_INFO("Camera cannot see the calibration marker!");

		}
	    
	    }
        
            
        } else {
	    ROS_INFO("Max calibration positions reached! Please restart Calibration node");
	    while(1){
	    }

	}
       
    
    }

}


// Main Sealer/Fridge to target arm location Calibration Function
/*
 * Function obtains the relative camera to object tag transform from the Alvar ROS package.
 * Using the kinematic loop, the function then mutiplies through the kinematic loop to calculate the relative tag bundle to target transform
 * Both the calculated tag to target transform and the rolling average tag to target transforms are printed to the terminal for the user's analysis and interpretation
 * The rolling average tag to target transform is then broadcasted to the TF tree for visualisation purposes.
 * Upon completing the calibration process, the values obtained for the average tag to target transform should be added to the tf_broadcaster.cpp file.
*/
void Calibrator::calibrateSealerFridge()
{
    while(ros::ok()) {
        // While position count < 5
        if(calib_pos_count_ < 5) {
            
            // Tell user to position KUKA Arm in required position
            std::cout << "Position KUKA Arm in required deposit position" << std::endl;
            
            
            // Prompt user to hit enter once robot is in position
            std::cout << "Press Space Bar then Enter when ready to continue Calibration... ";
	    std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
            

            // Get position of tag w.r.t. camera from Alvar package
	    // Create unique lock for safe access to member variables
            std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

            // Update member variable with most recent pose data
            marker_pose_for_calc_ = marker_pose_;

            marker_pose_lock.unlock();

            geometry_msgs::PoseStamped sensor_pose;
            float timestamp = 0.0;
	    tf::Transform camera_tag_transform;
          
            // Iterate through all the markers that can be seen by the Alvar package
            for (int i = 0; i < marker_pose_for_calc_.markers.size(); i++) {
                if (marker_pose_for_calc_.markers.at(i).id == 0 || marker_pose_for_calc_.markers.at(i).id == 4) {
                    ROS_INFO("I see the marker!");

                    // Store tag bundle pose data in temp variables
		    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
		    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

		    camera_tag_transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                    camera_tag_transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));

		    tf::StampedTransform camera_internal_transform;
		    listener_->lookupTransform("camera_base_link", sensor_pose.header.frame_id, ros::Time(0), camera_internal_transform);

                    // Get position of gripper w.r.t. link_7 from kinematic chain
		    tf::StampedTransform tag_arm_transform;
	    	    listener_->lookupTransform("link_7", "tool0", ros::Time(0), tag_arm_transform);
		
                    // Get position of robot base w.r.t. camera from kinematic chain
		    tf::StampedTransform camera_base_transform;
	    	    listener_->lookupTransform("camera_base_link", "base_link", ros::Time(0), camera_base_transform);

                    // Get position of link_1 w.r.t. robot base from kinematic chain
	    	    tf::StampedTransform arm_base1_transform;
	    	    listener_->lookupTransform("base_link", "link_1", ros::Time(0), arm_base1_transform);

                    // Get position of link_2 w.r.t. link_1 from kinematic chain
		    tf::StampedTransform arm_base2_transform;
	    	    listener_->lookupTransform("link_1", "link_2", ros::Time(0), arm_base2_transform);
		    
                    // Get position of link_3 w.r.t. link_2 from kinematic chain
		    tf::StampedTransform arm_base3_transform;
	    	    listener_->lookupTransform("link_2", "link_3", ros::Time(0), arm_base3_transform);

                    // Get position of link_4 w.r.t. link_3 from kinematic chain
		    tf::StampedTransform arm_base4_transform;
	    	    listener_->lookupTransform("link_3", "link_4", ros::Time(0), arm_base4_transform);

                    // Get position of link_5 w.r.t. link_4 from kinematic chain
		    tf::StampedTransform arm_base5_transform;
	    	    listener_->lookupTransform("link_4", "link_5", ros::Time(0), arm_base5_transform);

                    // Get position of link_6 w.r.t. link_5 from kinematic chain
		    tf::StampedTransform arm_base6_transform;
	    	    listener_->lookupTransform("link_5", "link_6", ros::Time(0), arm_base6_transform);

                    // Get position of link_7 w.r.t. link_6 from kinematic chain
		    tf::StampedTransform arm_base7_transform;
	    	    listener_->lookupTransform("link_6", "link_7", ros::Time(0), arm_base7_transform);


	            // Calculate relative transform between tag and target by solving the equation X = (A*B*C)^-1*I
		    tf::Transform calc_transform;
		    tf::Transform temp_transform;

	 	    temp_transform.mult(camera_internal_transform, camera_tag_transform);
	            temp_transform = temp_transform.inverse();
                    calc_transform = temp_transform;

	  	    calc_transform *= camera_base_transform;
		    calc_transform *= arm_base1_transform;
		    calc_transform *= arm_base2_transform;
		    calc_transform *= arm_base3_transform;
		    calc_transform *= arm_base4_transform;
		    calc_transform *= arm_base5_transform;
		    calc_transform *= arm_base6_transform;
		    calc_transform *= arm_base7_transform;
		    calc_transform *= tag_arm_transform;


		    tf::Transform tag_target_transform;

		    tag_target_transform = calc_transform;

		    tf::Matrix3x3 tag_target_rotation = tag_target_transform.getBasis();
		    tf::Vector3 tag_target_translation = tag_target_transform.getOrigin();


		    // DEBUGGING PRINTING STATEMENTS - print calculated Homogeneous transform to terminal
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "AR Tag to Target Transform: " << std::endl;
		    std::cout << tag_target_rotation[0][0] << " " <<  tag_target_rotation[0][1] << " " << tag_target_rotation[0][2] << " " << tag_target_translation[0] << std::endl;
		    std::cout << tag_target_rotation[1][0] << " " <<  tag_target_rotation[1][1] << " " << tag_target_rotation[1][2] << " " << tag_target_translation[1] << std::endl;
		    std::cout << tag_target_rotation[2][0] << " " <<  tag_target_rotation[2][1] << " " << tag_target_rotation[2][2] << " " << tag_target_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

                    // DEBUGGING PRINTING STATEMENTS - print calculated tag to target Quaternion to terminal
		    tf::Quaternion tag_target_rotation_quat = tag_target_transform.getRotation();
		    std::cout << "AR Tag to Target Quaternion: " << std::endl;
		    std::cout << tag_target_rotation_quat[0] << " " <<  tag_target_rotation_quat[1] << " " << tag_target_rotation_quat[2] << " " << tag_target_rotation_quat[3] << std::endl;

		    // Increment calibration position counter
		    calib_pos_count_++;


		    // Check to ensure all quaternions are facing the same direction, if not inverse the quaternion
		    if(tag_target_rotation_quat[3] < 0){
			// Calculate cumulative results for quaternion and translation between base->camera transforms
			cumulative_rot_x_ += -tag_target_rotation_quat[0];
			cumulative_rot_y_ += -tag_target_rotation_quat[1];
			cumulative_rot_z_ += -tag_target_rotation_quat[2];
			cumulative_rot_w_ += -tag_target_rotation_quat[3];

			cumulative_trans_x_ += tag_target_translation[0];
			cumulative_trans_y_ += tag_target_translation[1];
			cumulative_trans_z_ += tag_target_translation[2];
		    } else {
			// Calculate cumulative results for quaternion and translation between base->camera transforms
			cumulative_rot_x_ += tag_target_rotation_quat[0];
			cumulative_rot_y_ += tag_target_rotation_quat[1];
			cumulative_rot_z_ += tag_target_rotation_quat[2];
			cumulative_rot_w_ += tag_target_rotation_quat[3];

			cumulative_trans_x_ += tag_target_translation[0];
			cumulative_trans_y_ += tag_target_translation[1];
			cumulative_trans_z_ += tag_target_translation[2];
		    }

		    // Calculate average position and rotation by dividing the cumulative results by the number of calibration positions visited
		    tf::Vector3 average_tag_target_translation(cumulative_trans_x_/calib_pos_count_, cumulative_trans_y_/calib_pos_count_, cumulative_trans_z_/calib_pos_count_);
	 	    tf::Quaternion average_tag_target_rotation_quat(cumulative_rot_x_/calib_pos_count_, cumulative_rot_y_/calib_pos_count_, cumulative_rot_z_/calib_pos_count_, cumulative_rot_w_/calib_pos_count_);

		    // Normalise quaternion to avoid errors
		    average_tag_target_rotation_quat = average_tag_target_rotation_quat.normalize();

		    // Create average base->camera transform and store results in the transform
		    tf::Transform average_tag_target_transform;
		    average_tag_target_transform.setOrigin(average_tag_target_translation);
                    average_tag_target_transform.setRotation(average_tag_target_rotation_quat);

		    // Create unique mutex lock to update shared member variables
		    std::unique_lock<std::mutex> camera_tag_transform_lock(camera_tag_transform_mutex_);
		    camera_tag_transform_ = camera_tag_transform;
		    camera_tag_transform_lock.unlock();

		    // Create unique mutex lock to update shared member variables
		    std::unique_lock<std::mutex> tag_target_transform_lock(tag_target_mutex_);
		    // Store average tag->target transform in member variable so that it can be broadcasted by parallel running thread
           	    average_tag_target_transform_ = average_tag_target_transform;
            	    tag_target_transform_lock.unlock();	
    
		    // Solely used for debugging printing statements, not needed in the functionality
		    tf::Matrix3x3 average_tag_target_rotation = average_tag_target_transform.getBasis();

		    // DEBUGGING PRINTING STATEMENTS - PRINTING HOMOGENEOUS TRANSFORMATION MATRIX TO TERMINAL
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Average Tag to Target Transform: " << std::endl;
		    std::cout << average_tag_target_rotation[0][0] << " " <<  average_tag_target_rotation[0][1] << " " << average_tag_target_rotation[0][2] << " " << average_tag_target_translation[0] << std::endl;
		    std::cout << average_tag_target_rotation[1][0] << " " <<  average_tag_target_rotation[1][1] << " " << average_tag_target_rotation[1][2] << " " << average_tag_target_translation[1] << std::endl;
		    std::cout << average_tag_target_rotation[2][0] << " " <<  average_tag_target_rotation[2][1] << " " << average_tag_target_rotation[2][2] << " " << average_tag_target_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    // DEBUGGING PRINTING STATEMENTS - PRINTING QUATERNION TO TERMINAL
		    std::cout << "Average Base to Camera Quaternion: " << std::endl;
		    std::cout << average_tag_target_rotation_quat[0] << " " <<  average_tag_target_rotation_quat[1] << " " << average_tag_target_rotation_quat[2] << " " << average_tag_target_rotation_quat[3] << std::endl;


		    if (marker_pose_for_calc_.markers.at(i).id == 0) {
			// Sealer Unit
			broadcaster_->sendTransform(tf::StampedTransform(camera_tag_transform, ros::Time::now(), sensor_pose.header.frame_id, "sealer_tag"));
		        broadcaster_->sendTransform(tf::StampedTransform(tag_target_transform, ros::Time::now(), "sealer_tag", "sealer_link"));

			// Set flag to activate parallel running thread
		    	if(sealer_transform_set_ < 1){
			    std::unique_lock<std::mutex> transform_set_lock(sealer_transform_set_mutex_);
           	    
		            sealer_transform_set_++;
            	    
		            transform_set_lock.unlock();
		    	}

		    } else {
			// Fridge
			broadcaster_->sendTransform(tf::StampedTransform(camera_tag_transform, ros::Time::now(), sensor_pose.header.frame_id, "fridge_tag"));
		        broadcaster_->sendTransform(tf::StampedTransform(tag_target_transform, ros::Time::now(), "fridge_tag", "fridge_link"));
			
			// Set flag to activate parallel running thread
		    	if(fridge_transform_set_ < 1){
			    std::unique_lock<std::mutex> transform_set_lock(fridge_transform_set_mutex_);
           	    
		            fridge_transform_set_++;
            	    
		            transform_set_lock.unlock();
		    	}

		    }

                } else{
		    ROS_INFO("Camera cannot see the AR marker!");

		}
	    
	    }
        
            
        } else {
	      


	}
       
    
    }



}
