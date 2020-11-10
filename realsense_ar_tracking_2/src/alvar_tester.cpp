#include "alvar_tester.h"

// Constructor
AlvarTester::AlvarTester(ros::NodeHandle nh)
    : nh_(nh), listener_(new(tf::TransformListener)), broadcaster_(new(tf::TransformBroadcaster)), calib_pos_count_(0), cumulative_rot_x_(0.0), cumulative_rot_y_ (0.0), cumulative_rot_z_(0.0), cumulative_rot_w_(0.0), cumulative_trans_x_(0.0), cumulative_trans_y_ (0.0), cumulative_trans_z_(0.0)
{
    sub1_ = nh_.subscribe("ar_pose_marker", 1000, &AlvarTester::trackerCallback, this);


}

// Deconstructor
AlvarTester::~AlvarTester()
{

}

// AR Tag Tracking Node Callback
void AlvarTester::trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{

    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

    // Update member variable with most recent tag pose data
    marker_pose_ = *msg;

    marker_pose_lock.unlock();
    
}
    


 
// Main Test Alvar Function
/*
 * Used to obtain the Alvar package result and display the relative camera to tag transform data to the terminal.
 * This information is then collected and analysed by the user to comment on precision and accuracy of the Alvar package.
 * This function both prints the current transform from the camera to the tag returned by the Alvar package as well as calculates the rolling average of the transforms returned by the package.
*/
void AlvarTester::testAlvar()
{
    while(ros::ok()) {
        // While position count < 10
        if(calib_pos_count_ < 10) {
                         
	    // Display output to user
            std::cout << "Trial: " << (calib_pos_count_ + 1) << std::endl;     

            
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

            // Temporary variables
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

                   
                    // PRINTING STATEMENTS - PRINTING HOMOGENEOUS TRANSFORMATION MATRIX TO TERMINAL
		    tf::Matrix3x3 camera_tag_rotation = camera_tag_transform.getBasis();
		    tf::Vector3 camera_tag_translation = camera_tag_transform.getOrigin();
		    
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Camera to Tag Transform: " << std::endl;
		    std::cout << camera_tag_rotation[0][0] << " " <<  camera_tag_rotation[0][1] << " " << camera_tag_rotation[0][2] << " " << camera_tag_translation[0] << std::endl;
		    std::cout << camera_tag_rotation[1][0] << " " <<  camera_tag_rotation[1][1] << " " << camera_tag_rotation[1][2] << " " << camera_tag_translation[1] << std::endl;
		    std::cout << camera_tag_rotation[2][0] << " " <<  camera_tag_rotation[2][1] << " " << camera_tag_rotation[2][2] << " " << camera_tag_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    // PRINTING STATEMENTS - PRINTING QUATERNION TO TERMINAL
		    tf::Quaternion camera_tag_quat = camera_tag_transform.getRotation();
		    std::cout << "Camera to Tag Quaternion: " << std::endl;
		    std::cout << camera_tag_quat[0] << " " <<  camera_tag_quat[1] << " " << camera_tag_quat[2] << " " << camera_tag_quat[3] << std::endl;

		    // Increment calibration position counter
		    calib_pos_count_++;


		    // Check to ensure all quaternions are facing the same direction, if not inverse the quaternion
		    if(camera_tag_quat[3] < 0){
			// Calculate cumulative results for quaternion and translation between base->camera transforms
			cumulative_rot_x_ += -camera_tag_quat[0];
			cumulative_rot_y_ += -camera_tag_quat[1];
			cumulative_rot_z_ += -camera_tag_quat[2];
			cumulative_rot_w_ += -camera_tag_quat[3];

			cumulative_trans_x_ += camera_tag_translation[0];
			cumulative_trans_y_ += camera_tag_translation[1];
			cumulative_trans_z_ += camera_tag_translation[2];
		    } else {
			// Calculate cumulative results for quaternion and translation between base->camera transforms
			cumulative_rot_x_ += camera_tag_quat[0];
			cumulative_rot_y_ += camera_tag_quat[1];
			cumulative_rot_z_ += camera_tag_quat[2];
			cumulative_rot_w_ += camera_tag_quat[3];

			cumulative_trans_x_ += camera_tag_translation[0];
			cumulative_trans_y_ += camera_tag_translation[1];
			cumulative_trans_z_ += camera_tag_translation[2];
		    }

		    // Calculate average position and rotation by dividing the cumulative results by the number of calibration positions visited
		    tf::Vector3 average_camera_tag_translation(cumulative_trans_x_/calib_pos_count_, cumulative_trans_y_/calib_pos_count_, cumulative_trans_z_/calib_pos_count_);
	 	    tf::Quaternion average_camera_tag_quat(cumulative_rot_x_/calib_pos_count_, cumulative_rot_y_/calib_pos_count_, cumulative_rot_z_/calib_pos_count_, cumulative_rot_w_/calib_pos_count_);

		    // Normalise quaternion to avoid errors
		    average_camera_tag_quat = average_camera_tag_quat.normalize();

		    // Create average base->camera transform and store results in the transform
		    tf::Transform average_camera_tag_transform;
		    average_camera_tag_transform.setOrigin(average_camera_tag_translation);
                    average_camera_tag_transform.setRotation(average_camera_tag_quat);
		
    
		    // Solely used for debugging printing statements, not needed in the functionality
		    tf::Matrix3x3 average_camera_tag_rotation = average_camera_tag_transform.getBasis();

		    // PRINTING STATEMENTS - PRINTING HOMOGENEOUS TRANSFORMATION MATRIX TO TERMINAL
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Average Camera to Tag Transform: " << std::endl;
		    std::cout << average_camera_tag_rotation[0][0] << " " <<  average_camera_tag_rotation[0][1] << " " << average_camera_tag_rotation[0][2] << " " << average_camera_tag_translation[0] << std::endl;
		    std::cout << average_camera_tag_rotation[1][0] << " " <<  average_camera_tag_rotation[1][1] << " " << average_camera_tag_rotation[1][2] << " " << average_camera_tag_translation[1] << std::endl;
		    std::cout << average_camera_tag_rotation[2][0] << " " <<  average_camera_tag_rotation[2][1] << " " << average_camera_tag_rotation[2][2] << " " << average_camera_tag_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    // PRINTING STATEMENTS - PRINTING QUATERNION TO TERMINAL
		    std::cout << "Average Camera to Tag Quaternion: " << std::endl;
		    std::cout << average_camera_tag_quat[0] << " " <<  average_camera_tag_quat[1] << " " << average_camera_tag_quat[2] << " " << average_camera_tag_quat[3] << std::endl;

		   
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





