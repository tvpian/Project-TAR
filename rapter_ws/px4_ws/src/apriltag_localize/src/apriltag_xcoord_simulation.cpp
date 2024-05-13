#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <Eigen/Dense>

using namespace std::placeholders;


rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr local_pos_pub;
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

// rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr at_sub;

rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_info_sub;
rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_body_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_lpp_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_body_stab_pub;

sensor_msgs::msg::Joy joy_in;
geometry_msgs::msg::Transform at_in_data;
px4_msgs::msg::VehicleLocalPosition lpp_data;
px4_msgs::msg::VehicleAttitude att_data;

bool lpp_data_in = 0;
bool at_in = 0;
bool att_in = 0;
bool all_in = 0;

Eigen::Matrix3d Rmat(const Eigen::Quaterniond& q)
{
    double x, y, z, w;
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();

    Eigen::Matrix3d R;
    R(0,0) = 1 - 2*y*y - 2*z*z;
    R(0,1) = 2*x*y + 2*w*z;
    R(0,2) = 2*x*z - 2*w*y;
    R(1,0) = 2*x*y - 2*w*z;
    R(1,1) = 1 - 2*x*x - 2*z*z;
    R(1,2) = 2*y*z + 2*w*x;
    R(2,0) = 2*x*z + 2*w*y;
    R(2,1) = 2*y*z - 2*w*x;
    R(2,2) = 1 - 2*x*x - 2*y*y;

    return R;
}

void joy_cb(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
    // For future use
    joy_in = *joy_msg;
}

void tf_message_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    // RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Received TFMessage:");
    // for (const auto& transform : msg->transforms) {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "  Header:");
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Stamp: sec=" << transform.header.stamp.sec << " nanosec=" << transform.header.stamp.nanosec);
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Frame ID: " << transform.header.frame_id);
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "  Child Frame ID: " << transform.child_frame_id);
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "  Transform:");
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Translation: x=" << transform.transform.translation.x << " y=" << transform.transform.translation.y << " z=" << transform.transform.translation.z);
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Rotation: x=" << transform.transform.rotation.x << " y=" << transform.transform.rotation.y << " z=" << transform.transform.rotation.z << " w=" << transform.transform.rotation.w);
    // }

    // Check if the message contains any transforms and if there is a transform store it in the at_in_data variable
    if (msg->transforms.size() > 0) {
        at_in_data = msg->transforms[0].transform;
        at_in = 1;
    }
}

void lpp_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr lpp_msg)
{
    lpp_data = *lpp_msg;
    lpp_data_in = 1;
}

void att_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr att_msg)
{
    att_data = *att_msg;
    att_in = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("apriltag_localize_node");
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::QoS qos(rclcpp::QoSInitialization(qos_profile.history, 5));
    rclcpp::QoS qos_assured(rclcpp::KeepLast(5));
    qos_assured.best_effort();
    qos_assured.durability_volatile();

    local_pos_pub = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 5);
    joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&joy_cb, _1));
    // at_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose", qos_assured, std::bind(&at_cb, _1));
    // at_sub = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", qos_assured, std::bind(&at_cb, _1));
    
    auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        qos,
        tf_message_callback);
    
    
    local_info_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_assured, std::bind(&lpp_callback, _1));
    attitude_sub = node->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos_assured, std::bind(&att_callback, _1));
    target_body_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose_body", 5);
    target_lpp_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose_inertial", 5);
    target_body_stab_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose_body_stabilized", 5);

    Eigen::Matrix3d R;

    rclcpp::Rate rate( 20);
    joy_in.axes.resize(8);
    joy_in.buttons.resize(8);
    
    
    //int count = 0;

    while (rclcpp::ok()) {
        
        if(at_in && lpp_data_in){
        all_in=1;
        }

        // The position of the drone relative to the local inertial frame (vehicle starts along y axis using indoor parameters)
        // Drone axes are x = forward, y = left, z = up (FLU)
        // Vehicle should start with initial orientation of 90 deg right; quaternion = (0,0,-0.707, -0.707)
        double xp = lpp_data.x;
        double yp = lpp_data.y;
        double zp = lpp_data.z;

        // // // Position of the apriltag in the camera coordinate frame.  Z coincident with optical axis; Y down in camera frame; X to the right when looking from vehicle out
        double xt = at_in_data.translation.x;
        double yt = at_in_data.translation.y;
        double zt = at_in_data.translation.z;
        RCLCPP_INFO(node->get_logger(), "xt: %f, yt: %f, zt: %f", xt, yt, zt);
        // // EIGEN's convention is to have the SCALAR value FIRST!!!
        Eigen::Quaterniond quat_lpp(att_data.q[0], att_data.q[1], att_data.q[2], att_data.q[3]);
        Eigen::Matrix3d R_lpp = quat_lpp.toRotationMatrix();

        // // Populating a homogenous transformation matrix with /mavros/local_position/pose data
        // // converting quaternion to rotation matrix
        Eigen::Matrix4d H_lpp;
        H_lpp.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
        H_lpp.block(0,0,3,3) = R_lpp;
        H_lpp(3,3) = 1.0;
        H_lpp(0,3) = xp;
        H_lpp(1,3) = yp;
        H_lpp(2,3) = zp;

        // // A fixed homogenous transformation (zero translation) to convert between camera frame and local body FLU frame
        // // Camera is pointing 45 deg down looking forward. = tracking camera
        // // This matrix takes points in the body frame and converts to camera frame
        Eigen::Matrix4d H_M_B;
        //H_M_B << 0,-1,0,0,-0.707,0,-0.707, 0, 0.707, 0, -0.707, 0, 0, 0, 0, 1;
        H_M_B << 0, 1, 0, 0, -0.707, 0, 0.707, 0, 0.707, 0, 0.707, 0, 0, 0, 0, 1;
         
        // // create a vector with apriltag position relative to camera
        Eigen::Vector4d r4vec(4);
        r4vec << xt,yt,zt,1;

        // // Rotate the apriltag position from camera coordinates to FLU coordinates
        // // inverse H_M_B converts from camera to body coordinates.
        // // Note: For unitary transformation (unitary rotation matrix) transpose of matrix = inverse of matrix
        Eigen::Vector4d P_r_B(4);
        P_r_B = H_M_B.inverse()*r4vec;

        Eigen::Vector4d P_r_I(4);
        P_r_I = H_lpp*P_r_B;

        // // This computes P_r_I2 which is the computed location of the Apriltag in the local inertial mavros lpp frame
        Eigen::Vector4d P_r_I2(4);
        Eigen::Matrix4d H_lpp_nopos;
        H_lpp_nopos = H_lpp;
        H_lpp_nopos(0,3) = 0;
        H_lpp_nopos(1,3) = 0;
        H_lpp_nopos(2,3) = 0;
        P_r_I2 = H_lpp_nopos*H_M_B.inverse()*r4vec;

        // // This computes the difference between the inertial location of the apriltag and inertial location of the vehicle
        Eigen::Vector4d I_diff;
        I_diff = P_r_I2-P_r_I;

        // // This computes the Euler angles associated with the mavros/local_position/pose quaternion in yaw->pitch->roll format.
        Eigen::Vector3d euler_ang = quat_lpp.toRotationMatrix().eulerAngles(2,1,0);

        // // This creates a rotation matrix considering ONLY the heading (not roll/pitch) so we can rotate between body and inertial heading
        // // I'm mentally thinking of this as body referenced, but "stabilized" by removing roll and pitch 
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << cos(euler_ang(0)), -sin(euler_ang(0)), 0, sin(euler_ang(0)),  cos(euler_ang(0)), 0, 0, 0, 1;

        // // create the location of the tag not in inertial, but in local body referenced heading coordinates
        Eigen::Vector3d body_diff;
        body_diff = rotation_matrix.transpose()*I_diff.block<3,1>(0,0);

        // // populate and publish the apriltag in body FLU coordinates
        geometry_msgs::msg::PoseStamped body_pub_data;

        // // Convert timestamp from nanoseconds to seconds
        double timestamp_sec = static_cast<double>(lpp_data.timestamp) / 1e9;

        // //body_pub_data.header.stamp = lpp_data.timestamp;
        body_pub_data.pose.position.x = P_r_B(0);
        body_pub_data.pose.position.y = P_r_B(1);
        body_pub_data.pose.position.z = P_r_B(2);
        target_body_pub->publish(body_pub_data);

        // populate and publish the apriltag in inertial coordinates
        geometry_msgs::msg::PoseStamped lpp_pub_data;
        //lpp_pub_data.header.stamp = lpp_data.timestamp;
        lpp_pub_data.header.stamp.sec = static_cast<int32_t>(timestamp_sec);
        lpp_pub_data.header.stamp.nanosec = static_cast<int32_t>((timestamp_sec - lpp_pub_data.header.stamp.sec) * 1e9);

        lpp_pub_data.pose.position.x = P_r_I(0);
        lpp_pub_data.pose.position.y = P_r_I(1);
        lpp_pub_data.pose.position.z = P_r_I(2);
        target_lpp_pub->publish(lpp_pub_data);

        // populate and publish the apriltag in stabilized body coordindates
        geometry_msgs::msg::PoseStamped body_stab_pub_data;
        //body_stab_pub_data.header.stamp = lpp_data.header.stamp;
        body_stab_pub_data.pose.position.x = body_diff(0);
        body_stab_pub_data.pose.position.y = body_diff(1);
        body_stab_pub_data.pose.position.z = body_diff(2);
        target_body_stab_pub->publish(body_stab_pub_data);

        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

