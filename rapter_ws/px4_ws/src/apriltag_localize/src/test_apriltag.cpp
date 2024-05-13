#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

void tf_message_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Received TFMessage:");
    for (const auto& transform : msg->transforms) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "  Header:");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Stamp: sec=" << transform.header.stamp.sec << " nanosec=" << transform.header.stamp.nanosec);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Frame ID: " << transform.header.frame_id);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "  Child Frame ID: " << transform.child_frame_id);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "  Transform:");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Translation: x=" << transform.transform.translation.x << " y=" << transform.transform.translation.y << " z=" << transform.transform.translation.z);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("subscriber"), "    Rotation: x=" << transform.transform.rotation.x << " y=" << transform.transform.rotation.y << " z=" << transform.transform.rotation.z << " w=" << transform.transform.rotation.w);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_message_subscriber");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::QoS qos(rclcpp::QoSInitialization(qos_profile.history, 5));
    rclcpp::QoS qos_assured(rclcpp::KeepLast(5));
    qos_assured.best_effort();
    qos_assured.durability_volatile();

    auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        qos,
        tf_message_callback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
