#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher odom_pub;

// Funzione di callback per gestire l'odometria e pubblicare la trasformazione TF
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, tf2_ros::TransformBroadcaster &tf_broadcaster, const std::string& root_frame, const std::string& child_frame) {
    geometry_msgs::TransformStamped transform;

    // Configura l'intestazione TF
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = root_frame; // Frame di partenza
    transform.child_frame_id = child_frame; // Frame di arrivo

    // Copia la posizione dall'odometria
    transform.transform.translation.x = odom_msg->pose.pose.position.x;
    transform.transform.translation.y = odom_msg->pose.pose.position.y;
    transform.transform.translation.z = odom_msg->pose.pose.position.z;

    // Copia l'orientamento dall'odometria
    transform.transform.rotation = odom_msg->pose.pose.orientation;

    // Pubblica la trasformazione TF
    tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Ottieni i parametri root_frame e child_frame
    std::string root_frame = "odom";
    std::string child_frame = "base_link";

    private_nh.getParam("root_frame", root_frame);
    private_nh.getParam("child_frame", child_frame);

    // Crea un TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Crea un subscriber per ricevere messaggi di odometria
    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
        "ugv/odom", 10,
        [&tf_broadcaster, root_frame, child_frame](const nav_msgs::Odometry::ConstPtr& odom_msg) {
            odomCallback(odom_msg, tf_broadcaster, root_frame, child_frame);
        }
    );

    ros::spin(); // Mantieni il nodo attivo
    return 0;
}
