
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void broadcastTransform() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0)); // Modifica la posizione secondo le tue esigenze
    tf::Quaternion q;
    q.setRPY(0, 0, 0); // Modifica l'orientamento secondo le tue esigenze
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "rslidar"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_to_lidar");
    ros::NodeHandle node;
    ros::Rate rate(10.0);
    while (node.ok()) {
        broadcastTransform();
        rate.sleep();
    }
    return 0;
}
