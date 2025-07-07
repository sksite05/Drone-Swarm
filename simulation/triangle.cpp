#include <gnc_functions.hpp>
// Include API

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    // Initialize control publisher/subscribers
    init_publisher_subscriber(gnc_node);

    // Wait for FCU connection
    ROS_INFO("Waiting for FCU connection...");
    wait4connect();

    ROS_INFO("Setting mode to GUIDED...");
    set_mode("GUIDED");

    // Create local reference frame
    ROS_INFO("Initializing local reference frame...");
    initialize_local_frame();

    // Request takeoff
    ROS_INFO("Taking off to altitude 3m...");
    takeoff(3);

    // Specify triangular waypoints
    std::vector<gnc_api_waypoint> waypointList;
    gnc_api_waypoint nextWayPoint;

    // Waypoints for a triangular path
    nextWayPoint.x = 0; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.x = 5; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = -90;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.x = 2.5; nextWayPoint.y = 4.33; nextWayPoint.z = 3; nextWayPoint.psi = 120;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.x = 0; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);

    // Control loop rate
    ros::Rate rate(2.0);
    int counter = 0;

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if (check_waypoint_reached(0.3) == 1) {
            if (counter < waypointList.size()) {
                ROS_INFO("Setting destination: x=%f, y=%f, z=%f, psi=%f",
                         waypointList[counter].x, waypointList[counter].y,
                         waypointList[counter].z, waypointList[counter].psi);
                set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                counter++;
            } else {
                ROS_INFO("All waypoints reached. Landing...");
                land();
                break;
            }
        }
    }
    return 0;
}
