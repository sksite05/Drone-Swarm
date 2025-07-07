#include <gnc_functions.hpp>

int main(int argc, char** argv)
{
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

    std::vector<gnc_api_waypoint> waypointList;
    gnc_api_waypoint wp;

    // Position depends on the namespace (drone1, drone2, drone3)
    std::string drone_name;
    gnc_node.getParam("namespace", drone_name);

    if (drone_name == "/drone1") {
        wp.x = 0; wp.y = 0; wp.z = 3; wp.psi = 0;
        waypointList.push_back(wp);
    } else if (drone_name == "/drone2") {
        wp.x = 2.5; wp.y = 0; wp.z = 3; wp.psi = 0;
        waypointList.push_back(wp);
    } else if (drone_name == "/drone3") {
        wp.x = 5; wp.y = 0; wp.z = 3; wp.psi = 0;
        waypointList.push_back(wp);
    }

    ros::Rate rate(2.0);
    int i = 0;
    bool sent = false;

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        if (check_waypoint_reached(0.3) && i < waypointList.size())
        {
            set_destination(waypointList[i].x, waypointList[i].y, waypointList[i].z, waypointList[i].psi);
            i++;
        }
        else if (i >= waypointList.size() && !sent)
        {
            land();
            sent = true;
        }
    }

    return 0;
}
