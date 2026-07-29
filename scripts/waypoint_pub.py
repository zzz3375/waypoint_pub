import rospy
import numpy as np 
import os
import sys
import yaml
from sensor_msgs.msg import LaserScan
# Get the directory containing this script
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from env.uav_ros import UAV_ROS
from obstacle_avoider import ObstacleAvoider


# Set numpy print options for 2 decimal places
np.set_printoptions(precision=2, floatmode='fixed')

def split_waypoints(waypoints, max_distance=0.9):
    """Interpolate between xyzYaw waypoints to ensure max distance between points
    
    Args:
        waypoints: List of numpy arrays [x,y,z,yaw]
        max_distance: Maximum allowed distance between consecutive points (meters)
    
    Returns:
        List of interpolated waypoints with same format
    """
    if not waypoints:
        return []
    
    interpolated = [waypoints[0]]
    
    for i in range(1, len(waypoints)):
        p0 = waypoints[i-1][:3]  # Get xyz
        p1 = waypoints[i][:3]
        yaw0 = waypoints[i-1][3]
        yaw1 = waypoints[i][3]
        
        # Calculate distance between current waypoints
        dist = np.linalg.norm(p1 - p0)
        
        if dist <= max_distance:
            interpolated.append(waypoints[i])
        else:
            # Calculate number of intermediate points needed
            num_points = int(np.ceil(dist / max_distance))
            
            # Linear interpolation for position and yaw
            for j in range(1, num_points + 1):
                alpha = j / (num_points + 1)
                pos = p0 + alpha * (p1 - p0)
                
                # Handle yaw interpolation (shortest path)
                yaw_diff = (yaw1 - yaw0 + np.pi) % (2*np.pi) - np.pi
                yaw = yaw0 + alpha * yaw_diff
                new_point = np.array([pos[0], pos[1], pos[2], yaw])
                
                interpolated.append(new_point)
            
            interpolated.append(waypoints[i])
    
    # Print the interpolated waypoints
    # print(f"\nInterpolated waypoints (max distance: {max_distance}m):")
    # for i, wp in enumerate(interpolated):
    #     print(f"Waypoint {i}: {wp}")
    
    return interpolated

def load_waypoints(config_file="scripts/waypoints_config/allen_fly.yaml"):
    """Load waypoints from YAML configuration file
    Expected YAML format:
    waypoints:
      - [x, y, z, yaw_deg]  # yaw in degrees
      - [x, y, z, yaw_deg]
    """
    # Get path relative to this script
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(package_path, config_file)
    print(f"Looking for waypoints at: {config_path}")
    
    try:
        with open(config_path) as f:
            config = yaml.safe_load(f)
        
        # Convert to numpy arrays and process yaw
        waypoints = []
        for wp in config['waypoints']:
            if len(wp) == 4:  # If waypoint includes yaw
                x, y, z, yaw_deg = wp
                normalized_yaw_deg = yaw_deg%360
                yaw_rad = np.radians(normalized_yaw_deg)  # Convert to radians
                waypoints.append(np.array([x, y, z, yaw_rad]))
            else:  # If no yaw specified
                waypoints.append(np.array(wp))
        
        # Print the loaded waypoints
        print("Successfully loaded waypoints (x,y,z,yaw_rad):")
        for i, wp in enumerate(waypoints):
            print(f"Waypoint before split: {i}: {wp}")
        
        splited_waypoints = split_waypoints(waypoints)

        for i, wp in enumerate(splited_waypoints):
            print(f"Waypoint after split: {i}: {wp}")
        return splited_waypoints
        
    except FileNotFoundError:
        rospy.logerr(f"\n\nWaypoints file not found at: {config_path}")

if __name__ == "__main__":
    # rospy.init_node("waypoint_pub")
    dt = 0.01
    print("we are in main")
    reached_final_waypoint = False
    # Load waypoints from config
    waypoints = load_waypoints()
    print("after load_waypoints")

    control_name = "PdT_" + "fix"

    uav = UAV_ROS(m=0.72, dt=dt, use_gazebo=False, control_name=control_name)

    # ---- obstacle avoidance module ----
    avoider = ObstacleAvoider(safe_distance=5.0, sector_half_deg=30,
                              cloud_topic="/obstacle_cloud")
    scan_topic = "/" + uav.group.rstrip("/") + "/mavros/obstacle/send"
    rospy.Subscriber(scan_topic, LaserScan, avoider.feed_scan)
    rospy.loginfo(f"Obstacle avoidance ON  |  safe=5.0m  sector=±30°  |  {scan_topic}")

    approch_time = 5
    simulation_time = 600 #10 minute duration

    start_time = rospy.Time.now()
    current_time = start_time
    uav.t0 = start_time.to_sec()
    sim_t = (current_time - start_time).to_sec()
    current_waypoint_index = 0
    
    rospy.loginfo('system init')
    uav.initialize_system()

    rospy.loginfo(f"Control loop started at {1/uav.dt:.1f}Hz")
    rospy.loginfo("Starting position guidance...")

    rospy.loginfo('Wait 1 sec')
    for _ in range(100):
        uav.pub_position_keep()
        uav.rate.sleep()

    uav.is_show_rviz = True
    if not uav.use_gazebo:
        uav.is_show_rviz = False

    print("Successfully loaded waypoints:\n x y z yaw")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i}: {wp}")


    start_curr_point_time = rospy.Time.now().to_sec()
    while (not rospy.is_shutdown()) and (not reached_final_waypoint):  
        curr_point_time = rospy.Time.now().to_sec()

        pos = uav.uav_states[0:3]            # world [x, y, z]
        yaw = uav.uav_states[8]              # world yaw (rad)

        current_target = waypoints[current_waypoint_index]
        tgt = current_target[:3]             # target [x, y, z]
        threshold_distance = 0.315
        dist = np.linalg.norm(pos - tgt)

        # ---- obstacle avoidance (before publishing target) ----
        if dist > threshold_distance and avoider.should_hold(pos, tgt, yaw):
            uav.pub_position_keep()
            rospy.logwarn_throttle(2, "OBSTACLE in movement path, holding position...")
            uav.rate.sleep()
            continue

        # ---- obstacle cloud for RViz (throttled internally) ----
        avoider.publish_obstacle_cloud(pos, yaw)

        # ---- normal waypoint sequencing ----
        if dist > threshold_distance and current_waypoint_index < len(waypoints) - 1:
            print(f"approaching waypoint {current_waypoint_index}, curr dist is {dist:.2f}")
        elif dist < threshold_distance and current_waypoint_index < len(waypoints) - 1 and curr_point_time-start_curr_point_time<2.:
            print(f"reached waypoint {current_waypoint_index}, need to wait for 5 sec")
        elif dist < threshold_distance and current_waypoint_index < len(waypoints) - 1 and curr_point_time-start_curr_point_time>=2.:
            current_waypoint_index += 1
            start_curr_point_time = rospy.Time.now().to_sec()
            current_target = waypoints[current_waypoint_index]
            print(f"Switching to waypoint {current_waypoint_index}, the points is {waypoints[current_waypoint_index]}")
        elif dist < threshold_distance and current_waypoint_index == len(waypoints) - 1:
            reached_final_waypoint = True
            print("Reached final waypoint! Prepare to land")
        
        uav.target_xyzYaw=(current_target[0], current_target[1], current_target[2], current_target[3])
        uav.pub_target_position()
        uav.rate.sleep()

    rospy.loginfo('Simulation is finished')
    uav.is_show_rviz = False
    uav.target_xyzYaw = (.0, .0, .25, 0)
    uav.reach_target_position()


    


        

