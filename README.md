# ROS1 to ROS2 Syntax Conversion

- targets: Convert the Syntax from ROS1 to ROS2.
- tips: Compare the source code below ros1_example/ros2_example folders.

Source language
---

- C++
- Python (Updating)

ROS Conversion (C++)
---

### 1. Library Including

- ROS 1 -> ROS 2

    ```cpp=
    #include "ros/ros.h"                -> #include "rclcpp/rclcpp.hpp"
    ```
    
    ```cpp=
    // Notice the Capitalization

    #include "std_msgs/Float64.h"           -> #include "std_msgs/msg/float64.hpp"
    #include "msgs/DetectedObjectArray.h"   -> #include "msgs/msg/detected_object_array.hpp"
    #include "msgs/LidLLA.h"                -> #include "msgs/msg/lid_lla.hpp"
    #include "msgs/VehInfo.h"               -> #include "msgs/msg/veh_info.hpp"
    #include "std_msgs/String.h"            -> #include "msgs/msg/flag_info.hpp"
    #include "msgs/StopInfoArray.h"         -> #include "msgs/msg/stop_info_array.hpp"
    #include "msgs/StopInfo.h"              -> #include "msgs/msg/stop_info.hpp"
    #include "msgs/RouteInfo.h"             -> #include "msgs/msg/route_info.hpp"
    #include "msgs/BackendInfo.h"           -> #include "msgs/msg/backend_info.hpp"
    #include "msgs/Spat.h"                  -> #include "msgs/msg/spat.hpp"
    #include "msgs/Obu.h"                   -> #include "msgs/msg/obu.hpp"
    ```

    ```cpp=
    #include <tf/tf.h>                  -> #include "tf2/utils.h"
    #include <tf/transform_listener.h>  -> #include "tf2_ros/transform_listener.h"
    #include <tf/transform_datatypes.h> -> #include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
    ```

- Syntax added in ROS 2

    ```cpp=
    #include <memory>          // if use std::shared_ptr, std::bind
    #include <functional>      // if use std::placeholders
    #include <chrono>          // if use timer and set loop rate like 50ms

    using namespace std::chrono_literals;
    ```

### 2. Variable Declaration

- ROS 1

    ```cpp=
    msgs::DetectedObjectArray g_det_obj_array;
    msgs::LidLLA g_gps;
    msgs::VehInfo g_veh_info;
    geometry_msgs::PoseStamped msg_pose;
    ```

- ROS 2

    ```cpp=
    msgs::msg::DetectedObjectArray g_det_obj_array;
    msgs::msg::LidLLA g_gps;
    msgs::msg::VehInfo g_veh_info;
    geometry_msgs::msg::PoseStamped msg_pose;
    ```

### 3. ROS Node Declaration

- ROS 1 -> ROS 2

    ```cpp=
    ros::init(argc, argv, "node");            -> rclcpp::init(argc, argv, "node");
    ros::NodeHandle node;                     -> std::shared_ptr<rclcpp::Node> node;
                                              -> node = rclcpp::Node::make_shared("node");
    ros::Rate loop_rate(10);                  -> rclcpp::Rate loop_rate(10);
    ros::ok()                                 -> rclcpp::ok()
    ros::spinOnce();                          -> rclcpp::spin_some(node);
    ros::spin();                              -> rclcpp::spin(node); rclcpp::shutdown();
    ros::Time::now();                         -> node->get_clock()->now();
    ```

### 4. ROS Publisher Declaration

- ROS 1

    ```cpp=
    static ros::Publisher othercar_pub;
    othercar_pub = n.advertise<msgs::Obu>("/obu", 1000);
    othercar_pub.publish(input);
    ```

- ROS 2

    ```cpp=
    static rclcpp::Publisher<msgs::msg::Obu>::SharedPtr othercar_pub;
    othercar_pub = n->create_publisher<msgs::msg::Obu>("/obu", 1000);
    othercar_pub->publish(input);
    ```

### 5. Callback and ROS Subscriber Declaration

- ROS 1

    ```cpp=
    void callback(const msgs::Flag_Info::ConstPtr& input){}

    static ros::Subscriber detObj = n.subscribe("LidarDetection", 1, cb1);
    ```

- ROS 2

    ```cpp=
    void callback_flag_info02(const msgs::msg::FlagInfo::SharedPtr input){}
    
    rclcpp::Subscription<msgs::msg::DetectedObjectArray>::SharedPtr detObj;
    detObj = n->create_subscription<msgs::msg::DetectedObjectArray>("LidarDetection", 1, cb1);
    ```

### 6. Get ROS Parameter by Name

- ROS 1

    ```cpp=
    ros::param::get(ros::this_node::getName()+"/force_disable_avoidance", force_disable_avoidance_);
    ```

- ROS 2

    ```cpp=
    std::string node_name_str(node->get_name());
    node->get_parameter(node_name_str+"/force_disable_avoidance", force_disable_avoidance_);
    ```

Offical Document
---

[Migration guide from ROS 1](https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html)
