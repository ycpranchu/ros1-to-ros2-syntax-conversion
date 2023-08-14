#ifndef ROSMODULE_H
#define ROSMODULE_H


//--------------------------------------------------------------------------------------------------------------Traffic
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/Float64.h"
#include "msgs/msg/detected_object_array.hpp"
#include "msgs/msg/lid_lla.hpp"
#include "msgs/msg/veh_info.hpp"
#include "msgs/msg/flag_info.hpp"
#include "msgs/msg/stop_info_array.hpp"
#include "msgs/msg/stop_info.hpp"
#include "msgs/msg/route_info.hpp"
#include "msgs/msg/backend_info.hpp"
#include "msgs/msg/spat.hpp"
#include "msgs/msg/obu.hpp"
#include "msgs/msg/SpeedFeedback.h"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include <chrono>
#include <thread>

static rclcpp::Publisher<msgs::msg::Spat>::SharedPtr othercar_pub;
static rclcpp::Publisher<msgs::msg::Spat>::SharedPtr traffic_pub;
static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr backend_pub;
static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr occ_pub;
static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::currentPose_vToV_pub;
static rclcpp::Publisher<std_msgs::msg::Float64>::speedCmd_vToV_pub;

struct Pose {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class RosModuleTraffic
{
  public:
    
    static void
    Initial (int argc,
             char ** argv)
    {
      rclcpp::init(argc, argv);
		  n = rclcpp::Node::make_shared("adv_to_server");
      othercar_pub = n->create_publisher<msgs::msg::Obu>("/obu", 1000);
      traffic_pub = n->create_publisher<msgs::msg::Spat>("/traffic", 1000);
      backend_pub = n->create_publisher<std_msgs::msg::Bool>("/backend_sender/status", 1000);
      occ_pub = n->create_publisher<std_msgs::msg::Bool>("/occ_sender/status", 1000);
      currentPose_vToV_pub = n->create_publisher<geometry_msgs::msg::PoseStamped>("/vToV/currentPose", 10);
      speedCmd_vToV_pub = n->create_publisher<std_msgs::msg::Float64>("/vToV/speedCmd", 10);
    }

    static std::string getPlate(){
        n = rclcpp::Node::make_shared("adv_to_server");
        std::string plate;
        if(n->get_parameter("/south_bridge/license_plate_number", plate)){
            return plate;
        }else{
            return "DEFAULT-ITRI-ADV";
        }
    }

    static std::string getVid(){
        n = rclcpp::Node::make_shared("adv_to_server");
        std::string vid;
        if(n->get_parameter("/south_bridge/vid", vid)){
            return vid;
        }else{
            return "Default-vid";
        }
    }

    static void RegisterCallBack (void(*cb1) (const msgs::msg::DetectedObjectArray::SharedPtr),
                      void(*cb2) (const msgs::msg::LidLLA::SharedPtr),
                      void(*cb3) (const msgs::msg::VehInfo::SharedPtr),
                      void(*cb4) (const geometry_msgs::msg::PoseStamped::SharedPtr),
                      void(*cb5) (const std_msgs::msg::String::SharedPtr),
                      void(*cb6) (const msgs::msg::Flag_Info::SharedPtr),
                      void(*cb7) (const std_msgs::msg::String::SharedPtr),
                      void(*cb8) (const msgs::msg::Flag_Info::SharedPtr),
                      void(*cb9) (const std_msgs::msg::Int32::SharedPtr),
                      void(*cb10) (const sensor_msgs::msg::Imu::SharedPtr),
                      void(*cb11) (const std_msgs::msg::Bool::SharedPtr),
                      void(*cb12) (const msgs::msg::BackendInfo::SharedPtr),
                      void(*cb13) (const std_msgs::msg::String::SharedPtr),
                      void(*cb14) (const msgs::msg::DetectedObjectArray::SharedPtr),
                      void(*cb15) (const std_msgs::msg::String::SharedPtr),
                      void(*cb16) (const msgs::msg::Flag_Info::SharedPtr),
                      void(*cb17) (const msgs::msg::Flag_Info::SharedPtr),
                      void(*cb18) (const geometry_msgs::msg::PoseStamped::SharedPtr),
                      void(*cb19) (const std_msgs::msg::Float64::SharedPtr),
                      void(*cb20) (const std_msgs::msg::Float64::SharedPtr),
                      bool isNewMap) 
      {
      n = rclcpp::Node::make_shared("adv_to_server");
      rclcpp::Subscription<msgs::msg::DetectedObjectArray>::SharedPtr detObj;
      detObj = n->create_subscription<msgs::msg::DetectedObjectArray>("LidarDetection", 1, cb1);
      rclcpp::Subscription<msgs::msg::VehInfo>::SharedPtr vehInfo;
      vehInfo = n->create_subscription<msgs::msg::VehInfo>("veh_info", 1, cb3);

      if(isNewMap){
        std::cout << "===============================subscribe for new map" << std::endl;

        rclcpp::Subscription<msgs::msg::LidLLA>::SharedPtr gps;
        gps = n->create_subscription<msgs::msg::LidLLA>("lidar_lla_wgs84", 1, cb2);
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss2local_sub;
        gnss2local_sub = n->create_subscription<geometry_msgs::msg::PoseStamped>("gnss_data", 1, cb4);
      }else{
        std::cout << "===============================subscribe for old map" << std::endl;
        
        rclcpp::Subscription<msgs::msg::LidLLA>::SharedPtr gps;
        gps = n->create_subscription<msgs::msg::LidLLA>("lidar_lla", 1, cb2);
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss2local_sub;
        gnss2local_sub = n->create_subscription<geometry_msgs::msg::PoseStamped>("gnss2local_data", 1, cb4);
      }
      
      clcpp::Subscription<std_msgs::msg::String>::SharedPtr fps;
      fps = n->create_subscription<std_msgs::msg::String>("/GUI/topic_fps_out", 1, cb5);
      rclcpp::Subscription<msgs::msg::FlagInfo>::SharedPtr busStopInfo;
      busStopInfo = n->create_subscription<msgs::msg::FlagInfo>("/BusStop/Info", 1, cb6);
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reverse;
      reverse = n->create_subscription<std_msgs::msg::String>("/mileage/relative_mileage", 1, cb7);
      rclcpp::Subscription<msgs::msg::FlagInfo>::SharedPtr next_stop;
      next_stop = n->create_subscription<msgs::msg::FlagInfo>("/NextStop/Info", 1, cb8);
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr round;
      round = n->create_subscription<std_msgs::msg::Int32>("/BusStop/Round", 1, cb9);
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
      imu = n->create_subscription<sensor_msgs::msg::Imu>("imu_data_rad", 1, cb10);
      //checker big buffer for multi event at the same time.
      //get event from fail_safe

      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr checker;
      checker = n->create_subscription<std_msgs::msg::Bool>("/ADV_op/sys_ready", 1000, cb11);
      rclcpp::Subscription<msgs::msg::BackendInfo>::SharedPtr backendInfo;
      backendInfo = n->create_subscription<msgs::msg::BackendInfo>("Backend/Info", 1, cb12);
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensor_status;
      sensor_status = n->create_subscription<std_msgs::msg::String>("/vehicle/report/itri/sensor_status", 1, cb13);
      rclcpp::Subscription<msgs::msg::DetectedObjectArray>::SharedPtr tracking;
      tracking = n->create_subscription<msgs::msg::DetectedObjectArray>("/Tracking3D/xyz2lla", 100, cb14);
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fail_safe;
      fail_safe = n->create_subscription<std_msgs::msg::String>("/vehicle/report/itri/fail_safe_status", 1, cb15);
      rclcpp::Subscription<msgs::msg::FlagInfo>::SharedPtr flag04;
      flag04 = n->create_subscription<msgs::msg::FlagInfo>("/Flag_Info04", 1, cb16);
      rclcpp::Subscription<msgs::msg::FlagInfo>::SharedPtr flag02;
      flag02 = n->create_subscription<msgs::msg::FlagInfo>("/Flag_Info02", 1, cb17);

      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_currentPose;
      sub_currentPose = n->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose", 1, cb18);
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_speedInfo;
      sub_speedInfo = n->create_subscription<std_msgs::msg::Float64>("control/speed_cmd", 1, cb19);
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lidarlla_heading_sub;
      lidarlla_heading_sub = n->create_subscription<std_msgs::msg::Float64>("lidar_lla_heading", 1, cb20);
    }

    static void publishOtherCar(msgs::msg::Obu input) {
      //std::cout << "publishOtherCar topic " << topic <<  std::endl;
      othercar_pub->publish(input);
    }
    static void publishTraffic(std::string topic, msgs::msg::Spat input) {
      std::cout << "publishTraffic topic " << topic <<  std::endl;
      traffic_pub->publish(input);
    }
    static void pubBackendState(bool input) {
        std_msgs::msg::Bool result;
        result.data = input;
        backend_pub->publish(result);
    }
    static void pubOCCState(bool input) {
        std_msgs::msg::Bool result;
        result.data = input;
        occ_pub->publish(result);
    }
    static void publishServerStatus(std::string topic, bool input) {
      //std::cout << "publishServerStatus topic " << topic << " , input " << input << std::endl;
      n = rclcpp::Node::make_shared("adv_to_server");
      static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr server_status_pub;
		  server_status_pub =  n->create_publisher<std_msgs::msg::Bool>(topic, 1000);
      std_msgs::msg::Bool msg;
      msg.data = input;
      server_status_pub->publish(msg);
    }
    static void publishReserve(std::string topic, msgs::msg::StopInfoArray msg) {
      //std::cout << "publishReserve topic " << topic  << std::endl;
      n = rclcpp::Node::make_shared("adv_to_server");
      static rclcpp::Publisher<msgs::msg::StopInfoArray>::SharedPtr reserve_status_pub;
		  reserve_status_pub =  n->create_publisher<msgs::msg::StopInfoArray>(topic, 1000);
      
      short count = 0;
      while (count < 30)
      { 
        count++;
        int numOfSub = reserve_status_pub->get_subscription_count() ;
        //std::cout << "numOfSub = " << numOfSub << std::endl;
        if(numOfSub > 0) 
        {
          std::chrono::duration<int, std::milli> timespan(100);
          std::this_thread::sleep_for(timespan);
          reserve_status_pub->publish(msg);
          return;
        }
      } 
    }
    static void publishRoute(std::string topic, msgs::msg::RouteInfo msg) {
      //std::cout << "publishReserve topic " << topic  << std::endl;
      n = rclcpp::Node::make_shared("adv_to_server");
      static rclcpp::Publisher<msgs::msg::RouteInfo>::SharedPtr route_pub;
		  route_pub =  n->create_publisher<msgs::msg::RouteInfo>(topic, 1000);
      short count = 0;
      while (count < 30)
      { 
        count++;
        int numOfSub = route_pub->get_subscription_count() ;
        //std::cout << "numOfSub = " << numOfSub << std::endl;
        if(numOfSub > 0) 
        {
          std::chrono::duration<int, std::milli> timespan(100);
          std::this_thread::sleep_for(timespan);
          route_pub->publish(msg);
          return;
        }
      } 
    }
    static void publishCurrentPose(Pose aPose) {
        geometry_msgs::msg::PoseStamped msg;
        msg.pose.position.x = aPose.x;
        msg.pose.position.y = aPose.y;
        msg.pose.position.z = aPose.z;
        currentPose_vToV_pub.publish(msg);
    }
    static void publishSpeedCmd(double speedKPH) {
        std_msgs::msg::Float64 msg;
        msg.data = speedKPH;
        speedCmd_vToV_pub.publish(msg);
    }
};

#endif // ROSMODULE_H
