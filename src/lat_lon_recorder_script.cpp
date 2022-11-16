#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <stdlib.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <math.h>
#include <algorithm>
#include <mutex>
#include <tuple>
#include "geometry_msgs/msg/vector3.hpp"
#include <fstream>
#include <random>
#include <time.h>
#include <fstream>
#include <map>
using std::placeholders::_1;
std::mutex mtx;

class LatLonRecorder : public rclcpp::Node{
private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cur_global_state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cur_map_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lon_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lat_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hdg_subscription_;

    geometry_msgs::msg::Vector3 cur_boat_global_pos;
    geometry_msgs::msg::Vector3 cur_boat_map_pos;
    std_msgs::msg::Float64 lon;
    std_msgs::msg::Float64 lat;
    std_msgs::msg::Float64 hdg;
    bool received_boat_map_pos;
    bool received_lat;
    bool received_lon;
    bool received_hdg;

    std::vector<std::pair<std::pair<double, double>, std::pair<int, int>>> tracking;
    rclcpp::TimerBase::SharedPtr step_timer_;
public:
    LatLonRecorder() : Node("lat_lon_follower") {
        this->cur_global_state_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/boat_pos_global_frame", 1, std::bind(&LatLonRecorder::receiveGlobalState, this, std::placeholders::_1));
        this->cur_map_state_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/boat_pos_map_frame", 1, std::bind(&LatLonRecorder::receiveMapState, this, std::placeholders::_1));
        this->lon_subscription_ = this->create_subscription<std_msgs::msg::Float64>("/wamv/sensors/gps/lon", 1, std::bind(&LatLonRecorder::receiveLon, this, std::placeholders::_1));
        this->lat_subscription_ = this->create_subscription<std_msgs::msg::Float64>("/wamv/sensors/gps/lat", 1, std::bind(&LatLonRecorder::receiveLat, this, std::placeholders::_1));
        this->hdg_subscription_ = this->create_subscription<std_msgs::msg::Float64>("/wamv/sensors/gps/hdg", 1, std::bind(&LatLonRecorder::receiveHdg, this, std::placeholders::_1));
        received_boat_map_pos = false;
        received_lat = false;
        received_lon = false;
        received_hdg = false;
        this->step_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(0.5), [this] {step();});
    }
    ~LatLonRecorder(){
        std::ofstream MyFile("lat_lon_x_y.txt");
        MyFile << lat.data << " " << lon.data << " " << cur_boat_map_pos.x << " " << cur_boat_map_pos.y << std::endl;
        MyFile.close();
    }
    void receiveGlobalState(const geometry_msgs::msg::Vector3 msg) {
        mtx.lock();
        this->cur_boat_global_pos = msg;
        mtx.unlock();
    }
    void receiveMapState(const geometry_msgs::msg::Vector3 msg) {
        mtx.lock();
        this->received_boat_map_pos = true;
        this->cur_boat_map_pos = msg;
        mtx.unlock();
    }
    void receiveLon(const std_msgs::msg::Float64 msg){
        mtx.lock();
        this->received_lon = true;
        this->lon = msg;
        mtx.unlock();
    }
    void receiveLat(const std_msgs::msg::Float64 msg){
        mtx.lock();
        this->received_lat = true;
        this->lat = msg;
        mtx.unlock();
    }
    void receiveHdg(const std_msgs::msg::Float64 msg){
        mtx.lock();
        this->hdg = msg;
        mtx.unlock();
    }
    void step(){
        mtx.lock();
        if(received_lat && received_lon && received_boat_map_pos){
            std::pair<double, double> lat_lon;
            lat_lon.first = lat.data;
            lat_lon.second = lon.data;
            std::pair<int, int> x_y;
            x_y.first = cur_boat_map_pos.x;
            x_y.second = cur_boat_map_pos.y;
            tracking.push_back(std::make_pair(lat_lon, x_y));
        }
        mtx.unlock();
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatLonRecorder>());
    rclcpp::shutdown();
    return 0;
}
