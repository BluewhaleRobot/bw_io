#ifndef BWIO_H
#define BWIO_H

#include "bw_io/AsyncSerial.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include "bw_io/ReadPins.h"
#include "bw_io/SetPins.h"

#define PI 3.14159265

namespace bw_io
{
typedef struct {
  unsigned int pin_value;//端口状态
  float power1;//模拟通道1
  float power2;//模拟通道1
  float power3;//模拟通道1
  float power4;//模拟通道1
}SENSOR_STATUS;

class StatusPublisher
{
public:
    StatusPublisher(CallbackAsyncSerial* cmd_serial);
    void Refresh();
    void Getdata();
    void UpdateStatus(const char *data, unsigned int len);
    void run();
    void Parsepins();
    bool readPinsService(bw_io::ReadPins::Request &req, bw_io::ReadPins::Response &resp);
    bool setPinsService(bw_io::SetPins::Request &req, bw_io::SetPins::Response &resp);
    SENSOR_STATUS car_status;

private:
    sensor_msgs::Joy mJoy;
    ros::NodeHandle mNH;
    ros::Publisher mJoyPub;
    bool mbUpdated_;
    boost::mutex mMutex;
    CallbackAsyncSerial* cmd_serial_;

    ros::ServiceServer read_pins_srv_;
    ros::ServiceServer set_pins_srv_;

    double k1_,b1_,mvalue1_;
    double k2_,b2_,mvalue2_;
    double k3_,b3_,mvalue3_;
    double k4_,b4_,mvalue4_;

    float power1_;
    float power2_;
    float power3_;
    float power4_;

    int input_1_;
    int input_2_;
    int input_3_;
    int input_4_;
    int input_5_;
    int input_6_;
    int input_7_;
    int input_8_;
    int input_9_;
    int input_10_;
    int input_11_;

    int output_1_;
    int output_2_;
    int output_3_;
    int output_4_;
    int output_5_;
    int output_6_;
    int output_7_;
    int output_8_;
    int output_9_;
    int output_10_;

};

} //bw_io

#endif // BWIO_H
