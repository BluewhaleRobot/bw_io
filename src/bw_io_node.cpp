#include "bw_io/AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "bw_io/bw_io.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to bw_io serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "bw_io_serial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/bwSensors");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;


    try {
        CallbackAsyncSerial serial(port,baud);
        bw_io::StatusPublisher bwSensors_server(&serial);
        serial.setCallback(boost::bind(&bw_io::StatusPublisher::UpdateStatus,&bwSensors_server,_1,_2));
        boost::thread cmd2serialThread(&bw_io::StatusPublisher::run,&bwSensors_server);

        ros::Rate r(100);
        int i =0;
        while (ros::ok())
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port closed unexpectedly"<<endl;
                break;
            }
            if(i%3 ==0 ) bwSensors_server.Refresh();//定时发布状态, 发布周期为30hz
            i++;
            bwSensors_server.Getdata();
            r.sleep();
        }
        quit:
        serial.close();
    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }
    ros::shutdown();
    return 0;
}
