#include "bw_io/AsyncSerial.h"
#include "bw_io/bw_io.h"
#define DISABLE 0
#define ENABLE 1

namespace bw_io
{
StatusPublisher::StatusPublisher(CallbackAsyncSerial* cmd_serial)
                                :cmd_serial_(cmd_serial)
{
  ros::param::param<double>("~k1", k1_, 2.0);
  ros::param::param<double>("~b1", b1_, 2.5);
  ros::param::param<double>("~mvalue1", mvalue1_, 0.1);

  ros::param::param<double>("~k2", k2_, 2.0);
  ros::param::param<double>("~b2", b2_, 2.5);
  ros::param::param<double>("~mvalue2", mvalue2_, 0.1);

  ros::param::param<double>("~k3", k3_, 2.0);
  ros::param::param<double>("~b3", b3_, 2.5);
  ros::param::param<double>("~mvalue3", mvalue3_, 0.1);

  ros::param::param<double>("~k4", k4_, 2.0);
  ros::param::param<double>("~b4", b4_, 2.5);
  ros::param::param<double>("~mvalue4", mvalue4_, 0.1);

  car_status.pin_value = 0x000003ff;
  car_status.power1 = b1_;
  car_status.power2 = b2_;
  car_status.power3 = b3_;
  car_status.power4 = b4_;

  power1_ = 0;
  power2_ = 0;
  power3_ = 0;
  power4_ = 0;

  input_1_= 0;
  input_2_ = 0;
  input_3_ = 0;
  input_4_ = 0;
  input_5_ = 0;
  input_6_ = 0;
  input_7_ = 0;
  input_8_ = 0;
  input_9_ = 0;
  input_10_ = 0;
  input_11_ = 0;

  output_1_ = 1;
  output_2_ = 1;
  output_3_ = 1;
  output_4_ = 1;
  output_5_ = 1;
  output_6_ = 1;
  output_7_ = 1;
  output_8_ = 1;
  output_9_ = 1;
  output_10_ = 1;

  mbUpdated_=false;

  power1_ = car_status.power1*k1_ - b1_;
  if(power1_>=-mvalue1_ && power1_<=mvalue1_) power1_ = 0.0;

  power2_ = car_status.power2*k2_ - b2_;
  if(power2_>=-mvalue2_ && power2_<=mvalue2_) power2_ = 0.0;

  power3_ = car_status.power3*k3_ - b3_;
  if(power3_>=-mvalue3_ && power3_<=mvalue3_) power3_ = 0.0;

  power4_ = car_status.power4*k4_ - b4_;
  if(power4_>=-mvalue4_ && power4_<=mvalue4_) power4_ = 0.0;

  mJoy.header.stamp = ros::Time::now();
  mJoy.header.frame_id = "bw_io";
  mJoy.axes.resize(4);
  mJoy.buttons.resize(16);
  mJoy.axes[0] =  power1_;
  mJoy.axes[1] =  power2_;
  mJoy.axes[2] =  power3_;
  mJoy.axes[3] =  power4_;

  mJoy.buttons[0] = input_1_;
  mJoy.buttons[1] = input_2_;
  mJoy.buttons[2] = input_3_;
  mJoy.buttons[3] = input_4_;
  mJoy.buttons[4] = input_5_;
  mJoy.buttons[5] = input_6_;
  mJoy.buttons[6] = input_7_;
  mJoy.buttons[7] = input_8_;
  mJoy.buttons[8] = input_9_;
  mJoy.buttons[9] = input_10_;
  mJoy.buttons[10] = input_11_;

  mJoyPub = mNH.advertise<sensor_msgs::Joy>("/bw_io/joy", 1, true);

}

void StatusPublisher::Parsepins()
{
  unsigned int input_pins = car_status.pin_value >>16;
  unsigned int output_pins = car_status.pin_value & 0x0000ffff;
  if( (input_pins & 0x00000001)!=0)
  {
    input_1_ = 1;
  }
  else
  {
    input_1_ = 0;
  }

  if( (input_pins & 0x00000002)!=0)
  {
    input_2_ = 1;
  }
  else
  {
    input_2_ = 0;
  }

  if( (input_pins & 0x00000004)!=0)
  {
    input_3_ = 1;
  }
  else
  {
    input_3_ = 0;
  }

  if( (input_pins & 0x00000008)!=0)
  {
    input_4_ = 1;
  }
  else
  {
    input_4_ = 0;
  }

  if( (input_pins & 0x00000010)!=0)
  {
    input_5_ = 1;
  }
  else
  {
    input_5_ = 0;
  }

  if( (input_pins & 0x00000020)!=0)
  {
    input_6_ = 1;
  }
  else
  {
    input_6_ = 0;
  }

  if( (input_pins & 0x00000040)!=0)
  {
    input_7_ = 1;
  }
  else
  {
    input_7_ = 0;
  }

  if( (input_pins & 0x00000080)!=0)
  {
    input_8_ = 1;
  }
  else
  {
    input_8_ = 0;
  }

  if( (input_pins & 0x00000100)!=0)
  {
    input_9_ = 1;
  }
  else
  {
    input_9_ = 0;
  }

  if( (input_pins & 0x00000200)!=0)
  {
    input_10_ = 1;
  }
  else
  {
    input_10_ = 0;
  }

  if( (input_pins & 0x00000400)!=0)
  {
    input_11_ = 1;
  }
  else
  {
    input_11_ = 0;
  }

  if( (output_pins & 0x00000001)!=0)
  {
    output_1_ = 1;
  }
  else
  {
    output_1_ = 0;
  }

  if( (output_pins & 0x00000002)!=0)
  {
    output_2_ = 1;
  }
  else
  {
    output_2_ = 0;
  }

  if( (output_pins & 0x00000004)!=0)
  {
    output_3_ = 1;
  }
  else
  {
    output_3_ = 0;
  }

  if( (output_pins & 0x00000008)!=0)
  {
    output_4_ = 1;
  }
  else
  {
    output_4_ = 0;
  }

  if( (output_pins & 0x00000010)!=0)
  {
    output_5_ = 1;
  }
  else
  {
    output_5_ = 0;
  }

  if( (output_pins & 0x00000020)!=0)
  {
    output_6_ = 1;
  }
  else
  {
    output_6_ = 0;
  }

  if( (output_pins & 0x00000040)!=0)
  {
    output_7_ = 1;
  }
  else
  {
    output_7_ = 0;
  }

  if( (output_pins & 0x00000080)!=0)
  {
    output_8_ = 1;
  }
  else
  {
    output_8_ = 0;
  }

  if( (output_pins & 0x00000100)!=0)
  {
    output_9_ = 1;
  }
  else
  {
    output_9_ = 0;
  }

  if( (output_pins & 0x00000200)!=0)
  {
    output_10_ = 1;
  }
  else
  {
    output_10_ = 0;
  }

  power1_ = car_status.power1*k1_ - b1_;
  if(power1_>=-mvalue1_ && power1_<=mvalue1_) power1_ = 0.0;

  power2_ = car_status.power2*k2_ - b2_;
  if(power2_>=-mvalue2_ && power2_<=mvalue2_) power2_ = 0.0;

  power3_ = car_status.power3*k3_ - b3_;
  if(power3_>=-mvalue3_ && power3_<=mvalue3_) power3_ = 0.0;

  power4_ = car_status.power4*k4_ - b4_;
  if(power4_>=-mvalue4_ && power4_<=mvalue4_) power4_ = 0.0;

}

void StatusPublisher::Refresh()
{
  boost::mutex::scoped_lock lock(mMutex);
  if(mbUpdated_ )
  {
    mJoy.axes[0] =  power1_;
    mJoy.axes[1] =  power2_;
    mJoy.axes[2] =  power3_;
    mJoy.axes[3] =  power4_;

    mJoy.buttons[0] = input_1_;
    mJoy.buttons[1] = input_2_;
    mJoy.buttons[2] = input_3_;
    mJoy.buttons[3] = input_4_;
    mJoy.buttons[4] = input_5_;
    mJoy.buttons[5] = input_6_;
    mJoy.buttons[6] = input_7_;
    mJoy.buttons[7] = input_8_;
    mJoy.buttons[8] = input_9_;
    mJoy.buttons[9] = input_10_;
    mJoy.buttons[10] = input_11_;
    mJoyPub.publish(mJoy);
  }
  mbUpdated_ = false;
}

void StatusPublisher::Getdata()
{
  //下发获取数据命令
  boost::mutex::scoped_lock lock(mMutex);
  char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,'g'};
  if(NULL!=cmd_serial_)
  {
      cmd_serial_->write(cmd_str,5);
  }
}

void StatusPublisher::UpdateStatus(const char *data, unsigned int len)
{
    int i=0,j=0;
    int * receive_byte;
    static unsigned char last_str[2]={0x00,0x00};
    static unsigned char new_packed_ctr=DISABLE;//ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len=0;//包的理论长度
    static int new_packed_len=0;//包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str=0x00;
    const int cmd_string_max_size=512;
    receive_byte=(int *)&car_status;

    for(i=0;i<len;i++)
    {
        current_str=data[i];
        //判断是否有新包头
        if(last_str[0]==205&&last_str[1]==235&&current_str==215) //包头 205 235 215
        {
            //std::cout<<"runup1 "<<std::endl;
            new_packed_ctr=ENABLE;
            new_packed_ok_len=0;
            new_packed_len=new_packed_ok_len;
            last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
            last_str[1]=current_str;
            continue;
        }
        last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
        last_str[1]=current_str;
        if (new_packed_ctr==ENABLE)
        {

            //获取包长度
            new_packed_ok_len=current_str;
            if(new_packed_ok_len>cmd_string_max_size) new_packed_ok_len=cmd_string_max_size; //包内容最大长度有限制
            new_packed_ctr=DISABLE;
            //std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if(new_packed_ok_len<=new_packed_len)
            {
                //std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len-1]=current_str;
                if(new_packed_ok_len==new_packed_len&&new_packed_ok_len>0)
                {
                    //std::cout<<"runup4 "<<std::endl;
                    boost::mutex::scoped_lock lock(mMutex);
                    //当前包已经处理完成，开始处理
                    if(new_packed_ok_len==25)
                    {
                      for(j=0;j<5;j++)
                      {
                          //要校验和
                          unsigned char sum = cmd_string_buf[5*j] + cmd_string_buf[5*j+1] + cmd_string_buf[5*j+2] + cmd_string_buf[5*j+3];
                          if(sum == cmd_string_buf[5*j+4])
                          {
                            memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                            mbUpdated_=true;
                          }
                      }
                      if(mbUpdated_) Parsepins();
                    }
                    new_packed_ok_len=0;
                    new_packed_len=0;
                }
            }

        }

    }

    return;
}

void StatusPublisher::run()
{
  read_pins_srv_ = mNH.advertiseService("/bw_io/read_pins", &StatusPublisher::readPinsService, this);
  set_pins_srv_ = mNH.advertiseService("/bw_io/set_pins", &StatusPublisher::setPinsService, this);
  ros::spin();
}

bool StatusPublisher::readPinsService(bw_io::ReadPins::Request &req, bw_io::ReadPins::Response &resp)
{
  boost::mutex::scoped_lock lock(mMutex);
  resp.axes.resize(4);
  resp.input_buttons.resize(11);
  resp.output_buttons.resize(10);
  resp.axes[0] = power1_;
  resp.axes[1] = power2_;
  resp.axes[2] = power3_;
  resp.axes[3] = power4_;

  resp.input_buttons[0] = input_1_;
  resp.input_buttons[1] = input_2_;
  resp.input_buttons[2] = input_3_;
  resp.input_buttons[3] = input_4_;
  resp.input_buttons[4] = input_5_;
  resp.input_buttons[5] = input_6_;
  resp.input_buttons[6] = input_7_;
  resp.input_buttons[7] = input_8_;
  resp.input_buttons[8] = input_9_;
  resp.input_buttons[9] = input_10_;
  resp.input_buttons[10] = input_11_;

  resp.output_buttons[0] = output_1_;
  resp.output_buttons[1] = output_2_;
  resp.output_buttons[2] = output_3_;
  resp.output_buttons[3] = output_4_;
  resp.output_buttons[4] = output_5_;
  resp.output_buttons[5] = output_6_;
  resp.output_buttons[6] = output_7_;
  resp.output_buttons[7] = output_8_;
  resp.output_buttons[8] = output_9_;
  resp.output_buttons[9] = output_10_;
  return true;
}

bool StatusPublisher::setPinsService(bw_io::SetPins::Request &req, bw_io::SetPins::Response &resp)
{
  unsigned int enable_pins = 0x0000;
  unsigned int pin_values = 0x03ff;
  for(int i=0;i<req.set_buttons.size();i++)
  {
    unsigned int bit = req.set_buttons[i];
    if(bit>0)
    {
      //N号输出对应N-1号
      bit = bit -1;
      enable_pins |= (unsigned int)(1<<bit);
      unsigned int set_value = req.button_value[i];
      unsigned int set_value1 = (unsigned int)(1<<bit);
      if(set_value>0)
      {
        set_value = set_value1 + (~set_value1);
      }
      else
      {
        set_value = ~set_value1;
      }
      pin_values &= set_value;
    }
  }
  char cmd_str[10]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x06,'s',(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
  unsigned char * char_byte;
  char_byte = (unsigned char *)&enable_pins;
  cmd_str[7] = char_byte[0];
  cmd_str[8] = char_byte[1];

  char_byte = (unsigned char *)&pin_values;
  cmd_str[5] = char_byte[0];
  cmd_str[6] = char_byte[1];

  cmd_str[9] = cmd_str[5] + cmd_str[6] + cmd_str[7] + cmd_str[8];
  if(enable_pins>0)
  {
    boost::mutex::scoped_lock lock(mMutex);
    if(NULL!=cmd_serial_)
    {
        cmd_serial_->write(cmd_str,10);
        resp.code = 0;
        resp.error_message = "ok";
    }
    else
    {
      resp.code = -1;
      resp.error_message = "serial port not ready";
    }
  }
  return true;
}

}
