#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <sensor_msgs/Imu.h>//pub 
#include <mpu_imu/IMU_DiffWheel.h>

#define q30 1073741824.0f
 
serial::Serial ser; //声明串口对象
uint8_t MPU_date[80]; //定义串口数据存放数组
//回调函数
uint16_t comb16(uint8_t hi,uint8_t lo)
{
    uint16_t com = hi<<8;
    com += lo;
    return com;
}
uint32_t comb32(uint16_t hi,uint16_t lo)
{
    uint32_t com = hi<<16;
    com += lo;
    return com;
}
void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" <<msg->data);
    ser.write(msg->data);   //发送串口数据
}
 
int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
 
    //订阅主题，并配置回调函数
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    ros::Publisher IMU_pub = nh.advertise<mpu_imu::IMU_DiffWheel>("/imu/data", 20); 
    try
    {
    //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
 
    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
 
    //指定循环的频率
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
 
        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        mpu_imu::IMU_DiffWheel Imu_Difwheel;
        //sensor_msgs::Imu imu_data;
        Imu_Difwheel.header.stamp = ros::Time::now();
        Imu_Difwheel.header.frame_id = "base_link";
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            ser.read(MPU_date,80);//从缓冲区中读取80位数据，存放到定义好的数组中
            for(int i =0 ;i<42;i++) 
            { 
                if(MPU_date[i]==0xff) //帧头0
                { 
                    if(MPU_date[i+1]==0x00) //帧头1
                    {
                        if(MPU_date[i+2]==0xfe) //帧头2
                        { 
                            //wheel movement conunt
                            int LeftEncoder = MPU_date[i+3];
                            if(MPU_date[i+4]==2) LeftEncoder*=-1;
                            int RightEncoder = MPU_date[i+5];
                            if(MPU_date[i+6]==2) RightEncoder*=-1;
                            double pitch = ((double)(comb16(MPU_date[i+7],MPU_date[i+8])-15000))/100.0f;
                            double roll = ((double)(comb16(MPU_date[i+9],MPU_date[i+10])-15000))/100.0f;
                            double yaw = ((double)(comb16(MPU_date[i+11],MPU_date[i+12])-15000))/100.0f;
                            double gyro_x=((double)(comb16(MPU_date[i+13],MPU_date[i+14])-32768))/32.768f;//range -1000~+1000
                            double gyro_y=((double)(comb16(MPU_date[i+15],MPU_date[i+16])-32768))/32.768f;
                            double gyro_z=((double)(comb16(MPU_date[i+17],MPU_date[i+18])-32768))/32.768f;
                            double acc_x=((double)(comb16(MPU_date[i+19],MPU_date[i+20])-32768))*9.81/16384.0f;//range -2g~+2g
                            double acc_y=((double)(comb16(MPU_date[i+21],MPU_date[i+22])-32768))*9.81/16384.0f;// m/s^2
                            double acc_z=((double)(comb16(MPU_date[i+23],MPU_date[i+24])-32768))*9.81/16384.0f;
                            double q0=((int)comb32(comb16(MPU_date[i+25],MPU_date[i+26]),comb16(MPU_date[i+27],MPU_date[i+28])))/q30;
                            double q1=((int)comb32(comb16(MPU_date[i+29],MPU_date[i+30]),comb16(MPU_date[i+31],MPU_date[i+32])))/q30;
                            double q2=((int)comb32(comb16(MPU_date[i+33],MPU_date[i+34]),comb16(MPU_date[i+35],MPU_date[i+36])))/q30;
                            double q3=((int)comb32(comb16(MPU_date[i+37],MPU_date[i+48]),comb16(MPU_date[i+39],MPU_date[i+40])))/q30;
                            

                        // ROS_INFO("Lfspeed:%d;Rgspeed:%d;",LeftEncoder,RightEncoder); 
                        // ROS_INFO("pitch:%lf;roll:%lf,yaw:%lf",pitch,roll,yaw); 
                        // ROS_INFO("gyro_x:%lf;gyro_y:%lf,gyro_z:%lf",gyro_x,gyro_y,gyro_z); 
                        // ROS_INFO("acc_x:%lf;acc_y:%lf,acc_z:%lf",acc_x,acc_y,acc_z);
                        // 
                        double Roll, Pitch, Yaw;
                        Roll = asin(-2 * q1 * q3 + 2 * q0* q2);   //y-axis roll
                        Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); // x-axis pitch
                        Yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);//z-axis yaw
                        ROS_INFO("pitch:%lf;roll:%lf,yaw:%lf",-Pitch*57.2958,Roll*57.2958,Yaw*57.2958);
                        double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
                        q0 = q0 / norm;
                        q1 = q1 / norm;
                        q2 = q2 / norm;
                        q3 = q3 / norm;
                        ROS_INFO("q0:%lf;q1:%lf,q2:%lf,q3:%lf",q0,q1,q2,q3);
                        Imu_Difwheel.orientation.x=q1;//wxyz
                        Imu_Difwheel.orientation.y=q2;
                        Imu_Difwheel.orientation.z=q3;
                        Imu_Difwheel.orientation.w=q0;


                        Imu_Difwheel.angular_velocity.x=gyro_x;
                        Imu_Difwheel.angular_velocity.y=gyro_y;
                        Imu_Difwheel.angular_velocity.z=gyro_z;

                        Imu_Difwheel.linear_acceleration.x=acc_x;
                        Imu_Difwheel.linear_acceleration.y=acc_y;
                        Imu_Difwheel.linear_acceleration.z=acc_z;

                        Imu_Difwheel.LeftSpeed = LeftEncoder*0.0513127;//0.0637m*2pi/1560*200Hz;
                        Imu_Difwheel.RightSpeed= RightEncoder*0.0513127;
                        Imu_Difwheel.Speed = (Imu_Difwheel.LeftSpeed+Imu_Difwheel.RightSpeed)/2;
                        Imu_Difwheel.Yaw_velocity = (Imu_Difwheel.RightSpeed-Imu_Difwheel.LeftSpeed)/15.504;

                        //int yaw =comb16(MPU_date[i+3],MPU_date[i+4]);
                        //ROS_INFO_STREAM("yaw :"<<ruler.yaw/10); 
                        //int pitch =comb16(MPU_date[i+6],MPU_date[i+7]);
                        //ROS_INFO_STREAM("pitch :"<<ruler.pitch/10); 
                        //int roll =comb16(MPU_date[i+8],MPU_date[i+9]);
                            //ROS_INFO_STREAM("roll :"<<ruler.roll/10); } 
                            /*解析出GPS数据*/ 
                        }
                    } 

                }
            }

            IMU_pub.publish(Imu_Difwheel);
            
            //read_pub.publish(result);
        }
        ser.flush ();//清空串口存储空间
        loop_rate.sleep();
 
    }
} 