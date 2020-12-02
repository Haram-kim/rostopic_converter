#include <ros/ros.h>
#include <fstream>
#include <iostream>
// messages
#include <sensor_msgs/Imu.h>

struct ImuData{
    double time_stamp;
    geometry_msgs::Vector3 linear_acceleration;
    geometry_msgs::Vector3 angular_velocity;
};

class ImuGrabber
{
public:

    ImuGrabber(std::string filename);
    ~ImuGrabber();

    void GrabImu(const sensor_msgs::ImuConstPtr& msg);
    void WriteImu(const ImuData& imuData);

    std::ofstream writeFile;

};

ImuGrabber::ImuGrabber(std::string filename){
    std::string save_dir;
    save_dir.append(filename);
    save_dir.append("imu.txt");
    writeFile.open(save_dir, std::ios::trunc);

    if(writeFile.is_open()){
        std::cout << "Success to open imu writer" << std::endl;
        writeFile << "# imu\n";
        writeFile << "# acceleration gyroscope\n";
        writeFile << "# timestamp ax ay az gx gy gz\n";
    }
    else{
        std::cout << "Fail to open imu writer" << std::endl;
        
        std::string folder_create_command = "mkdir " + filename;
    
        if(system(folder_create_command.c_str()) == 0){
            std::cout << "Create directory at "<< filename << std::endl;
            writeFile.open(save_dir, std::ios::trunc);

            if(writeFile.is_open()){
                std::cout << "Success to open imu writer" << std::endl;
                writeFile << "# imu\n";
                writeFile << "# acceleration gyroscope\n";
                writeFile << "# timestamp ax ay az gx gy gz\n";
            }
            else{
                std::cout << "Fail to open imu writer again" << std::endl;
            }
        }
        else{
            std::cout << "Fail to create directory at " << filename << std::endl;
        }

        
    }
}

ImuGrabber::~ImuGrabber(){
    writeFile.close();
    std::cout << "Success to close imu writer" << std::endl;
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    static int lastseq = msg->header.seq - 1;
    lastseq = msg->header.seq;

    ImuData imuData;
    imuData.angular_velocity = msg->angular_velocity;
    imuData.linear_acceleration = msg->linear_acceleration;
    imuData.time_stamp = msg->header.stamp.toSec();
    WriteImu(imuData);
}

void ImuGrabber::WriteImu(const ImuData& imuData)
{
    writeFile << imuData.time_stamp << "\t";
    writeFile << imuData.angular_velocity.x << "\t";
    writeFile << imuData.angular_velocity.y << "\t";
    writeFile << imuData.angular_velocity.z << "\t";
    writeFile << imuData.linear_acceleration.x << "\t";
    writeFile << imuData.linear_acceleration.y << "\t";
    writeFile << imuData.linear_acceleration.z << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_bridge");
    ros::start();

    std::string filename = "~/";

    ros::NodeHandle nodeHandler("~");
    nodeHandler.param<std::string>("filename", filename, "");

    ImuGrabber imuGrabber(filename);

    ros::Subscriber imu_sub = nodeHandler.subscribe("/imu", 100, &ImuGrabber::GrabImu, &imuGrabber);

    ros::spin();

    ros::shutdown();
    return 0;
}
