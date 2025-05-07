/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

#include <pcl/io/pcd_io.h>

//#define ORDERLY_EXIT

// Define the macro: 1 to enable IMU parsing, 0 to disable IMU parsing
#define ENABLE_IMU_PARSE 1 
typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;


SyncQueue<std::shared_ptr<ImuData>> free_imu_data_queue;
SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_data_queue;
//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller. 
// @param msg  The stuffed point cloud message.
//
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
  stuffed_cloud_queue.push(msg);
}

//
// @brief IMU data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed IMU data message to the caller. 
// @param msg  The stuffed IMU data message.
//
std::shared_ptr<ImuData> driverGetIMUDataFromCallerCallback(void)
{
   // Note: This callback function runs in the packet-parsing/imu-data-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<ImuData> msg = free_imu_data_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<ImuData>();
}

//
// @brief IMU data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed IMU data message to the caller. 
// @param msg  The stuffed IMU data message.
//
void driverReturnImuDataToCallerCallback(const std::shared_ptr<ImuData>& msg)
{
  // Note: This callback function runs in the packet-parsing/imu-data-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processImuData() below)
  stuffed_imu_data_queue.push(msg);
}


bool to_exit_process = false;
void processImuData(void)
{
  uint32_t imu_cnt = 0;
  while (!to_exit_process)
  {
    std::shared_ptr<ImuData> msg = stuffed_imu_data_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the IMU data msg, even it is time-consuming.

    imu_cnt++;
#if 0
    RS_MSG << "msg: " << imu_cnt << " imu data ts: " <<std::dec<<std::to_string(msg->timestamp) << RS_REND;
    RS_DEBUG  <<"imu data: " << " , linear_a_x" << msg->linear_acceleration_x 
      << " , linear_a_y " << msg->linear_acceleration_y << "  , linear_a_z" << msg->linear_acceleration_z   
      << " , angular_v_x " << msg->angular_velocity_x << " , angular_v_y" << msg->angular_velocity_y 
      << " , angular_v_z" <<msg->angular_velocity_z << RS_REND;
#endif

    free_imu_data_queue.push(msg);
  }

}

//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() << RS_REND;
}

void processCloud(const std::string& output_folder)
{

  std::string pcd_file_path = output_folder + "/cloud_";
  std::string pcd_file_suffix = ".pcd";
  
  // Create the output directory if it doesn't exist
  
  // std::cout << "Output folder: " << output_folder << std::endl;

  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      RS_WARNING << "msg is null" << RS_REND;
      continue;
    }
    // Well, it is time to process the point cloud msg, even it is time-consuming.
    RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
    
    // Save to PCD file
    std::string filename = pcd_file_path + std::to_string(msg->seq) + pcd_file_suffix;
    pcl::io::savePCDFileASCII(filename, *msg);
    RS_MSG << "Saved " << filename << RS_REND;

#if 0
    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
    {
      std::cout << std::fixed << std::setprecision(3) 
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                << std::endl;
    }
#endif

    free_cloud_queue.push(msg);
  }
}

int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Pcap Updated Demo" << RS_REND;
  if (argc < 2) {
    RS_ERROR << "Usage: " << argv[0] << " <pcap_file_path> [--output_dir=output][--msop_port=6699] [--difop_port=7788] [--imu_port=6688] [--lidar_type=RSAIRY] [--pcap_rate=1.0]" << RS_REND;
    return -1;
  }

  // -----------------------------
  // Default parameters
  // -----------------------------
  std::string pcap_file_path = argv[1];
  std::string output_dir = "output"; // Default output directory
  // TODO: Check if the output directory exists, if not, create it
  int msop_port = 7502;
  int difop_port = 7788;
  int imu_port = 6688;
  std::string lidar_type_str = "RSAIRY";
  double pcap_rate = 1.0;

  // -----------------------------
  // Parse optional parameters
  // -----------------------------
  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.find("--output_dir=") == 0)
      output_dir = arg.substr(13);
    if (arg.find("--msop_port=") == 0)
      msop_port = std::stoi(arg.substr(12));
    else if (arg.find("--difop_port=") == 0)
      difop_port = std::stoi(arg.substr(13));
#if ENABLE_IMU_PARSE
    else if (arg.find("--imu_port=") == 0)
      imu_port = std::stoi(arg.substr(11));
#endif
    else if (arg.find("--lidar_type=") == 0)
      lidar_type_str = arg.substr(13);
    else if (arg.find("--pcap_rate=") == 0)
      pcap_rate = std::stod(arg.substr(13));
    else {
      RS_WARNING << "Unknown argument: " << arg << RS_REND;
    }
  }

  // -----------------------------
  // Set up param object
  // -----------------------------
  RSDriverParam param;
  param.input_type = InputType::PCAP_FILE;
  param.input_param.pcap_path = pcap_file_path;
  param.input_param.msop_port = msop_port;
  param.input_param.difop_port = difop_port;
#if ENABLE_IMU_PARSE
  param.input_param.imu_port = imu_port;
#endif
  param.lidar_type = LidarType::RSAIRY;  // Default
  if (lidar_type_str == "RSAIRY") param.lidar_type = LidarType::RSAIRY;
  else if (lidar_type_str == "RSBP") param.lidar_type = LidarType::RSBP;
  else if (lidar_type_str == "RSP128") param.lidar_type = LidarType::RSP128;
  else if (lidar_type_str == "RS32") param.lidar_type = LidarType::RS32;
  else {
    RS_ERROR << "Unknown lidar type: " << lidar_type_str << RS_REND;
    param.lidar_type = LidarType::RSAIRY;
  }
  // TODO: Add other lidar types if needed. Update from the driver_param.hpp file.

  param.input_param.pcap_rate = pcap_rate;
  param.print();
  
  LidarDriver<PointCloudMsg> driver;               ///< Declare the driver object
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function
#if ENABLE_IMU_PARSE
  driver.regImuDataCallback(driverGetIMUDataFromCallerCallback, driverReturnImuDataToCallerCallback);
#endif
  if (!driver.init(param))                         ///< Call the init function
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  std::thread cloud_handle_thread = std::thread(processCloud, output_dir);

#if ENABLE_IMU_PARSE
  std::thread imuData_handle_thread = std::thread(processImuData);
#endif

  driver.start();  ///< The driver thread will start

  RS_DEBUG << "RoboSense Lidar-Driver Linux pcap demo start......" << RS_REND;


#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
