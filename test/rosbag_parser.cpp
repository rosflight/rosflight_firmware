#include <iostream>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <ostream>

#include <yaml-cpp/yaml.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "rosflight.h"
#include "test_board.h"
#include "mavlink.h"

using namespace std;

inline bool file_exists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}


void load_parameters(const string filename, rosflight_firmware::ROSflight& RF)
{
  (void) RF;
  if (!file_exists(filename))
  {
    cout << "unable to find parameter file " << filename << endl;
    return;
  }

  try
  {
    YAML::Node node = YAML::LoadFile(filename);
    for (auto it = node.begin(); it != node.end(); it++)
    {
      if ((*it)["type"].as<int>() == 6)
          RF.params_.set_param_by_name_int((*it)["name"].as<string>().c_str(), (*it)["value"].as<int>());
      else if ((*it)["type"].as<int>() == 9)
        RF.params_.set_param_by_name_float((*it)["name"].as<string>().c_str(), (*it)["value"].as<float>());
      else
        throw std::runtime_error("unrecognized parameter type");
    }
    int debug = 1;
    (void)debug;
    (void)node;
  }
  catch (...)
  {
    std::cout << "Failed to Read yaml file " << filename << std::endl;
  }
}




int main(int argc, char * argv[])
{
  string bag_filename = "";
  bool verbose = false;
  double start_time = 0;
  double duration = INFINITY;
  string param_filename = "";
  for (int i = 0; i < argc; i++)
  {
    string arg = argv[i];
    if (arg == "-h" || argc == 1 || arg == "--help")
    {
      cout << "USAGE: vi_ekf_rosbag [options]" << "\n\n";
      cout << "Options:\n";
      cout << "\t -h, --help\tShow this help message and exit\n";
      cout << "\t -f FILENAME\tBagfile to parse\n";
      cout << "\t -p FILENAME\tParameter file to load\n";
      cout << "\t -s START_TIME\tstart time of bag (seconds)\n";
      cout << "\t -u DURATION\tduration to run bag (seconds)\n";
      cout << "\t -v Show Verbose Output\n";
      cout << endl;
      return 0;
    }
    else if (arg == "-f")
    {
      if (i + 1 >= argc)
      {
        cout << "Please supply bag filename" << endl;
        return 0;
      }
      bag_filename = argv[++i];
    }
    else if (arg == "-p")
    {
      if (i + 1 >= argc)
      {
        cout << "Please supply parameter filename" << endl;
        return 0;
      }
      param_filename = argv[++i];
    }
    else if (arg == "-s")
    {
      if (i + 1 >= argc)
      {
        cout << "Please specify start time" << endl;
        return 0;
      }
      start_time = atof(argv[++i]);
    }
    else if (arg == "-u")
    {
      if (i + 1 >= argc)
      {
        cout << "Please specify duration" << endl;
        return 0;
      }
      duration = atof(argv[++i]);
    }
    else if (arg == "-v")
    {
      verbose = true;
    }
    else if (i == 0)
    {
      continue;
    }
  }

  if (bag_filename.empty())
  {
    cout << "Please Specify bag file" << endl;
    return -1;
  }

  rosbag::Bag bag;
  try
  {
    bag.open(bag_filename.c_str(), rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    ROS_ERROR("unable to load rosbag %s, %s", bag_filename.c_str(), e.what());
    return -1;
  }
  rosbag::View view(bag);

  // Get list of topics and print to screen - https://answers.ros.org/question/39345/rosbag-info-in-c/
  if (verbose)
  {
    vector<const rosbag::ConnectionInfo *> connections = view.getConnections();
    vector<string> topics;
    vector<string> types;
    cout << "\nloaded bagfile: " << bag_filename << "\n===================================\n";
    cout << "Topics\t\tTypes\n----------------------------\n\n" << endl;
    foreach(const rosbag::ConnectionInfo *info, connections) {
      topics.push_back(info->topic);
      types.push_back(info->datatype);
      cout << info->topic << "\t\t" << info->datatype << endl;
    }
  }

  // Figure out the end time of the bag
  double end_time = start_time + duration;
  end_time = (end_time < view.getEndTime().toSec() - view.getBeginTime().toSec()) ? end_time : view.getEndTime().toSec() - view.getBeginTime().toSec();
  if (verbose)
    cout << "Playing bag from: = " << start_time << "s to: " << end_time << "s" << endl;

  // Create the ROSflight object
  rosflight_firmware::testBoard board;
  rosflight_firmware::Mavlink mavlink(board);
  rosflight_firmware::ROSflight RF(board, mavlink);
  RF.init();

  if (!param_filename.empty())
  {
    load_parameters(param_filename, RF);
  }

  // Get some time variables
  ros::Time bag_start = view.getBeginTime() + ros::Duration(start_time);
  ros::Time bag_end = view.getBeginTime() + ros::Duration(end_time);
  board.set_time(bag_start.toNSec()/1000);

  // Prepare the output file
  fstream est_log, truth_log, imu_log;
  est_log.open("estimate.bin", std::ofstream::out | std::ofstream::trunc);
  truth_log.open("truth.bin", std::ofstream::out | std::ofstream::trunc);
  imu_log.open("imu.bin", std::ofstream::out | std::ofstream::trunc);


  foreach (rosbag::MessageInstance const m, view)
  {
    // skip messages before start time
    if (m.getTime() < bag_start) continue;

    // End bag after duration has passed
    if (m.getTime() > bag_end) break;


    /// Call all the callbacks

    // Cast datatype into proper format and call the appropriate callback
    string datatype = m.getDataType();

    if (datatype.compare("sensor_msgs/Imu") == 0)
    {
      const sensor_msgs::ImuConstPtr imu(m.instantiate<sensor_msgs::Imu>());

      // Move the board time forward
      float acc[3] = {(float)imu->linear_acceleration.x,
                      (float)imu->linear_acceleration.y,
                      (float)imu->linear_acceleration.z};
      float gyro[3] = {(float)imu->angular_velocity.x,
                       (float)imu->angular_velocity.y,
                       (float)imu->angular_velocity.z};
      uint64_t t_us = (imu->header.stamp - bag_start).toNSec()/1000;

      board.set_imu(acc, gyro, t_us);
      RF.run();
      double est[8] = {(double) t_us/1e6,
                       (double) RF.estimator_.state().attitude.w,
                       (double) RF.estimator_.state().attitude.x,
                       (double) RF.estimator_.state().attitude.y,
                       (double) RF.estimator_.state().attitude.z,
                       (double) RF.estimator_.bias().x,
                       (double) RF.estimator_.bias().y,
                       (double) RF.estimator_.bias().z};
      est_log.write((char*) est, sizeof(est));

      double imud[7] = {(double) t_us/1e6,
                       (double)acc[0], (double)acc[1], (double)acc[2],
                       (double)gyro[0], (double)gyro[1], (double)gyro[2]};

      imu_log.write((char*) imud, sizeof(imud));
    }

    else if (datatype.compare("geometry_msgs/PoseStamped") == 0)
    {
      const geometry_msgs::PoseStampedConstPtr pose(m.instantiate<geometry_msgs::PoseStamped>());
      double t = (pose->header.stamp - bag_start).toSec();
      double truth[5] = {t,
                         pose->pose.orientation.w,
                         pose->pose.orientation.x,
                         pose->pose.orientation.y,
                         pose->pose.orientation.z};
      truth_log.write((char*) truth, sizeof(truth));
    }

    else if (datatype.compare("geometry_msgs/TransformStamped") == 0)
    {
      const geometry_msgs::TransformStampedConstPtr trans(m.instantiate<geometry_msgs::TransformStamped>());
      double t = (trans->header.stamp - bag_start).toSec();
      double truth[5] = {t,
                         trans->transform.rotation.w,
                         trans->transform.rotation.x,
                         trans->transform.rotation.y,
                         trans->transform.rotation.z};
      truth_log.write((char*) truth, sizeof(truth));
    }
  }
}



