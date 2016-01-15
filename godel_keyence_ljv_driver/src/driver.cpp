/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Caterpillar, Inc. (Matthew T. West)
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Caterpillar Inc., nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Minimal, Godel project-specific driver for the Keyence LJ-V (7000) Ultra-High
 * Speed In-Line Profilometers.
 *
 * Tested with LJ-V7080 heads only.
 *
 * Uses synchronous GetProfile (0x42) request/reply communication with the
 * Keyence LJ-V7000 controller.
 */
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <godel_keyence_ljv_driver/KeyenceConfig.h>

#include <godel_keyence_ljv_driver/Keyence_tcp_client.h>
#include <simple_message/byte_array.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sysexits.h>

#include <limits>

// pkg local includes
#include "ljv7_rawdata.h"

// keyence protocol / profile related defines
#define KEYENCE_DEFAULT_TCP_PORT 24691
#define KEYENCE_DEFAULT_TCP_PORT_HS 24692

#define KEYENCE_RET_CODE_NO_DATA 0xA0

#define KEYENCE_PDEPTH_UNIT 0.01d // in micro meters

// TODO: only a difference of 2; most likely some (undocumented) flag?
#define KEYENCE_INFINITE_DISTANCE_VALUE -524288
#define KEYENCE_INFINITE_DISTANCE_VALUE2 -524286

// values LJ Navigator uses for out-of-range points (in meters)
#define KEYENCE_INFINITE_DISTANCE_VALUE_SI -999.9990d / 1e3
#define KEYENCE_INFINITE_DISTANCE_VALUE_SI2 -999.9970d / 1e3

// default values for parameters
#define DEFAULT_SAMPLE_RATE 10.0
#define DEFAULT_FRAME_ID "sensor_optical_frame"

// error codes for receive_get_profile_response(..)
#define RESP_ERR_OK 0
#define RESP_ERR_PFX_LEN 1
#define RESP_ERR_UNL_PFX_LEN 2
#define RESP_ERR_PKT_BUFF_OVERFLOW 3
#define RESP_ERR_RECV_PAYLOAD 4
#define RESP_ERR_EAGAIN 5
#define RESP_ERR_BRC 6
#define RESP_ERR_ABN_BODY_LEN 7
#define RESP_ERR_HDR_RET_CODE 8

// local types
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

typedef struct
{
  profile_point_t* points; // array with unpacked data points
  uint32_t num_points;     // number of data points in array
  uint16_t data_unit;      // 'profile data unit' (see excel sheet)
  int32_t x_start;         // '1st point X coordinate' (see excel sheet)
  int32_t x_increment;     // 'profile data X direction interval' (see excel sheet)
} keyence_profile_t;

typedef struct
{
  double pc_scale_factor;
  bool cnv_inf_pts;
} config_t;

/**
 * Extract type at 'offset' into byte array pointed to by 'buf'.
 */
template <typename T> static T extract_field(unsigned char* buf, uint32_t offset)
{
  return (*((T*)(buf + offset)));
}

// prototypes
bool send_get_profile_request(industrial::tcp_client::Keyence_TcpClient& tcp_client);
int receive_get_profile_response(industrial::tcp_client::Keyence_TcpClient& tcp_client,
                                 unsigned char* response_data, uint32_t* response_data_sz);
int unpack_response_command_data(unsigned char* cmd_data, uint32_t cmd_data_sz,
                                 keyence_profile_t& profile);
int keyence_profile_to_pc(keyence_profile_t& profile, point_cloud_t::Ptr msg, bool cnv_inf_pts,
                          double scale_factor);

// TODO: refactor node to (a) proper clas(ses), avoid global variables
config_t config_;

void dr_callback(godel_keyence_ljv_driver::KeyenceConfig& config, uint32_t level)
{
  ROS_DEBUG("Reconfigure Request (scale_factor: %.2f; "
            "cnv_inf_pts: %s)",
            config.scale_factor, config.cnv_inf_pts ? "true" : "false");

  config_.pc_scale_factor = config.scale_factor;
  config_.cnv_inf_pts = config.cnv_inf_pts;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyence_lj_driver");
  ros::NodeHandle nh, pnh("~");

  // ros parameters
  std::string sensor_host;
  std::string head_a_model;
  std::string frame_id;
  int sensor_port;
  double sample_rate;

  // simple message socket comm
  industrial::tcp_client::Keyence_TcpClient tcp_client;
  unsigned char response_data[4096]; // TODO: magic nr
  uint32_t response_data_sz = 0;
  keyence_profile_t profile;

  // init profile var
  memset(&profile, 0, sizeof(keyence_profile_t));

  // dynamic reconfigure init
  dynamic_reconfigure::Server<godel_keyence_ljv_driver::KeyenceConfig> dr_server(pnh);
  dynamic_reconfigure::Server<godel_keyence_ljv_driver::KeyenceConfig>::CallbackType f;
  f = boost::bind(&dr_callback, _1, _2);
  dr_server.setCallback(f);

  // param init.
  // check required parameters
  if (!pnh.hasParam("controller_ip"))
  {
    ROS_FATAL("Parameter 'controller_ip' missing. Cannot continue.");
    return EX_CONFIG;
  }
  if (!pnh.hasParam("head_a_model"))
  {
    ROS_FATAL("Parameter 'head_a_model' missing. Cannot continue.");
    return EX_CONFIG;
  }

  pnh.getParam("controller_ip", sensor_host);
  pnh.getParam("head_a_model", head_a_model);
  pnh.param("controller_port", sensor_port, KEYENCE_DEFAULT_TCP_PORT);
  pnh.param("sample_rate", sample_rate, DEFAULT_SAMPLE_RATE);
  pnh.param<std::string>("frame_id", frame_id, DEFAULT_FRAME_ID);

  ROS_INFO("Connecting to %s (TCP %d), expecting a single %s head", sensor_host.c_str(),
           sensor_port, head_a_model.c_str());

  ROS_INFO("Scaling profiles %.2f times", config_.pc_scale_factor);
  ROS_INFO("Attempting to publish at %.2f Hz", sample_rate);

  if (config_.cnv_inf_pts)
    ROS_INFO("Profile points at infinite distances published with Z: +Inf (REP-117)");
  else
    ROS_INFO("Profile points at infinite distances published with Z: %.2f (m)",
             KEYENCE_INFINITE_DISTANCE_VALUE_SI);

  // setup point cloud message (we reuse single one)
  // TODO: this won't work with nodelets
  point_cloud_t::Ptr pc_msg(new point_cloud_t);
  pc_msg->header.frame_id = frame_id;
  // message is essentially a line-strip of points
  pc_msg->height = 1;

  // set up profile cloud publisher
  ros::Publisher pub = nh.advertise<point_cloud_t>("profiles", 10);

  // socket comm init
  tcp_client.init(const_cast<char*>(sensor_host.c_str()), sensor_port);
  if (!tcp_client.makeConnect())
  {
    ROS_FATAL("Could not connect to controller at %s (TCP %d). Aborting", sensor_host.c_str(),
              sensor_port);
    return EX_IOERR;
  }

  /**
   * Main loop:
   *
   *  1. send GetProfile request
   *  2. receive GetProfile response
   *  3. unpack profile data
   *  4. convert to point cloud
   *  5. publish
   */
  int res = 0;
  ros::Rate sleeper(sample_rate);
  while (ros::ok() /* && still connected*/)
  {
    // sleep?
    sleeper.sleep();

    // for dynamic reconfigure
    ros::spinOnce();

    // avoid interacting with sensor if there are no publishers
    // TODO: maybe we should actually always poll sensor, but just not
    //       publish anything (although unpacking + publishing is cheap)
    if (pub.getNumSubscribers() == 0)
    {
      ROS_INFO_THROTTLE(60, "No (more) subscribers. Not polling sensor.");
      continue;
    }

    if (!send_get_profile_request(tcp_client))
    {
      ROS_FATAL("Sending GetProfile request failed. Aborting");
      break;
    }

    if ((res = receive_get_profile_response(tcp_client, response_data, &response_data_sz)) < 0)
    {
      if (res == -RESP_ERR_EAGAIN)
      {
        // Warn user at maximum rate of 1 msg per second, so as to not swamp
        // the terminal / log (too much)
        ROS_WARN_THROTTLE(1, "Sensor out-of-data, backing off (sampling rate too high?)");
        // and try again later
        continue;
      }

      ROS_FATAL("Receiving GetProfile response failed. Aborting");
      break;
    }

    // unpack profile data
    memset(profile.points, 0, profile.num_points * sizeof(profile_point_t));
    res = unpack_response_command_data(response_data, response_data_sz, profile);
    if (res < 0)
    {
      ROS_WARN("Problem unpacking GetProfile response data, ignoring profile (err: %d)", res);
      continue;
    }

    // convert to pointcloud
    pc_msg->points.clear();
    res = keyence_profile_to_pc(profile, pc_msg, config_.cnv_inf_pts, config_.pc_scale_factor);
    // res can only be 0 here

    // publish pointcloud
    pub.publish(pc_msg);
  }

  // TODO: socket should be properly closed here, but
  //       industrial::tcp_client::TcpClient doesn't seem to allow us?

  return EX_OK;
}

bool send_get_profile_request(industrial::tcp_client::Keyence_TcpClient& tcp_client)
{
  // TODO: lots of magic nrs (protocol constants)
  industrial::byte_array::ByteArray buf;
  buf.init();

  // setup pkt according to documentation
  buf.load(0x00000020); // total pkt length: 32 bytes

  buf.load(0x00F00001); // it's a request & pkg version
  buf.load(0x0);        // 'fixed as 0x00'
  buf.load(0x00000014); // body length: 20 bytes

  buf.load(0x42); // command code: get profile

  buf.load(0x0);        // profile bank: active surface (page 9, comm lib refman)
  buf.load(0x0);        // profile pos : from current (page 9, comm lib refman)
  buf.load(0x1);        // nr of profile (how many profiles do we want in a single reply)
  buf.load(0x00000101); // last bits: 'nr of demanded profile' & 'erase reading data'
                        // 'nr of demanded profile' only valid/necessary when 'profile pos'
                        // has been set to '2: specify position'
                        // 'erase reading data' will remove the profile after
                        // it has been read

  ROS_DEBUG("Sending GetProfile (%d bytes)", buf.getBufferSize());

  return tcp_client.my_sendBytes(buf);
}

/**
 * Returns the 'response data' part of a GetProfile reply
 */
int receive_get_profile_response(industrial::tcp_client::Keyence_TcpClient& tcp_client,
                                 unsigned char* response_data, uint32_t* response_data_sz)
{
  // TODO: magic nr: large 'enough'
  const uint32_t buf_sz = 4096;
  unsigned char buf[buf_sz];
  uint32_t buf_ptr = 0;
  uint32_t pkt_len = 0;

  industrial::byte_array::ByteArray barray;
  barray.init();

  // first receive prefix.pkt_len, so we know how large pkt is
  if (!tcp_client.my_receiveBytes(barray, sizeof(pkt_len)))
  {
    ROS_ERROR("Error receiving prefix.pkt_len. Aborting");
    return -RESP_ERR_PFX_LEN;
  }

  ROS_DEBUG("raw pkt_len: 0x%X", *((uint32_t*)barray.getRawDataPtr()));

  if (!barray.unload(&pkt_len, sizeof(pkt_len)))
  {
    ROS_ERROR("Error unloading prefix.pkt_len. Aborting");
    return -RESP_ERR_UNL_PFX_LEN;
  }

  if (pkt_len >= buf_sz)
  {
    ROS_ERROR("Incoming pkt too large for buffer (%d > %d). Aborting", pkt_len, buf_sz);
    return -RESP_ERR_PKT_BUFF_OVERFLOW;
  }

  // now wait for at least enough for pkt_len
  while (buf_ptr < pkt_len)
  {
    barray.init();
    const uint32_t to_receive = std::min<uint32_t>(pkt_len - buf_ptr, barray.getMaxBufferSize());
    ROS_DEBUG("Reading %d bytes from socket", to_receive);

    // TODO: This is blocking, no way to timeout from this
    if (!tcp_client.my_receiveBytes(barray, to_receive))
    {
      ROS_ERROR("Error receiving GetProfile response part. Aborting");
      return -RESP_ERR_RECV_PAYLOAD;
    }

    // assume we have the number of bytes we requested (TODO: make sure we do)
    memcpy(&buf[buf_ptr], barray.getRawDataPtr(), to_receive);
    buf_ptr += to_receive;
  }

  if (buf_ptr > pkt_len)
  {
    ROS_WARN("GetProfile response was larger than "
             "expected (%d > %d)",
             buf_ptr, pkt_len);
  }

  // extract fields from raw pkt data
  const uint8_t return_code = extract_field<uint8_t>(buf, 4);
  const uint32_t body_length = extract_field<uint32_t>(buf, 8);

  ROS_DEBUG("GetProfile response with %d retcode. Body length: %d", return_code, body_length);

  // check reply, make sure we got what we expected
  // TODO: magic nr
  if (return_code != 0)
  {
    ROS_WARN("Received unexpected return code "
             "from sensor for GetProfile: 0x%02X",
             return_code);
    return -RESP_ERR_HDR_RET_CODE;
  }

  // see if we have enough to check the 'body return code'
  if (body_length >= 2)
  {
    // 12 bytes of response header, +1 for the 'command code' byte
    // in the response body
    const uint8_t body_return_code = extract_field<uint8_t>(buf, 13);

    if (body_return_code == KEYENCE_RET_CODE_NO_DATA)
    {
      return -RESP_ERR_EAGAIN;
    }
    else if (body_return_code != 0)
    {
      ROS_WARN("GetProfile response error: 0x%02X.", body_return_code);
      return -RESP_ERR_BRC;
    }
  }

  // TODO: magic (and arbitrary) nr
  if (body_length < 512 /*bytes*/)
  {
    ROS_WARN("Abnormal (for GetProfile) body length encountered "
             "(%d bytes), ignoring response",
             body_length);
    return -RESP_ERR_ABN_BODY_LEN;
  }

  // everything ok: extract response body data for caller
  const uint32_t pfx_len = 4;
  const uint32_t hdr_len = 12;
  const uint32_t resp_body_bits_len = 12; // response body without response data

  const uint32_t command_data_offset = hdr_len + resp_body_bits_len;
  const uint32_t command_data_sz = body_length - resp_body_bits_len;

  memcpy(response_data, &buf[command_data_offset], command_data_sz);

  // done
  return RESP_ERR_OK;
}

// assumption: 'profile' already initialised, with 'points' array already alloc-ed
// assumption2: single head, no compression
int unpack_response_command_data(unsigned char* cmd_data, uint32_t cmd_data_sz,
                                 keyence_profile_t& profile)
{
  /* Need:
   *  - number of profile points
   *  - data unit
   *  - x_start
   *  - x_increment
   *  - unpacked points
   */

  // all offsets relative to start of response data
  const uint32_t profile_info_sz = 12;
  const uint32_t profile_info_offset = 24;
  const uint32_t profile_data_offset = profile_info_offset + profile_info_sz;

  // get at profile information within response data
  unsigned char* profile_info = cmd_data + profile_info_offset;

  // get at profile data within response data
  unsigned char* profile_data = cmd_data + profile_data_offset;

  // get at point data within profile data
  const uint32_t point_data_offset = 24; // relative to profile_data start
  unsigned char* point_data = profile_data + point_data_offset;

  // get simple fields
  profile.num_points = extract_field<uint16_t>(profile_info, 0);
  profile.data_unit = extract_field<uint16_t>(profile_info, 2);
  profile.x_start = extract_field<int32_t>(profile_info, 4);
  profile.x_increment = extract_field<int32_t>(profile_info, 8);

  // check
  if (profile.points == NULL)
  {
    // TODO: this leaks (but only minor)
    // TODO: this also assumes that 'num_points' can't change during a run
    //       (which is true, as long as we don't allow / expect any config
    //        changes to the sensor during a run).
    profile.points = new profile_point_t[profile.num_points]();
  }

  const uint32_t trigger_count = extract_field<uint32_t>(profile_data, 4);
  const uint32_t encoder_count = extract_field<uint32_t>(profile_data, 8);

  ROS_DEBUG("Unpacking profile %d. Nr of points: %d; data unit: %d; X Start: %d; X Incr: %d",
            trigger_count, profile.num_points, profile.data_unit, profile.x_start,
            profile.x_increment);

  // unpack profile points
  // TODO: hard coded point_data sz (should calculate from pointers)
  const uint32_t point_data_sz = 2000; // bytes
  int res = ljv7_unpack_profile_data(point_data, point_data_sz, profile.num_points, profile.points,
                                     (profile.num_points * sizeof(profile_point_t)));

  if (res != 0)
  {
    ROS_WARN("Error unpacking profile data (err: %d). Ignoring profile %d", res, trigger_count);
    return res;
  }

  // done
  return 0;
}

// assumption: 'msg' is a properly setup (and empty) PointCloud msg
int keyence_profile_to_pc(keyence_profile_t& profile, point_cloud_t::Ptr msg, bool cnv_inf_pts,
                          double scale_factor)
{
  // TODO: get proper timestamp from somewhere
  // pcl header stamps are in microseconds
  msg->header.stamp = ros::Time::now().toNSec() / 1e3;
  msg->width = profile.num_points;

  const double x_start = (profile.x_start * KEYENCE_PDEPTH_UNIT);
  double x = 0., y = 0., z = 0.;

  // add points
  for (unsigned int i = 0; i < profile.num_points; ++i)
  {
    // convert profile points to SI units (meters)
    x = (x_start + (i * (profile.x_increment * KEYENCE_PDEPTH_UNIT))) / 1e6;
    y = 0.;
    z = ((profile.data_unit * KEYENCE_PDEPTH_UNIT) * profile.points[i]) / 1e6;

    // filter out 'infinite distance' points
    // REP-117: http://www.ros.org/reps/rep-0117.html
    //  "out of range detections will be represented by +Inf."
    if (profile.points[i] == KEYENCE_INFINITE_DISTANCE_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI;
    }

    // device returns two different values that are supposed to be interpreted
    // as out-of-range or 'infinite'. This is the second
    if (profile.points[i] == KEYENCE_INFINITE_DISTANCE_VALUE2)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI2;
    }

    x *= scale_factor;
    y *= scale_factor;
    z *= scale_factor; // 'inf * something' still inf

    msg->points.push_back(pcl::PointXYZ(x, y, z));
  }

  return 0;
}
