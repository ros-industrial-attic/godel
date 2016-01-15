/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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
 *       * Neither the name of the Southwest Research Institute, nor the names
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

#ifndef GODEL_KEYENCE_LJV_DRIVER_TCP_CLIENT_H
#define GODEL_KEYENCE_LJV_DRIVER_TCP_CLIENT_H

#include <simple_message/socket/tcp_socket.h>
#include <simple_message/socket/tcp_client.h>

#include <simple_message/shared_types.h>

#include <simple_message/smpl_msg_connection.h>

namespace industrial
{
namespace tcp_client
{

/**
 * \brief Defines TCP client functions.
 *
 * public industrial::tcp_socket::TcpSocket
 */
class Keyence_TcpClient : public industrial::tcp_client::TcpClient,
                          public industrial::byte_array::ByteArray
{
public:
  // Provides SimpleSerialize access to byte array internals
  // friend class SimpleSerialize;

  /**
   * \brief Constructor
   */
  Keyence_TcpClient() {}

  /**
   * \brief Destructor
   */
  ~Keyence_TcpClient() {}

  bool init(char* buff, int port_num)
  {
    return industrial::tcp_client::TcpClient::init(buff, port_num);
  }
  bool my_sendBytes(industrial::byte_array::ByteArray& buf) { return sendBytes(buf); }
  bool my_receiveBytes(industrial::byte_array::ByteArray& buf,
                       industrial::shared_types::shared_int num_bytes)
  {
    return receiveBytes(buf, num_bytes);
  }
};

} // tcp_client
} // industrial

#endif /* GODEL_KEYENCE_LJV_DRIVER_TCP_CLIENT_H */
