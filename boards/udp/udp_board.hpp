/*
 * Copyright (c) 2017 Daniel Koch, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file udp_board.h
 * \author Daniel Koch
 */

#ifndef ROSFLIGHT_FIRMWARE_UDP_BOARD_H
#define ROSFLIGHT_FIRMWARE_UDP_BOARD_H

#include <list>
#include <string>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "board.h"
#include "mavlink/mavlink.h"

namespace rosflight_firmware
{
class UDPBoard : public Board
{
public:
  explicit UDPBoard(std::string bind_host = "localhost", uint16_t bind_port = 14525,
                    std::string remote_host = "localhost", uint16_t remote_port = 14520);
  ~UDPBoard();

  void serial_init(uint32_t baud_rate, uint32_t dev) override;
  void serial_write(const uint8_t * src, size_t len) override;
  uint16_t serial_bytes_available() override;
  uint8_t serial_read() override;
  void serial_flush() override;

  void set_ports(std::string bind_host, uint16_t bind_port, std::string remote_host,
                 uint16_t remote_port);

private:
  struct Buffer
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {0};
    size_t len;
    size_t pos;

    Buffer() : len(0), pos(0) {}

    Buffer(const uint8_t * src, size_t length) : len(length), pos(0)
    {
      assert(length <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here
      memcpy(data, src, length);
    }

    const uint8_t * dpos() const { return data + pos; }
    size_t nbytes() const { return len - pos; }
    void add_byte(uint8_t byte) { data[len++] = byte; }
    uint8_t consume_byte() { return data[pos++]; }
    bool empty() const { return pos >= len; }
    bool full() const { return len >= MAVLINK_MAX_PACKET_LEN; }
  };

  typedef boost::lock_guard<boost::recursive_mutex> MutexLock;

  void async_read();
  void async_read_end(const boost::system::error_code & error, size_t bytes_transferred);

  void async_write(bool check_write_state);
  void async_write_end(const boost::system::error_code & error, size_t bytes_transferred);

  std::string bind_host_;
  uint16_t bind_port_;

  std::string remote_host_;
  uint16_t remote_port_;

  boost::thread io_thread_;
  boost::recursive_mutex write_mutex_;
  boost::recursive_mutex read_mutex_;

  boost::asio::io_service io_service_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint bind_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;

  uint8_t read_buffer_[MAVLINK_MAX_PACKET_LEN] = {0};
  std::list<Buffer *> read_queue_;

  std::list<Buffer *> write_queue_;
  bool write_in_progress_;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_UDP_BOARD_H
