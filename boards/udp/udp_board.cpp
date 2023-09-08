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
 * \file udp_board.cpp
 * \author Daniel Koch
 */

#include "udp_board.hpp"

#include <iostream>

using boost::asio::ip::udp;

namespace rosflight_firmware
{
UDPBoard::UDPBoard(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port) :
  bind_host_(std::move(bind_host)),
  bind_port_(bind_port),
  remote_host_(std::move(remote_host)),
  remote_port_(remote_port),
  io_service_(),
  socket_(io_service_),
  write_in_progress_(false)
{
}

UDPBoard::~UDPBoard()
{
  MutexLock read_lock(read_mutex_);
  MutexLock write_lock(write_mutex_);

  io_service_.stop();
  socket_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void UDPBoard::set_ports(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port)
{
  bind_host_ = std::move(bind_host);
  bind_port_ = bind_port;
  remote_host_ = std::move(remote_host);
  remote_port_ = remote_port;
}

void UDPBoard::serial_init(uint32_t baud_rate, uint32_t dev)
{
  // can throw an uncaught boost::system::system_error exception
  (void)dev;

  udp::resolver resolver(io_service_);

  bind_endpoint_ = *resolver.resolve({udp::v4(), bind_host_, ""});
  bind_endpoint_.port(bind_port_);

  remote_endpoint_ = *resolver.resolve({udp::v4(), remote_host_, ""});
  remote_endpoint_.port(remote_port_);

  socket_.open(udp::v4());
  socket_.bind(bind_endpoint_);

  socket_.set_option(udp::socket::reuse_address(true));
  socket_.set_option(udp::socket::send_buffer_size(1000 * MAVLINK_MAX_PACKET_LEN));
  socket_.set_option(udp::socket::receive_buffer_size(1000 * MAVLINK_MAX_PACKET_LEN));

  async_read();
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

void UDPBoard::serial_flush() {}

void UDPBoard::serial_write(const uint8_t* src, size_t len)
{
  auto buffer = new Buffer(src, len);

  {
    MutexLock lock(write_mutex_);
    write_queue_.push_back(buffer);
  }

  async_write(true);
}

uint16_t UDPBoard::serial_bytes_available()
{
  MutexLock lock(read_mutex_);
  return !read_queue_.empty(); //! \todo This should return a number, not a bool
}

uint8_t UDPBoard::serial_read()
{
  MutexLock lock(read_mutex_);

  if (read_queue_.empty())
  {
    return 0;
  }

  Buffer* buffer = read_queue_.front();
  uint8_t byte = buffer->consume_byte();

  if (buffer->empty())
  {
    read_queue_.pop_front();
    delete buffer;
  }
  return byte;
}

void UDPBoard::async_read()
{
  if (!socket_.is_open())
  {
    return;
  }

  MutexLock lock(read_mutex_);
  socket_.async_receive_from(boost::asio::buffer(read_buffer_, MAVLINK_MAX_PACKET_LEN), remote_endpoint_,
                             boost::bind(&UDPBoard::async_read_end, this, boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

void UDPBoard::async_read_end(const boost::system::error_code& error, size_t bytes_transferred)
{
  if (!error)
  {
    MutexLock lock(read_mutex_);
    read_queue_.push_back(new Buffer(read_buffer_, bytes_transferred));
  }
  async_read();
}

void UDPBoard::async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
  {
    return;
  }

  MutexLock lock(write_mutex_);
  if (write_queue_.empty())
  {
    return;
  }

  write_in_progress_ = true;
  Buffer* buffer = write_queue_.front();
  socket_.async_send_to(boost::asio::buffer(buffer->dpos(), buffer->nbytes()), remote_endpoint_,
                        boost::bind(&UDPBoard::async_write_end, this, boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred));
}

void UDPBoard::async_write_end(const boost::system::error_code& error, size_t bytes_transferred)
{
  if (!error)
  {
    MutexLock lock(write_mutex_);

    if (write_queue_.empty())
    {
      write_in_progress_ = false;
      return;
    }

    Buffer* buffer = write_queue_.front();
    buffer->pos += bytes_transferred;
    if (buffer->empty())
    {
      write_queue_.pop_front();
      delete buffer;
    }

    if (write_queue_.empty())
    {
      write_in_progress_ = false;
    }
    else
    {
      async_write(false);
    }
  }
}

} // namespace rosflight_firmware
