/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "udpmulti/udpmulti_subscriber_link.h"
#include "ros/publication.h"
#include "ros/header.h"
#include "ros/this_node.h"
#include "ros/topic_manager.h"
#include "ros/file_log.h"

#include <boost/bind.hpp>

#include <cstdlib> 
#include <cstddef>
#include <cassert>
#include <utility>

#include <ros/ros.h>

using namespace ros;

namespace udpmulti
{

    UDPMultiSubscriberLink::UDPMultiSubscriberLink() :
        io_service_()
    {
        endpoint_ = NULL;
        socket_ = NULL;
    }

    UDPMultiSubscriberLink::~UDPMultiSubscriberLink()
    {
        drop();
    }

    bool UDPMultiSubscriberLink::initialize(const std::string & multicast_ip, unsigned int multicast_port)
    {

        multicast_address_ = multicast_ip;
        port_ = multicast_port;

        endpoint_ = new boost::asio::ip::udp::endpoint(
                boost::asio::ip::address::from_string(multicast_address_),port_);
        socket_= new boost::asio::ip::udp::socket(io_service_,endpoint_-> protocol());
        ROSCPP_LOG_DEBUG("Endpoint %p / Socket %p\n",endpoint_,socket_);

        return true;
    }

    bool UDPMultiSubscriberLink::handleHeader(const ros::Header& header)
    {
        std::string topic;
        if (!header.getValue("topic", topic))
        {
            std::string msg("Header from subscriber did not have the required element: topic");

            ROS_ERROR("%s", msg.c_str());

            return false;
        }

        // This will get validated by validateHeader below
        std::string client_callerid;
        header.getValue("callerid", client_callerid);
        PublicationPtr pt = TopicManager::instance()->lookupPublication(topic);
        if (!pt)
        {
            std::string msg = std::string("received a connection for a nonexistent topic [") +
                topic + std::string("] from [UDPMulti] [" + client_callerid +"].");

            ROSCPP_LOG_DEBUG("%s", msg.c_str());

            return false;
        }

        std::string error_msg;
        if (!pt->validateHeader(header, error_msg))
        {
            ROSCPP_LOG_DEBUG("%s", error_msg.c_str());

            return false;
        }
        std::string filters;
        if (header.getValue("filters", filters)) {
            setMessageFilters(filters);
        }

        return true;
    }

    void UDPMultiSubscriberLink::enqueueMessage(const ros::SerializedMessage& m, bool ser, bool nocopy)
    {
        if (!ser)
        {
            return;
        }
        assert(socket_);
        assert(endpoint_);

        SerializedMessage filtered;
        if (!applyFilters(m, filtered)) { 
            return;
        }


        stats_.messages_sent_++;
        stats_.bytes_sent_ += filtered.num_bytes;
        stats_.message_data_sent_ += filtered.num_bytes;

        if (filtered.num_bytes > UDPMultiSubscriberLink::MAX_UDP_PACKET_SIZE) {
            ROS_ERROR("This type of message is too big (%d bytes) for UDP (max %d bytes)",
                    filtered.num_bytes, UDPMultiSubscriberLink::MAX_UDP_PACKET_SIZE);
            return;
        }

        // ROS_INFO("Sending datagram");
        socket_-> send_to(boost::asio::buffer(filtered.buf.get(), filtered.num_bytes),*endpoint_);
        io_service_.poll();
    }

    std::string UDPMultiSubscriberLink::getTransportType()
    {
        return "MULTICASTROS";
    }

    void UDPMultiSubscriberLink::drop()
    {
        if (socket_) delete socket_;
        if (endpoint_) delete endpoint_;
        endpoint_ = NULL;
        socket_ = NULL;
    }

} // namespace udpmulti
