/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/platform.h>  // platform dependendant requirements

#include "udpmulti/udpmulti_publisher_link.h"
#include "udpmulti/udpmulti_subscriber_link.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/this_node.h"
#include "ros/file_log.h"
#include "ros/callback_queue.h"
#include "ros/transport_plugin_manager.h"
#include "ros/ros.h"

#include <boost/bind.hpp>

#include <sstream>

using namespace ros;

namespace udpmulti
{

    void printHeader(const std::string & prefix, Header & h) {
        printf("%s: Header:\n",prefix.c_str());
        for (M_string::const_iterator it=h.getValues()->begin();
                it != h.getValues()->end(); it++) {
            printf("[%s] -> [%s]\n",it->first.c_str(),it->second.c_str());
        }
        printf("---\n");
    }

    UDPMultiPublisherLink::UDPMultiPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, 
            const ros::TransportDescription& transport_description, const ros::TransportFilters& transport_filters)
        : PublisherLink(parent, xmlrpc_uri, transport_description, transport_filters), io_service_(), 
        socket_(io_service_), rec_thread_(NULL) 
    {
        dropping_ = false;
    }

    UDPMultiPublisherLink::~UDPMultiPublisherLink()
    {
        drop();
        ROS_DEBUG("UDPMultiPublisherLink destructed");
    }

    void UDPMultiPublisherLink::receiveThread() {
        uint8_t data[UDPMultiSubscriberLink::MAX_UDP_PACKET_SIZE];
        while (!dropping_) {
            std::size_t rec;
            // ROS_INFO("Waiting for datagram");
            rec = socket_.receive_from(
                    boost::asio::buffer(data, UDPMultiSubscriberLink::MAX_UDP_PACKET_SIZE), endpoint_);
            // ROS_INFO("Received datagram: %d bytes",rec);
            if (!rec || dropping_) continue;

            uint32_t message_length = *((uint32_t*)data); // Dangerous!!! but done like that in roscpp
            boost::shared_array<uint8_t> buf(new uint8_t[message_length]);
            memcpy(buf.get(),data+4,message_length);
            ros::SerializedMessage msg(buf,message_length);
            handleMessage(msg, true, false);

        }
        ROS_DEBUG("Unregistering client");
    }

    bool UDPMultiPublisherLink::initialize(const std::string & multicast_ip, unsigned int multicast_port)
    {				
        // Process the header, and get the listener started.
        if (listening_interface_.empty()) {
            // First message, launch the initialisation
            ros::NodeHandle nh("~");
            nh.param<std::string>("listening_interface",listening_interface_,"0.0.0.0");
            multicast_address_ = multicast_ip;
            port_ = multicast_port;
            ROS_INFO("Listening on %s, address '%s:%d'",listening_interface_.c_str(),multicast_address_.c_str(),port_);

            // Create the socket so that multiple may be bound to the same address.
            boost::asio::ip::udp::endpoint listen_endpoint(
                    boost::asio::ip::address::from_string(listening_interface_), port_);
            socket_.open(listen_endpoint.protocol());
            socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
            socket_.bind(listen_endpoint);

            // Join the multicast group.
            socket_.set_option(boost::asio::ip::multicast::join_group(
                        boost::asio::ip::address::from_string(multicast_address_)));
            // Allow loopback
            socket_.set_option(boost::asio::ip::multicast::enable_loopback(true));

            dropping_ = false;
            rec_thread_ = new  boost::thread(&UDPMultiPublisherLink::receiveThread,this);
        }
        return true;
    }

    void UDPMultiPublisherLink::drop()
    {
        ROS_INFO("Shutting down UDPMultiSubscriber");
        if (dropping_) return;
        dropping_ = true;
        io_service_.stop();
        if (rec_thread_) {
            try {
                socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_receive);
            } catch (boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> > e) {
                // ignore
            }
            rec_thread_->interrupt();
            rec_thread_->join();
            delete rec_thread_;
        }
        rec_thread_ = NULL;

        if (SubscriptionPtr parent = parent_.lock())
        {
            parent->removePublisherLink(shared_from_this());
        }
    }

    void UDPMultiPublisherLink::handleMessage(const SerializedMessage& m, bool ser, bool nocopy)
    {
        SerializedMessage filtered;
        if (!ros::TransportPluginManager::unapplyFilters(transport_filters_, m, filtered)) {
            return;
        }

        stats_.bytes_received_ += filtered.num_bytes;
        stats_.messages_received_++;

        SubscriptionPtr parent = parent_.lock();

        if (parent)
        {
            stats_.drops_ += parent->handleMessage(filtered, ser, nocopy, header_.getValues(), shared_from_this());
        }
    }

    std::string UDPMultiPublisherLink::getTransportType()
    {
        return std::string("MULTICASTROS");
    }

} // namespace udpmulti

