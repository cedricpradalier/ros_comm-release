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

#ifndef UDP_MULTICAST_PUBLISHER_LINK_H
#define UDP_MULTICAST_PUBLISHER_LINK_H


#include "boost/asio.hpp"
#include "ros/common.h"
#include "ros/header.h"
#include "ros/publisher_link.h"
#include "ros/subscription.h"

typedef boost::shared_ptr<ros::Subscription> SubscriptionPtr;

namespace udpmulti
{
    /**
     * \brief Handles a connection to a single publisher on a given topic.  Receives messages from a publisher
     * and hands them off to its parent Subscription
     */
    class ROSCPP_DECL UDPMultiPublisherLink : public ros::PublisherLink
    {
        public:
            UDPMultiPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, 
                    const ros::TransportDescription& transport_description, const ros::TransportFilters& transport_filters);
            virtual ~UDPMultiPublisherLink();

            bool initialize(const std::string & multicast_ip, unsigned int multicast_port);

            virtual std::string getTransportType();
            virtual void drop();

        protected:
            /**
             * \brief Handles handing off a received message to the subscription, where it will be deserialized and called back
             */
            virtual void handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy);

			uint32_t port_;
			std::string multicast_address_;
			std::string listening_interface_;
            boost::asio::io_service io_service_;
            boost::asio::ip::udp::endpoint endpoint_;
            boost::asio::ip::udp::socket socket_;
            boost::thread *rec_thread_;
            bool dropping_;

            void receiveThread();
    };
    typedef boost::shared_ptr<UDPMultiPublisherLink> UDPMultiPublisherLinkPtr;

} // namespace udpmulti

#endif // UDP_MULTICAST_PUBLISHER_LINK_H



