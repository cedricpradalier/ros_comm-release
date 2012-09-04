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

#ifndef ROSSHM_SHM_PUBLISHER_LINK_H
#define ROSSHM_SHM_PUBLISHER_LINK_H
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>



#include "ros/common.h"
#include "ros/header.h"
#include "ros/publisher_link.h"
#include "ros/subscription.h"

#include "rosshm/SharedMemoryBlock.h"

typedef boost::shared_ptr<ros::Subscription> SubscriptionPtr;

namespace rosshm
{
    void printHeader(const std::string & prefix, ros::Header & h) ;

    /**
     * \brief Handles a connection to a single publisher on a given topic.  Receives messages from a publisher
     * and hands them off to its parent Subscription
     */
    class ROSCPP_DECL SHMPublisherLink : public ros::PublisherLink
    {
        public:
            SHMPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, 
                    const ros::TransportDescription& transport_description, const ros::TransportFilters& transport_filters);
            virtual ~SHMPublisherLink();

            bool initialize(const std::string & segment_name, 
                    const std::string & topic_name);

            virtual std::string getTransportType();
            virtual void drop();

        protected:
            /**
             * \brief Handles handing off a received message to the subscription, where it will be deserialized and called back
             */
            virtual void handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy);

            boost::thread *rec_thread_;
            boost::interprocess::managed_shared_memory *segment_ ;
            SharedMemoryBlock *blockmgr_;
            shm_handle shm_handle_;
            bool dropping_;

            void receiveThread();
    };
    typedef boost::shared_ptr<SHMPublisherLink> SHMPublisherLinkPtr;

} // namespace rosshm

#endif // ROSSHM_SHM_PUBLISHER_LINK_H



