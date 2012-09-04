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

#include "rosshm/shm_publisher_link.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/this_node.h"
#include "ros/file_log.h"
#include "ros/callback_queue.h"
#include "ros/transport_plugin_manager.h"

#include <boost/bind.hpp>

#include <sstream>

using namespace ros;

namespace rosshm
{

    void printHeader(const std::string & prefix, Header & h) {
        printf("%s: Header:\n",prefix.c_str());
        for (M_string::const_iterator it=h.getValues()->begin();
                it != h.getValues()->end(); it++) {
            printf("[%s] -> [%s]\n",it->first.c_str(),it->second.c_str());
        }
        printf("---\n");
    }

    SHMPublisherLink::SHMPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, 
            const ros::TransportDescription& transport_description, const ros::TransportFilters& transport_filters)
        : PublisherLink(parent, xmlrpc_uri, transport_description, transport_filters)
    {
        dropping_ = false;
        rec_thread_ = NULL;
        segment_ = NULL;
    }

    SHMPublisherLink::~SHMPublisherLink()
    {
        drop();
        ROS_DEBUG("SHMPublisherLink destructed");
    }

    void SHMPublisherLink::receiveThread() {
        ROS_DEBUG("Receive thread running");
        while (!dropping_) {
            ROSCPP_LOG_DEBUG("Waiting for data");
            SerializedMessage sm;
            if (blockmgr_->wait_data(*segment_, shm_handle_, sm)
                    && !dropping_) {
                ROSCPP_LOG_DEBUG("Prepared SerMsg");
                handleMessage(sm, true, false);
            }
        }
        ROS_DEBUG("Unregistering client");
    }

    bool SHMPublisherLink::initialize(const std::string & segment_name, const std::string & topic_name)
    {
        if (!segment_) {
            try {
                segment_ = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only,segment_name.c_str());
                ROS_DEBUG("Connected to segment");
            } catch (boost::interprocess::bad_alloc e) {
                segment_ = NULL;
                ROS_ERROR("Failed to connect to shared memory segment");
                return false;
            }
            blockmgr_ = (segment_->find<SharedMemoryBlock>("Manager")).first;
            if (!blockmgr_) {
                delete segment_;
                segment_ = NULL;
                ROS_ERROR("Cannot find Manager block in shared memory segment");
                return false;
            }
            ROS_DEBUG("Got block mgr %p",blockmgr_);
            std::string record_name = topic_name+"["+transport_filters_.getFilterString()+"]";
            shm_handle_ = blockmgr_->findHandle(*segment_,record_name.c_str());
            if (shm_handle_.is_valid()) {
                ROS_DEBUG("Got shm handle %p",shm_handle_.ptr);
                dropping_ = false;
                rec_thread_ = new  boost::thread(&SHMPublisherLink::receiveThread,this);
            } else {
                delete segment_;
                segment_ = NULL;
                ROS_ERROR("Cannot find memory block for %s", record_name.c_str());
            }
        }

        return true;
    }

    void SHMPublisherLink::drop()
    {
        ROS_DEBUG("Shutting down SharedmemSubscriber");
        if (dropping_) return;
        dropping_ = true;
        if (rec_thread_) {
            // We probably need to do something to clean up the
            // cancelled thread here
            rec_thread_->interrupt();
            rec_thread_->join();
            delete rec_thread_;
        }
        rec_thread_ = NULL;
            
        delete segment_;
        segment_ = NULL;

        if (SubscriptionPtr parent = parent_.lock())
        {
            parent->removePublisherLink(shared_from_this());
        }
    }

    void SHMPublisherLink::handleMessage(const SerializedMessage& m, bool ser, bool nocopy)
    {
        SerializedMessage filtered;
        if (!TransportPluginManager::unapplyFilters(transport_filters_, m, filtered)) {
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

    std::string SHMPublisherLink::getTransportType()
    {
        return std::string("SHMROS");
    }

} // namespace rosshm

