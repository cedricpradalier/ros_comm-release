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

#include "rosshm/shm_subscriber_link.h"
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

#include <boost/interprocess/managed_shared_memory.hpp>
#include "rosshm/SharedMemoryBlock.h"

using namespace boost::interprocess;
using namespace ros;

namespace rosshm
{

    SHMSubscriberLink::SHMSubscriberLink()
    {
        clientRegistered = false;
        segment_ = NULL;
    }

    SHMSubscriberLink::~SHMSubscriberLink()
    {
        drop();
    }

    bool SHMSubscriberLink::initialize(const std::string & segment_name, 
            const std::string & topic_name, const std::string & filter_string)
    {
        if (!clientRegistered) {
            clientRegistered = true;
            try {
                segment_ = new managed_shared_memory(open_only,segment_name.c_str());
                ROS_INFO("Got segment %p",segment_);
            } catch (boost::interprocess::bad_alloc e) {
                segment_ = NULL;
                ROS_ERROR("Could not open shared memory segment");
                return false;
            }
            blockmgr_ = (segment_->find<SharedMemoryBlock>("Manager")).first;
            if (!blockmgr_) {
                delete segment_;
                segment_ = NULL;
                ROS_ERROR("Cannot find Manager block in shared memory segment");
                return false;
            }
            ROS_INFO("Got manager %p",blockmgr_);
            try {
                std::string record_name = topic_name+"["+filter_string+"]";
                shm_handle_ = blockmgr_->allocateBlock(*segment_,record_name.c_str(),16);
                ROS_INFO("Got shm handle");
            } catch (boost::interprocess::bad_alloc e) {
                delete segment_;
                segment_ = NULL;
                ROS_ERROR("Could not open shared memory segment");
                return false;
            }
        }
        return true;
    }

    bool SHMSubscriberLink::handleHeader(const ros::Header& header)
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
                topic + std::string("] from [SHM] [" + client_callerid +"].");

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

    void SHMSubscriberLink::enqueueMessage(const ros::SerializedMessage& m, bool ser, bool nocopy)
    {
        if (!ser)
        {
            return;
        }

        SerializedMessage filtered;
        if (!applyFilters(m, filtered)) {
            return ;
        }

        stats_.messages_sent_++;
        stats_.bytes_sent_ += filtered.num_bytes;
        stats_.message_data_sent_ += filtered.num_bytes;

        if (!shm_handle_.is_valid()) {
            ROS_DEBUG("Ignoring publish request on an invalid handle");
            return;
        }
        blockmgr_->reallocateBlock(*segment_,shm_handle_,filtered.num_bytes);
        if (shm_handle_.is_valid()) { // check again, in case reallocate failed
            // Remove the first 4 bytes (message length). We don't need them
            // for this transport
            blockmgr_->serialize(*segment_,shm_handle_,filtered.buf.get()+4,filtered.num_bytes-4);
        }


    }

    std::string SHMSubscriberLink::getTransportType()
    {
        return "SHMROS";
    }

    void SHMSubscriberLink::drop()
    {
        // This just disconnect from the segment, any subscriber can still
        // finish reading it.
        if (segment_) {
            delete segment_;
            segment_ = NULL;
        }
        clientRegistered = false;
    }

} // namespace rosshm
