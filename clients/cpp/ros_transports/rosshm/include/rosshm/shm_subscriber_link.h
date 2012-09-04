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

#ifndef ROSSHM_SHM_SUBSCRIBER_LINK_H
#define ROSSHM_SHM_SUBSCRIBER_LINK_H
#include "ros/common.h"
#include "ros/subscriber_link.h"

#include <boost/signals/connection.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include "rosshm/SharedMemoryBlock.h"

namespace rosshm
{

    /**
     * \brief SubscriberLink handles broadcasting messages to a single subscriber on a single topic
     */
    class ROSCPP_DECL SHMSubscriberLink : public ros::SubscriberLink {
        public:
            SHMSubscriberLink();
            virtual ~SHMSubscriberLink();

            //
            bool initialize(const std::string & segment_name, const std::string & topic_name,
                    const std::string & filter_string);
            bool handleHeader(const ros::Header& header);

            virtual void enqueueMessage(const ros::SerializedMessage& m, bool ser, bool nocopy);
            virtual void drop();
            virtual std::string getTransportType();

        protected:

            boost::interprocess::managed_shared_memory *segment_ ;
            SharedMemoryBlock *blockmgr_;
			bool clientRegistered;

			// This will be modified after the first image is received, so we
			// mark them mutable and publish stays "const"
            shm_handle shm_handle_;
    };
    typedef boost::shared_ptr<SHMSubscriberLink> SHMSubscriberLinkPtr;

} // namespace rosshm

#endif // ROSSHM_SHM_SUBSCRIBER_LINK_H
