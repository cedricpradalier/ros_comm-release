/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/* Author: Brian Gerkey */

/*
 * Subscribe to a topic, expecting to get a single message.
 */

#include <string>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
// #include "ros/transport_plugin_manager.h"
// #include "ros_transport_plugin/ros_transport_plugin.h"
#include <test_roscpp/TestArray.h>

int g_argc;
char** g_argv;

class Subscriptions 
{
    public:
        // A node is needed to make a service call
        ros::NodeHandle n;
        std::string transport;
        ros::Subscriber sub;
        bool first_message;
        // Warning, this could be problematic:
        // Filter description are created by dynamic classes and need to be
        // destroyed while the .so are still visible
        ros::TransportHints hints;

        void MsgCallback(const test_roscpp::TestArray::ConstPtr& msg)
        {
            if (first_message) {
                first_message = false;
                // ros::TransportHints hints;
                hints = sub.getTransportHints();
                ros::V_string tname = hints.getTransports();
                ROS_INFO("Used hints: %d transport, first is %s",
                        tname.size(), tname[0].c_str());
                ROS_INFO("Used filter: %s",hints.getFilters().getFilterString().c_str());
            }
            ROS_INFO("received message %d", msg->counter);
        }

        Subscriptions() {}
        void SetUp()
        {
            ros::NodeHandle n;
            // ros::TransportHints hints;

            assert(g_argc >= 2);
            hints.plugin(g_argv[1]).tcp();
            printf("Argv transports %s\n",g_argv[1]);
            printf("Submitted transports: ");
            ros::V_string tname = hints.getTransports();
            for (unsigned int i=0;i<tname.size();i++) {
                printf("%s ",tname[i].c_str());
            }
            printf("\n");
            if (g_argc >= 3) {
                hints.filters(g_argv[2]);
            }
            first_message = true;
            sub = n.subscribe("test_roscpp/pubsub_test", 1, &Subscriptions::MsgCallback, this, hints);
            ROS_INFO("Subscription to \"test_roscpp/pubsub_test\" ready: protocol %s filters %s",
                    g_argv[1], hints.getFilters().getFilterString().c_str());

        }

};



    int
main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber", ros::init_options::AnonymousName);
    // ideally in ros::init
    // ros_transport_plugin::registerAllTransportPlugins();

    g_argc = argc;
    g_argv = argv;
    Subscriptions sub;
    sub.SetUp();
    ros::spin();
}
