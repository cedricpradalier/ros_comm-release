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
 * Publish a message N times, back to back
 */

#include <string>
#include <cstdio>
#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
// #include "ros/transport_plugin_manager.h"
// #include "ros_transport_plugin/ros_transport_plugin.h"
#include <test_roscpp/TestArray.h>

#define USAGE "USAGE: publish_with_hints <min_size> <max_size>"

    int
main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_with_hints", ros::init_options::AnonymousName);
    // ideally in ros::init
    // ros_transport_plugin::registerAllTransportPlugins();
    ros::NodeHandle n;

    if(argc != 3)
    {
        puts(USAGE);
        exit(-1);
    }

    int min_size = atoi(argv[1]);
    int max_size = atoi(argv[2]);

    unsigned int i = 0;
    ros::Publisher pub_ = n.advertise<test_roscpp::TestArray>("test_roscpp/pubsub_test", 1);
    ros::Rate rate(2);
    test_roscpp::TestArray msg;
    while (ros::ok()) {
        ros::spinOnce();
        msg.counter = i++;
        int j = min_size + (int) ((max_size - min_size) * (rand() / (RAND_MAX + 1.0)));
        msg.float_arr.resize(j);
        pub_.publish(msg);
        ROS_INFO("published message %d (%d bytes)",
                msg.counter, ros::serialization::Serializer<test_roscpp::TestArray>::serializedLength(msg));
        rate.sleep();
    }

    return 0;
}
