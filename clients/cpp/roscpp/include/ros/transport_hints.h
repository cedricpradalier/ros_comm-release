/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_TRANSPORT_HINTS_H
#define ROSCPP_TRANSPORT_HINTS_H

#include "common.h"
#include "ros/forwards.h"
#include "ros/transport_description.h"

#include <boost/lexical_cast.hpp>

namespace ros
{
    class ROSCPP_DECL TransportFilters
    {
        public:
            /**
             * \brief Get the list of filter names
             */
            V_string getFilterNames() const ;

            /**
             * \brief Get the list of filters
             */
            const std::vector<FilterDescription> & getFilterDescriptions() const {
                return filters_;
            }

            /**
             * \brief Get the string representation of the filter pipeline
             */
            std::string getFilterString() const ;

            /**
             * \brief Create the filter pipeline from its string description
             */
            bool createFiltersFromString(const std::string & filters);

            /**
             * \brief Add a filter to the stack
             * */
            void push_back(const FilterDescription & f) {
                filters_.push_back(f);
            }

            /**
             * \brief Remove all filters from the list
             * */
            void clear() {
                filters_.clear();
            }

        protected:
            std::vector<FilterDescription> filters_;
    };

/**
 * \brief Provides a way of specifying network transport hints to ros::NodeHandle::subscribe() and
 * someday ros::NodeHandle::advertise()
 *
 * Uses the named parameter idiom, allowing you to do things like:
\verbatim
ros::TransportHints()
        .unreliable()
        .maxDatagramSize(1000)
        .tcpNoDelay();
\endverbatim
 *
 * Hints for the transport type are used in the order they are specified, i.e. TransportHints().unreliable().reliable()
 * specifies that you would prefer an unreliable transport, followed by a reliable one.
 */
class ROSCPP_DECL TransportHints
{
public:
  /**
   * \brief Specifies a reliable transport.  Currently this means TCP
   */
  TransportHints& reliable()
  {
    tcp();

    return *this;
  }

  /**
   * \brief Explicitly specifies the TCP transport
   */
  TransportHints& tcp()
  {
                transports_.push_back(TCPTransportDescription());
    return *this;
  }

  /**
   * \brief If a TCP transport is used, specifies whether or not to use TCP_NODELAY to provide
   * a potentially lower-latency connection.
   *
   * \param nodelay [optional] Whether or not to use TCP_NODELAY.  Defaults to true.
   */
  TransportHints& tcpNoDelay(bool nodelay = true)
  {
                for (unsigned int i=0;i<transports_.size();i++) {
                    if (transports_[i].getName() == "TCP") {
                        TCPTransportDescription tcp(transports_[i]);
                        tcp.setNoDelay(nodelay);
                    }
                }
    return *this;
  }

  /**
   * \brief Returns whether or not this TransportHints has specified TCP_NODELAY
   */
            bool getTCPNoDelay() const
    {
                for (unsigned int i=0;i<transports_.size();i++) {
                    if (transports_[i].getName() == "TCP") {
                        TCPTransportDescription tcp(transports_[i]);
                        return tcp.getNoDelay();
    }
    }
  }

  /**
   * \brief If a UDP transport is used, specifies the maximum datagram size.
   *
   * \param size The size, in bytes
   */
  TransportHints& maxDatagramSize(int size)
  {
                for (unsigned int i=0;i<transports_.size();i++) {
                    if (transports_[i].getName() == "UDP") {
                        UDPTransportDescription udp(transports_[i]);
                        udp.setMaxDatagramSize(size);
                    }
                }
    return *this;
  }

  /**
   * \brief Returns the maximum datagram size specified on this TransportHints, or 0 if
   * no size was specified.
   */
            int getMaxDatagramSize() const
  {
                for (unsigned int i=0;i<transports_.size();i++) {
                    if (transports_[i].getName() == "UDP") {
                        UDPTransportDescription udp(transports_[i]);
                        return udp.getMaxDatagramSize();
    }
                }
                return 0;
  }

  /**
   * \brief Specifies an unreliable transport.  Currently this means UDP.
   */
  TransportHints& unreliable()
  {
    udp();

    return *this;
  }

  /**
   * \brief Explicitly specifies a UDP transport.
   */
  TransportHints& udp()
  {
                transports_.push_back(UDPTransportDescription());
    return *this;
  }

  /**
   * \brief Returns a vector of transports, ordered by preference
   */
            V_string getTransports() const ;

            /**
             * \brief Returns a vector of transport names, ordered by preference
             */
            const std::vector<TransportDescription>& getTransportDescriptions() const { return transports_; }

  /**
   * \brief Returns the map of options created by other methods inside TransportHints
             * Kept for backward compatibility
             */
            M_string getOptions() const ;

            /**
             * \brief Raw tool to store data for plugins
             *
             */
            TransportHints& plugin(const std::string & name);

            /**
             * \brief Raw tool to add a transport description
             *
             */
            TransportHints& transport(TransportDescription & transport) {
                transports_.push_back(transport);
                return *this;
            }

            /**
             * \brief Raw tool to add the filter description
             *
             */
            TransportHints& filter(const FilterDescription & filter) {
                filters_.push_back(filter);
                return *this;
            }

            /**
             * \brief Build a filter pipeline from a string description
             * Typically "filter1|filter2|filter3"
             * Or even: "filter1|filter2:param1=value:param2=value|filter3"
             *
   */
            TransportHints& filters(const std::string & description) {
                filters_.createFiltersFromString(description);
                return *this;
            }


            const TransportFilters & getFilters() const {
                return filters_;
            }

            void setFilters(const TransportFilters & filters) {
                filters_ = filters;
            }

            /**
             * \brief Remove all transports from the list
             * */
            void clear() {
                transports_.clear();
            }

            void only(const TransportDescription & transport) {
                transports_.clear();
                transports_.push_back(transport);
            }


        protected:
            std::vector<TransportDescription> transports_;
            TransportFilters filters_;
};

}

#endif
