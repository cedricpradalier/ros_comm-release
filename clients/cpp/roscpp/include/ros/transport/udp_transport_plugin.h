#ifndef UDP_TRANSPORT_PLUGIN_H
#define UDP_TRANSPORT_PLUGIN_H

#include <boost/enable_shared_from_this.hpp>
#include <ros/publisher_link.h>
#include <ros/transport_plugin.h>
#include <ros/transport/transport_udp.h>
#include "XmlRpc.h"

namespace ros 
{
    class UDPTransportPluginInstance : public TransportPluginInstance {
        public:

            UDPTransportPluginInstance(TransportPluginPtr plugin) 
                : TransportPluginInstance(plugin) { }

            virtual ~UDPTransportPluginInstance();

            // For Subscription::negotiateConnection, common to all modules so
            virtual XmlRpc::XmlRpcValue prepareConnectionNegotiation(
                    const SubscriptionPtr & subscription, 
                    const TransportDescription& transport_description,
                    const TransportFilters & transport_filters);

            // For Subscription::pendingConnectionDone
            virtual PublisherLinkPtr createPublisherLink(const std::string & topic_name, 
                    const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response);

            // For TopicManager::requestTopic
            // will eventually create the connections and listener, as appropriate
            virtual bool processTopicRequest(const std::string & topic_name, 
                    const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply);

            // For TransportPluginManager::shutdown
            virtual void shutdown();

        protected:
            TransportUDPPtr udp_transport_;
    };

    class UDPTransportPlugin : public TransportPlugin {
        public:
            UDPTransportPlugin(); 
            virtual ~UDPTransportPlugin();

            virtual TransportPluginInstancePtr newInstance() {
                TransportPluginInstancePtr res(new UDPTransportPluginInstance(shared_from_this()));
                return res;
            }

            virtual TransportDescription getDefaultTransportDescription() const {
                return UDPTransportDescription();
            }
    };


};

#endif // UDP_TRANSPORT_PLUGIN_H
