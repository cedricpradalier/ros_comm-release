#ifndef TCP_TRANSPORT_PLUGIN_H
#define TCP_TRANSPORT_PLUGIN_H

#include <ros/transport_description.h>
#include <ros/transport_plugin.h>

namespace ros 
{
    class TCPTransportPluginInstance : public TransportPluginInstance {
        public:
            TCPTransportPluginInstance(TransportPluginPtr plugin) :
                TransportPluginInstance(plugin) {}
            virtual ~TCPTransportPluginInstance();

            // For Subscription::pendingConnectionDone
            virtual PublisherLinkPtr createPublisherLink(const std::string & topic_name, 
                    const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response);

            // For TopicManager::requestTopic
            // will eventually create the connections and listener, as appropriate
            virtual bool processTopicRequest(const std::string & topic_name, 
                    const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply);

            // For TransportPluginManager::shutdown
            virtual void shutdown();

    };

    class TCPTransportPlugin : public TransportPlugin {
        public:


            TCPTransportPlugin();
            virtual ~TCPTransportPlugin();

            virtual TransportPluginInstancePtr newInstance() {
                TransportPluginInstancePtr res(new TCPTransportPluginInstance(shared_from_this()));
                return res;
            }

            virtual TransportDescription getDefaultTransportDescription() const {
                return TCPTransportDescription();
            }
    };
};

#endif // TCP_TRANSPORT_PLUGIN_H
