#ifndef UDPMulti_TRANSPORT_PLUGIN_H
#define UDPMulti_TRANSPORT_PLUGIN_H

#include <ros/transport_plugin.h>

namespace udpmulti
{
    class UDPMultiTransportPluginInstance : public ros::TransportPluginInstance {
        public:
            UDPMultiTransportPluginInstance(ros::TransportPluginPtr plugin) :
                ros::TransportPluginInstance(plugin) {}
            virtual ~UDPMultiTransportPluginInstance();

            // For Subscription::pendingConnectionDone
            virtual ros::PublisherLinkPtr createPublisherLink(const std::string & topic_name, 
                    const ros::SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response);

            // For TopicManager::requestTopic
            // will eventually create the connections and listener, as appropriate
            virtual bool processTopicRequest(const std::string & topic_name, 
                    const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply);

            // For TransportPluginManager::shutdown
            virtual void shutdown();
        protected:
            bool requestMulticastChannel(const std::string & topic_name, const std::string & filter_name);

            std::string multicast_ip_;
            unsigned int multicast_port_;
    };

    class UDPMultiTransportDescription : public ros::TransportDescription {
        public:
            UDPMultiTransportDescription() :
                ros::TransportDescription("UDPMULTI") {}
            UDPMultiTransportDescription(const ros::TransportDescription & t) :
                ros::TransportDescription(t) {}

            ~UDPMultiTransportDescription();

    };
    typedef boost::shared_ptr<UDPMultiTransportDescription> UDPMultiTransportDescriptionPtr;
                
    class UDPMultiTransportPlugin : public ros::TransportPlugin {
        public:


            UDPMultiTransportPlugin();
            virtual ~UDPMultiTransportPlugin();

            virtual ros::TransportPluginInstancePtr newInstance() {
                ros::TransportPluginInstancePtr res(new UDPMultiTransportPluginInstance(shared_from_this()));
                return res;
            }

            virtual ros::TransportDescription getDefaultTransportDescription() const {
                return UDPMultiTransportDescription();
            }

    };
};

#endif // UDPMulti_TRANSPORT_PLUGIN_H
