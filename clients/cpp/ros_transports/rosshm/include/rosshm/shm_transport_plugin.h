#ifndef SHM_TRANSPORT_PLUGIN_H
#define SHM_TRANSPORT_PLUGIN_H

#include <ros/transport_plugin.h>

namespace rosshm
{
    class SHMTransportPluginInstance : public ros::TransportPluginInstance {
        public:
            SHMTransportPluginInstance(ros::TransportPluginPtr plugin) :
                ros::TransportPluginInstance(plugin) {}
            virtual ~SHMTransportPluginInstance();

            virtual XmlRpc::XmlRpcValue prepareConnectionNegotiation( const ros::SubscriptionPtr & subscription, 
                    const ros::TransportDescription& transport_description, const ros::TransportFilters & transport_filters) ;

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
            bool requestSegmentName();

            std::string segment_name_;
    };

    class SHMTransportDescription : public ros::TransportDescription {
        public:
            SHMTransportDescription() :
                ros::TransportDescription("SHM") {}
            SHMTransportDescription(const ros::TransportDescription & t) :
                ros::TransportDescription(t) {}

            ~SHMTransportDescription() {}
    };
    typedef boost::shared_ptr<SHMTransportDescription> SHMTransportDescriptionPtr;
                

    class SHMTransportPlugin : public ros::TransportPlugin {
        public:


            SHMTransportPlugin();
            virtual ~SHMTransportPlugin();

            virtual ros::TransportPluginInstancePtr newInstance() {
                ros::TransportPluginInstancePtr res(new SHMTransportPluginInstance(shared_from_this()));
                return res;
            }

            virtual ros::TransportDescription getDefaultTransportDescription() const {
                return SHMTransportDescription();
            }
    };
};

#endif // SHM_TRANSPORT_PLUGIN_H
