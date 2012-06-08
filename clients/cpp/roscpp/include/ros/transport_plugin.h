#ifndef TRANSPORT_PLUGIN_H
#define TRANSPORT_PLUGIN_H

#include <boost/enable_shared_from_this.hpp>
#include <ros/transport_description.h>
#include <ros/subscription.h>
#include <ros/publisher_link.h>
#include <ros/forwards.h>
#include "XmlRpc.h"

namespace ros 
{
    class TransportPlugin;
    typedef boost::shared_ptr<TransportPlugin> TransportPluginPtr;

    class TransportFilterPlugin;
    typedef boost::shared_ptr<TransportFilterPlugin> TransportFilterPluginPtr;

    class TransportPluginInstance;
    typedef boost::shared_ptr<TransportPluginInstance> TransportPluginInstancePtr;

    class TransportPlugin : public boost::enable_shared_from_this<TransportPlugin> {
        protected:
            std::string name_;
            std::string proto_name_;
        public:

            TransportPlugin(const std::string & name, const std::string & proto_name) 
                : name_(name), proto_name_(proto_name) {}
            virtual ~TransportPlugin();

            const std::string & getName() const {return name_;}
            const std::string & getProtocolName() const {return proto_name_;}
            
            // Will be useful when loading classes from plugin
            virtual TransportPluginInstancePtr newInstance() = 0;

            virtual TransportDescription getDefaultTransportDescription() const = 0;
    };

    class TransportFilterPlugin : public boost::enable_shared_from_this<TransportFilterPlugin> {
        protected:
            std::string name_;
        public:
            TransportFilterPlugin(const std::string & name) : name_(name) {}
            virtual ~TransportFilterPlugin();

            const std::string & getName() const {return name_;}

            virtual bool apply(const FilterDescription & filter_description, bool forward,  
                    const SerializedMessage & src, SerializedMessage & dest) = 0;

            virtual FilterDescription getDefaultFilterDescription() const = 0;
    };


    class TransportPluginInstance : public boost::enable_shared_from_this<TransportPluginInstance> {
        protected:
            TransportPluginPtr plugin_;
            TransportFilters transport_filters_;
            TransportDescription transport_description_;
        public:
            TransportPluginInstance(TransportPluginPtr plugin) : plugin_(plugin), transport_description_(plugin->getName()) {}
            virtual ~TransportPluginInstance();

            const std::string & getName() const {return plugin_->getName();}
            const std::string & getProtocolName() const {return plugin_->getProtocolName();}
            const TransportFilters & getFilters() const {return transport_filters_;}
            const TransportDescription & getDescription() const {return transport_description_;}
            

            // For Subscription::negotiateConnection, common to all modules so
            // far. TransportHints are used only for the filters
            // Parameters should be found in the transport_description
            virtual XmlRpc::XmlRpcValue prepareConnectionNegotiation(
                    const SubscriptionPtr & subscription, 
                    const TransportDescription & transport_description,
                    const TransportFilters & transport_filters); 

            // For Subscription::pendingConnectionDone
            virtual PublisherLinkPtr createPublisherLink(const std::string & topic_name, 
                    const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response) = 0;

            // For TopicManager::requestTopic
            // will eventually create the connections and listener, as appropriate
            virtual bool processTopicRequest(const std::string & topic_name, 
                    const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply) = 0;

            // For TransportPluginManager::shutdown
            virtual void shutdown() = 0;
    };

};

#endif // TRANSPORT_PLUGIN_H
