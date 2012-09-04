#include "udpmulti/udpmulti_transport_plugin.h"
#include "udpmulti/udpmulti_publisher_link.h"
#include "udpmulti/udpmulti_subscriber_link.h"
#include "ros/common.h"
#include "ros/subscription.h"
#include "ros/publication.h"
#include "ros/connection.h"
#include "ros/this_node.h"
#include "ros/file_log.h"
#include "ros/network.h"
#include "ros/transport_hints.h"
#include "ros/topic_manager.h"
#include <boost/algorithm/string.hpp>

#include <pluginlib/class_list_macros.h>

#include "XmlRpc.h"

using namespace ros;
using namespace udpmulti;
using namespace XmlRpc;
using namespace boost::algorithm;

PLUGINLIB_DECLARE_CLASS(udpmulti, UDPMultiTransportPlugin, udpmulti::UDPMultiTransportPlugin, ros::TransportPlugin);

UDPMultiTransportPlugin::UDPMultiTransportPlugin() : TransportPlugin("UDPMULTI", "MULTICASTROS") {
}

UDPMultiTransportPlugin::~UDPMultiTransportPlugin() {
}

UDPMultiTransportDescription::~UDPMultiTransportDescription() {
}

UDPMultiTransportPluginInstance::~UDPMultiTransportPluginInstance() {
    // For now this is not calling shutdown on purpose.
    // The UDPMulti transport object might be in use somewhere else
}


// For Subscription::pendingConnectionDone
PublisherLinkPtr UDPMultiTransportPluginInstance::createPublisherLink(const std::string & topic_name, 
        const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response) {
    if (response.size() != 4 ||
            response[1].getType() != XmlRpc::XmlRpcValue::TypeString ||
            response[2].getType() != XmlRpc::XmlRpcValue::TypeInt ||
            response[3].getType() != XmlRpc::XmlRpcValue::TypeBase64)
    {
        ROSCPP_LOG_DEBUG("publisher implements %s, but the " \
                "parameters aren't string,base64",getProtocolName().c_str());
        shutdown();
        return PublisherLinkPtr();
    }
    XmlRpc::XmlRpcValue reply = response;
    multicast_ip_ = std::string(reply[1]);
    multicast_port_ = (int)(reply[2]);
    std::vector<char> header_bytes = reply[3];
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
    memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
    Header h;
    std::string err;
    if (!h.parse(buffer, header_bytes.size(), err))
    {
        ROSCPP_LOG_DEBUG("Unable to parse %s connection header: %s", getProtocolName().c_str(), err.c_str());
        shutdown();
        return PublisherLinkPtr();
    }
    ROSCPP_LOG_DEBUG("Connecting via %s to topic [%s]", getProtocolName().c_str(), topic_name.c_str());

    std::string error_msg;
    if (h.getValue("error", error_msg))
    {
        ROSCPP_LOG_DEBUG("Received error message in header for connection to [%s]: [%s]", 
                XMLRPCManager::instance()->getServerURI().c_str(), error_msg.c_str());
        shutdown();
        return PublisherLinkPtr();
    }

    UDPMultiPublisherLinkPtr pub_link(new UDPMultiPublisherLink(subscription,
                XMLRPCManager::instance()->getServerURI(), transport_description_, transport_filters_));
    if (pub_link->setHeader(h))
    {
        pub_link->initialize(multicast_ip_, multicast_port_);

        ROSCPP_LOG_DEBUG("Connected to publisher of topic [%s] via UDPMulti", topic_name.c_str());

        return pub_link;
    } else {
        ROSCPP_LOG_DEBUG("Failed to connect to publisher of topic [%s] via UDPMulti", topic_name.c_str());
        shutdown();
        return PublisherLinkPtr();
    }
}

bool UDPMultiTransportPluginInstance::requestMulticastChannel(const std::string & topic_name,
        const std::string & filter_string) {
    XmlRpc::XmlRpcValue request,reply;
    request[0] = this_node::getName();
    request[1] = topic_name;
    request[2] = filter_string;
    std::string xmlrpc_uri("http://localhost:11323");
    const char * env = getenv("ROS_MULTICAST_URI");
    if (!env) {
        ROS_WARN("No ROS_MULTICAST_URI defined, assuming %s",xmlrpc_uri.c_str());
    } else {
        xmlrpc_uri = env;
        ROS_INFO("Contacting udpmulticast manager at %s",xmlrpc_uri.c_str());
    }
    std::string peer_host;
    uint32_t peer_port;
    if (!network::splitURI(xmlrpc_uri, peer_host, peer_port))
    {
        ROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
        return false;
    }

    XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
    if (!c.execute("requestMulticastChannel", request, reply)) {
        ROS_ERROR("Error while requesting segment name");
        return false;
    }
    if (reply.size() != 3 ||
            reply[1].getType() != XmlRpcValue::TypeString || 
            reply[2].getType() != XmlRpcValue::TypeInt)
    {
        ROSCPP_LOG_DEBUG("Invalid reply from udpmulti manager");
        return false;
    }
    multicast_ip_ = std::string(reply[1]);
    multicast_port_ = int(reply[2]);
    ROS_INFO("Connecting to udp multicast channel %s:%d",multicast_ip_.c_str(),multicast_port_);
    return true;
}


// For TopicManager::requestTopic
// will eventually create the connections and listener, as appropriate
bool UDPMultiTransportPluginInstance::processTopicRequest(const std::string & topic_name, 
        const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply) {
    if (request.size() != 2 ||
            request[1].getType() != XmlRpc::XmlRpcValue::TypeBase64)
    {
        ROSCPP_LOG_DEBUG("Invalid protocol parameters for UDPMultiROS");
        return false;
    }
    XmlRpc::XmlRpcValue proto = request;
    std::vector<char> header_bytes = proto[1];
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
    memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
    Header h;
    std::string err;
    if (!h.parse(buffer, header_bytes.size(), err))
    {
        ROSCPP_LOG_DEBUG("Unable to parse UDPMultiROS connection header: %s", err.c_str());
        return false;
    }

    TopicManagerPtr topic_manager = TopicManager::instance();
    PublicationPtr pub_ptr = topic_manager->lookupPublication(topic_name);
    if(!pub_ptr)
    {
        ROSCPP_LOG_DEBUG("Unable to find advertised topic %s for UDPMultiROS connection", topic_name.c_str());
        return false;
    }

    M_string m;
    std::string error_msg;
    if (!pub_ptr->validateHeader(h, error_msg))
    {
        ROSCPP_LOG_DEBUG("Error validating header for topic [%s]: %s", topic_name.c_str(), error_msg.c_str());
        return false;
    }



    std::string filter_string;
    h.getValue("filters",filter_string);
    bool found = false;
    found = pub_ptr->hasSubscriber(getProtocolName(),filter_string);

    if (!requestMulticastChannel(topic_name,filter_string)) {
        ROSCPP_LOG_DEBUG("Could not contact sharedmem manager");
        return false;
    }

    if (!found) {
        UDPMultiSubscriberLinkPtr sub_link(new UDPMultiSubscriberLink());
        if (!sub_link->initialize(multicast_ip_, multicast_port_)) {
            ROSCPP_LOG_DEBUG("Error while initialising Subscriber link");
            return false;
        }
        if (!sub_link->handleHeader(h)) { // to the contrary of udp plugin, this does not add the subscriber by default
            ROSCPP_LOG_DEBUG("Error while validating Subscriber header");
            return false;
        }
        pub_ptr->addSubscriberLink(sub_link);
        m["filters"] = sub_link->getFilterString();
        ROSCPP_LOG_DEBUG("Added subscriber for topic %s via UDPMulti",topic_name.c_str());
    } else {
        m["filters"] = filter_string; // From header
        ROSCPP_LOG_DEBUG("There is already a subscriber for topic %s via UDPMulti",topic_name.c_str());
    }

    XmlRpc::XmlRpcValue UDPMultiros_params;
    UDPMultiros_params[0] = std::string(getProtocolName());
    UDPMultiros_params[1] = multicast_ip_;
    UDPMultiros_params[2] = int(multicast_port_);
    m["topic"] = topic_name;
    m["md5sum"] = pub_ptr->getMD5Sum();
    m["type"] = pub_ptr->getDataType();
    m["callerid"] = this_node::getName();
    m["message_definition"] = pub_ptr->getMessageDefinition();
    boost::shared_array<uint8_t> msg_def_buffer;
    uint32_t len;
    Header::write(m, msg_def_buffer, len);
    XmlRpc::XmlRpcValue v(msg_def_buffer.get(), len);
    UDPMultiros_params[3] = v;
    reply[0] = int(1);
    reply[1] = std::string();
    reply[2] = UDPMultiros_params;
    return true;
}

// For TransportPluginManager::shutdown
void UDPMultiTransportPluginInstance::shutdown() {
}

