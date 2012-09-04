#include "rosshm/shm_transport_plugin.h"
#include "rosshm/shm_publisher_link.h"
#include "rosshm/shm_subscriber_link.h"
#include "rosshm/SharedMemoryBlock.h"
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
using namespace rosshm;
using namespace XmlRpc;
using namespace boost::algorithm;

PLUGINLIB_DECLARE_CLASS(rosshm, SHMTransportPlugin, rosshm::SHMTransportPlugin, ros::TransportPlugin);

SHMTransportPlugin::SHMTransportPlugin() : TransportPlugin("SHM", "SHMROS") {
}

SHMTransportPlugin::~SHMTransportPlugin() {
}


SHMTransportPluginInstance::~SHMTransportPluginInstance() {
    // For now this is not calling shutdown on purpose.
    // The SHM transport object might be in use somewhere else
}

XmlRpc::XmlRpcValue SHMTransportPluginInstance::prepareConnectionNegotiation(
        const SubscriptionPtr & subscription, const TransportDescription& transport_description, const TransportFilters & transport_filters) {
    XmlRpc::XmlRpcValue udpros_array = 
        TransportPluginInstance::prepareConnectionNegotiation(subscription,transport_description, transport_filters);
    udpros_array[udpros_array.size()] = network::getHost();
    return udpros_array;
}

// For Subscription::pendingConnectionDone
PublisherLinkPtr SHMTransportPluginInstance::createPublisherLink(const std::string & topic_name, 
        const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response) {
    if (response.size() != 3 ||
            response[1].getType() != XmlRpc::XmlRpcValue::TypeString ||
            response[2].getType() != XmlRpc::XmlRpcValue::TypeBase64)
    {
        ROSCPP_LOG_DEBUG("publisher implements SHMROS, but the " \
                "parameters aren't string,base64");
        shutdown();
        return PublisherLinkPtr();
    }
    XmlRpc::XmlRpcValue reply = response;
    std::string segment_name = reply[1];
    std::vector<char> header_bytes = reply[2];
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
    memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
    Header h;
    std::string err;
    if (!h.parse(buffer, header_bytes.size(), err))
    {
        ROSCPP_LOG_DEBUG("Unable to parse SHMROS connection header: %s", err.c_str());
        shutdown();
        return PublisherLinkPtr();
    }
    ROSCPP_LOG_DEBUG("Connecting via SHMros to topic [%s]", topic_name.c_str());

    std::string error_msg;
    if (h.getValue("error", error_msg))
    {
        ROSCPP_LOG_DEBUG("Received error message in header for connection to [%s]: [%s]", 
                XMLRPCManager::instance()->getServerURI().c_str(), error_msg.c_str());
        shutdown();
        return PublisherLinkPtr();
    }

    SHMPublisherLinkPtr pub_link(new SHMPublisherLink(subscription,
                XMLRPCManager::instance()->getServerURI(), transport_description_, transport_filters_));
    if (pub_link->setHeader(h))
    {
        pub_link->initialize(segment_name, topic_name);

        ROSCPP_LOG_DEBUG("Connected to publisher of topic [%s] via SHM", topic_name.c_str());

        return pub_link;
    } else {
        ROSCPP_LOG_DEBUG("Failed to connect to publisher of topic [%s] via SHM", topic_name.c_str());
        shutdown();
        return PublisherLinkPtr();
    }
}

bool SHMTransportPluginInstance::requestSegmentName() {
    XmlRpc::XmlRpcValue request,reply;
    request[0] = this_node::getName();
    std::string xmlrpc_uri("http://localhost:11322");
    const char * env = getenv("ROS_SHM_URI");
    if (!env) {
        ROS_WARN("No ROS_SHM_URI defined, assuming %s",xmlrpc_uri.c_str());
    } else {
        xmlrpc_uri = env;
        ROS_INFO("Contacting sharedmem server at %s",xmlrpc_uri.c_str());
    }
    std::string peer_host;
    uint32_t peer_port;
    if (!network::splitURI(xmlrpc_uri, peer_host, peer_port))
    {
        ROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
        return false;
    }

    XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
    if (!c.execute("requestSegmentName", request, reply)) {
        ROS_ERROR("Error while requesting segment name");
        return false;
    }
    if (reply.size() != 2 ||
            reply[1].getType() != XmlRpcValue::TypeString)
    {
        ROSCPP_LOG_DEBUG("Invalid reply from sharedmem manager");
        return false;
    }
    ROS_INFO("Connecting to shared memory segment");
    segment_name_ = std::string(reply[1]);
    return true;
}


// For TopicManager::requestTopic
// will eventually create the connections and listener, as appropriate
bool SHMTransportPluginInstance::processTopicRequest(const std::string & topic_name, 
        const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply) {
    // Default value, when returning false
    XmlRpc::XmlRpcValue SHMros_params;
    SHMros_params[0] = std::string(getProtocolName());
    reply[0] = int(0);
    reply[1] = std::string();
    reply[2] = SHMros_params;

    if (request.size() != 3 ||
            request[1].getType() != XmlRpc::XmlRpcValue::TypeBase64 || 
            request[2].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        ROSCPP_LOG_DEBUG("Invalid protocol parameters for SHMROS");
        return false;
    }
    XmlRpc::XmlRpcValue proto = request;
    std::vector<char> header_bytes = proto[1];
    std::string client_host = proto[2];
    if (client_host != network::getHost()) {
        ROS_ERROR("ROSSHM is only possible one the same machine (pub=%s, sub=%s)",network::getHost().c_str(),client_host.c_str());
        return false;
    }


    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
    memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
    Header h;
    std::string err;
    if (!h.parse(buffer, header_bytes.size(), err))
    {
        ROSCPP_LOG_DEBUG("Unable to parse SHMROS connection header: %s", err.c_str());
        return false;
    }

    TopicManagerPtr topic_manager = TopicManager::instance();
    PublicationPtr pub_ptr = topic_manager->lookupPublication(topic_name);
    if(!pub_ptr)
    {
        ROSCPP_LOG_DEBUG("Unable to find advertised topic %s for SHMROS connection", topic_name.c_str());
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
    bool found = pub_ptr->hasSubscriber(getProtocolName(),filter_string);

    if (!requestSegmentName()) {
        ROSCPP_LOG_DEBUG("Could not contact sharedmem manager");
        return false;
    }
    if (!found) {

        SHMSubscriberLinkPtr sub_link(new SHMSubscriberLink());
        if (!sub_link->initialize(segment_name_, topic_name, filter_string)) {
            ROSCPP_LOG_DEBUG("Error while initialising Subscriber link");
            return false;
        }
        if (!sub_link->handleHeader(h)) { // to the contrary of udp plugin, this does not add the subscriber by default
            ROSCPP_LOG_DEBUG("Error while validating Subscriber header");
            return false;
        }
        pub_ptr->addSubscriberLink(sub_link);
        m["filters"] = sub_link->getFilterString();
        ROSCPP_LOG_DEBUG("Added subscriber for topic %s via SHM",topic_name.c_str());
    } else {
        m["filters"] = filter_string; // From header
        ROSCPP_LOG_DEBUG("There is already a subscriber for topic %s via SHM",topic_name.c_str());
    }

    SHMros_params[1] = segment_name_;
    m["topic"] = topic_name;
    m["md5sum"] = pub_ptr->getMD5Sum();
    m["type"] = pub_ptr->getDataType();
    m["callerid"] = this_node::getName();
    m["message_definition"] = pub_ptr->getMessageDefinition();
    boost::shared_array<uint8_t> msg_def_buffer;
    uint32_t len;
    Header::write(m, msg_def_buffer, len);
    XmlRpc::XmlRpcValue v(msg_def_buffer.get(), len);
    SHMros_params[2] = v;
    reply[0] = int(1);
    reply[1] = std::string();
    reply[2] = SHMros_params;
    return true;
}

// For TransportPluginManager::shutdown
void SHMTransportPluginInstance::shutdown() {
}

