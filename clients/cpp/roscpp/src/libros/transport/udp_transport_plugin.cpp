
#include "ros/transport/udp_transport_plugin.h"
#include "ros/common.h"
#include "ros/subscription.h"
#include "ros/publication.h"
#include "ros/transport_publisher_link.h"
#include "ros/connection.h"
#include "ros/transport/transport_tcp.h"
#include "ros/connection_manager.h"
#include "ros/poll_manager.h"
#include "ros/network.h"
#include "ros/this_node.h"
#include "ros/file_log.h"
#include "ros/transport_hints.h"
#include "ros/topic_manager.h"

using namespace ros;

UDPTransportPlugin::UDPTransportPlugin() : TransportPlugin("UDP", "UDPROS") {
}

UDPTransportPlugin::~UDPTransportPlugin() {
}

UDPTransportPluginInstance::~UDPTransportPluginInstance() {
    // For now this is not calling shutdown on purpose.
    // The udp transport object might be in use somewhere else
}


// For Subscription::negotiateConnection
XmlRpc::XmlRpcValue UDPTransportPluginInstance::prepareConnectionNegotiation(
        const SubscriptionPtr & subscription, const TransportDescription& transport_description, const TransportFilters & transport_filters) {
    XmlRpc::XmlRpcValue udpros_array = 
        TransportPluginInstance::prepareConnectionNegotiation(subscription,transport_description, transport_filters);
    UDPTransportDescription udp(transport_description);
    int max_datagram_size = udp.getMaxDatagramSize();
    udp_transport_ = TransportUDPPtr(new TransportUDP(&PollManager::instance()->getPollSet()));
    if (!max_datagram_size) {
        max_datagram_size = udp_transport_->getMaxDatagramSize();
        udp.setMaxDatagramSize(max_datagram_size);
    }
    udp_transport_->createIncoming(0, false);
    udpros_array[udpros_array.size()] = network::getHost();
    udpros_array[udpros_array.size()] = udp_transport_->getServerPort();
    udpros_array[udpros_array.size()] = max_datagram_size;
    return udpros_array;
}

// For Subscription::pendingConnectionDone
PublisherLinkPtr UDPTransportPluginInstance::createPublisherLink(const std::string & topic_name, 
        const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response) {
    if (response.size() != 6 ||
        response[1].getType() != XmlRpc::XmlRpcValue::TypeString ||
        response[2].getType() != XmlRpc::XmlRpcValue::TypeInt ||
        response[3].getType() != XmlRpc::XmlRpcValue::TypeInt ||
        response[4].getType() != XmlRpc::XmlRpcValue::TypeInt ||
        response[5].getType() != XmlRpc::XmlRpcValue::TypeBase64)
    {
      ROSCPP_LOG_DEBUG("publisher implements UDPROS, but the " \
	    	       "parameters aren't string,int,int,int,base64");
      shutdown();
      return PublisherLinkPtr();
    }
    XmlRpc::XmlRpcValue reply = response;
    std::string pub_host = reply[1];
    int pub_port = reply[2];
    int conn_id = reply[3];
    int max_datagram_size = reply[4];
    std::vector<char> header_bytes = reply[5];
    boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
    memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
    Header h;
    std::string err;
    if (!h.parse(buffer, header_bytes.size(), err))
    {
      ROSCPP_LOG_DEBUG("Unable to parse UDPROS connection header: %s", err.c_str());
      shutdown();
      return PublisherLinkPtr();
    }
    ROSCPP_LOG_DEBUG("Connecting via udpros to topic [%s] at host [%s:%d] connection id [%08x] max_datagram_size [%d]", topic_name.c_str(), pub_host.c_str(), pub_port, conn_id, max_datagram_size);

    std::string error_msg;
    if (h.getValue("error", error_msg))
    {
      ROSCPP_LOG_DEBUG("Received error message in header for connection to [%s]: [%s]", 
              XMLRPCManager::instance()->getServerURI().c_str(), error_msg.c_str());
      shutdown();
      return PublisherLinkPtr();
    }

    TransportPublisherLinkPtr pub_link(new TransportPublisherLink(subscription,
                XMLRPCManager::instance()->getServerURI(), transport_description_, transport_filters_));
    if (pub_link->setHeader(h))
    {
      ConnectionPtr connection(new Connection());
      connection->initialize(udp_transport_, false, NULL);
      connection->setHeader(h);
      pub_link->initialize(connection);

      ConnectionManager::instance()->addConnection(connection);

      ROSCPP_LOG_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", topic_name.c_str(), pub_host.c_str(), pub_port);

      return pub_link;
    }
    else
    {
      ROSCPP_LOG_DEBUG("Failed to connect to publisher of topic [%s] at [%s:%d]", topic_name.c_str(), pub_host.c_str(), pub_port);
      shutdown();
      return PublisherLinkPtr();
    }
}

// For TopicManager::requestTopic
// will eventually create the connections and listener, as appropriate
bool UDPTransportPluginInstance::processTopicRequest(const std::string & topic_name, 
        const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply) {
    if (request.size() != 5 ||
            request[1].getType() != XmlRpc::XmlRpcValue::TypeBase64 ||
            request[2].getType() != XmlRpc::XmlRpcValue::TypeString ||
            request[3].getType() != XmlRpc::XmlRpcValue::TypeInt ||
            request[4].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
        ROSCPP_LOG_DEBUG("Invalid protocol parameters for UDPROS");
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
        ROSCPP_LOG_DEBUG("Unable to parse UDPROS connection header: %s", err.c_str());
        return false;
    }

    TopicManagerPtr topic_manager = TopicManager::instance();
    PublicationPtr pub_ptr = topic_manager->lookupPublication(topic_name);
    if(!pub_ptr)
    {
        ROSCPP_LOG_DEBUG("Unable to find advertised topic %s for UDPROS connection", topic_name.c_str());
        return false;
    }

    std::string host = proto[2];
    int port = proto[3];

    M_string m;
    std::string error_msg;
    if (!pub_ptr->validateHeader(h, error_msg))
    {
        ROSCPP_LOG_DEBUG("Error validating header from [%s:%d] for topic [%s]: %s", host.c_str(), port, topic_name.c_str(), error_msg.c_str());
        return false;
    }

    ConnectionManagerPtr connection_manager = ConnectionManager::instance();
    int max_datagram_size = proto[4];
    int conn_id = connection_manager->getNewConnectionID();
    TransportUDPPtr transport = connection_manager->getUDPServerTransport()->createOutgoing(host, port, conn_id, max_datagram_size);
    // This guy is generating a Publication::addSubscriberLink though
    //  - ConnectionManager::onConnectionHeaderReceived 
    //   - TransportSubscriberLink::handleHeader
    connection_manager->udprosIncomingConnection(transport, h);

    XmlRpc::XmlRpcValue udpros_params;
    udpros_params[0] = std::string("UDPROS");
    udpros_params[1] = network::getHost();
    udpros_params[2] = connection_manager->getUDPServerTransport()->getServerPort();
    udpros_params[3] = conn_id;
    udpros_params[4] = max_datagram_size;
    m["topic"] = topic_name;
    m["md5sum"] = pub_ptr->getMD5Sum();
    m["type"] = pub_ptr->getDataType();
    m["callerid"] = this_node::getName();
    m["message_definition"] = pub_ptr->getMessageDefinition();
    std::string filters;
    h.getValue("filters",filters);
    m["filters"] = filters;
    boost::shared_array<uint8_t> msg_def_buffer;
    uint32_t len;
    Header::write(m, msg_def_buffer, len);
    XmlRpc::XmlRpcValue v(msg_def_buffer.get(), len);
    udpros_params[5] = v;
    reply[0] = int(1);
    reply[1] = std::string();
    reply[2] = udpros_params;
    return true;
}

// For TransportPluginManager::shutdown
void UDPTransportPluginInstance::shutdown() {
    if (udp_transport_) {
        udp_transport_->close();
        udp_transport_.reset();
    }
}

