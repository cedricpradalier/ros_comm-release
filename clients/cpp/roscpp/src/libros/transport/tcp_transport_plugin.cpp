
#include "ros/common.h"
#include "ros/subscription.h"
#include "ros/publication.h"
#include "ros/transport_publisher_link.h"
#include "ros/network.h"
#include "ros/connection.h"
#include "ros/poll_manager.h"
#include "ros/transport/transport_tcp.h"
#include "ros/connection_manager.h"
#include "ros/file_log.h"
#include "ros/transport_hints.h"
#include "ros/transport/tcp_transport_plugin.h"

using namespace ros;

TCPTransportPlugin::TCPTransportPlugin() : TransportPlugin("TCP", "TCPROS") {
}

TCPTransportPlugin::~TCPTransportPlugin() {
}


TCPTransportPluginInstance::~TCPTransportPluginInstance() {
}

// For Subscription::pendingConnectionDone
PublisherLinkPtr TCPTransportPluginInstance::createPublisherLink(const std::string & topic_name, 
        const SubscriptionPtr & subscription, const XmlRpc::XmlRpcValue & response) {
    if (response.size() != 3 ||
            response[1].getType() != XmlRpc::XmlRpcValue::TypeString ||
            response[2].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
        ROSCPP_LOG_DEBUG("publisher implements TCPROS, but the " \
                "parameters aren't string,int");
        return PublisherLinkPtr();
    }
    XmlRpc::XmlRpcValue reply = response;
    std::string pub_host = reply[1];
    int pub_port = reply[2];
    ROSCPP_LOG_DEBUG("Connecting via tcpros to topic [%s] at host [%s:%d]", topic_name.c_str(), pub_host.c_str(), pub_port);

    TransportTCPPtr transport(new TransportTCP(&PollManager::instance()->getPollSet()));
    if (transport->connect(pub_host, pub_port))
    {
        ConnectionPtr connection(new Connection());
        TransportPublisherLinkPtr pub_link(new TransportPublisherLink(subscription, 
                    XMLRPCManager::instance()->getServerURI(), transport_description_, transport_filters_));

        connection->initialize(transport, false, HeaderReceivedFunc());
        pub_link->initialize(connection);

        ConnectionManager::instance()->addConnection(connection);
        ROSCPP_LOG_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", topic_name.c_str(), pub_host.c_str(), pub_port);

        return pub_link;
    }
    ROSCPP_LOG_DEBUG("Failed to connect to publisher of topic [%s] at [%s:%d]", topic_name.c_str(), pub_host.c_str(), pub_port);
    return PublisherLinkPtr();
}

// For TopicManager::requestTopic
// will eventually create the connections and listener, as appropriate
bool TCPTransportPluginInstance::processTopicRequest(const std::string & topic_name, 
        const XmlRpc::XmlRpcValue & request, XmlRpc::XmlRpcValue & reply) {
    // TODO: deal with the filters for TCP as well
    ConnectionManagerPtr connection_manager = ConnectionManager::instance();
    XmlRpc::XmlRpcValue tcpros_params;
    tcpros_params[0] = std::string("TCPROS");
    tcpros_params[1] = network::getHost();
    tcpros_params[2] = int(connection_manager->getTCPPort());
    reply[0] = int(1);
    reply[1] = std::string();
    reply[2] = tcpros_params;
    return true;
}

// For TransportPluginManager::shutdown
void TCPTransportPluginInstance::shutdown() {
    // Nothing special here
}
