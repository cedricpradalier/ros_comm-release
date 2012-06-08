
#include "ros/transport_plugin.h"
#include "ros/this_node.h"

namespace ros {

    // To fix the v-table
    TransportPlugin::~TransportPlugin() {
    }

    // To fix the v-table
    TransportFilterPlugin::~TransportFilterPlugin() {
    }

    // To fix the v-table
    TransportPluginInstance::~TransportPluginInstance() {
    }

    XmlRpc::XmlRpcValue TransportPluginInstance::prepareConnectionNegotiation(
            const SubscriptionPtr & subscription, 
            const TransportDescription & transport_description,
            const TransportFilters & transport_filters) {
        XmlRpc::XmlRpcValue rpc_array;
        transport_description_ = transport_description;
        transport_filters_ = transport_filters;
        rpc_array[0] = getProtocolName();
        M_string m;
        m["topic"] = subscription->getName();
        m["md5sum"] = subscription->md5sum();
        m["callerid"] = this_node::getName();
        m["type"] = subscription->datatype();
        m["filters"] = transport_filters.getFilterString();
        ROS_DEBUG("Negociation: sent filter string [%s]",m["filters"].c_str());
        boost::shared_array<uint8_t> buffer;
        uint32_t len;
        Header::write(m, buffer, len);
        XmlRpc::XmlRpcValue v(buffer.get(), len);
        rpc_array[1] = v;
        return rpc_array;
    }
};


