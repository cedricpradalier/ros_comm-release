
#include "ros/ros.h"
#include "ros/transport_plugin_manager.h"
// #include "ros_transport_plugin/ros_transport_plugin.h"

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "list_all_transport");
    // In ros::init
    // ros_transport_plugin::registerAllTransportPlugins();

    ros::M_string plugins = ros::TransportPluginManager::instance()->getRegisteredPlugins();

    printf("---------------- ----------------\n");
    printf("%-16s %-16s\n","Name","Protocol");
    printf("---------------- ----------------\n");
    for (ros::M_string::const_iterator it = plugins.begin(); it != plugins.end(); it++) {
        printf("%-16s %-16s\n",it->first.c_str(),it->second.c_str());
    }
    printf("\n----------------\n");
    printf("Filters\n");
    printf("----------------\n");
    const std::vector<std::string> filters = ros::TransportPluginManager::instance()->getRegisteredFilters();
    for (unsigned int i=0;i<filters.size();i++) {
        printf("%-16s\n",filters[i].c_str());
    }
    printf("---\n");
    return 0;
}

