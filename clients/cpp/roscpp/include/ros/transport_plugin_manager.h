#ifndef TRANSPORT_PLUGIN_MANAGER_H
#define TRANSPORT_PLUGIN_MANAGER_H

#include <ros/transport_plugin.h>


namespace ros
{
    class TransportPluginManager;
    typedef boost::shared_ptr<TransportPluginManager> TransportPluginManagerPtr;

    class TransportPluginManager {
        public:
            static const TransportPluginManagerPtr& instance();
            static void registerFilter(TransportFilterPluginPtr plugin);
            static void unregisterFilter(TransportFilterPluginPtr plugin);
            static void registerTransport(TransportPluginPtr plugin);
            static void unregisterTransport(TransportPluginPtr plugin);

            static bool applyFilters(const TransportFilters & transport_filters, 
                    const SerializedMessage & src, SerializedMessage & dest);

            static bool unapplyFilters(const TransportFilters & transport_filters, 
                    const SerializedMessage & src, SerializedMessage & dest);

            TransportPluginManager();
            ~TransportPluginManager();

            TransportFilterPluginPtr findFilterByName(const std::string & name);
            TransportPluginPtr findByName(const std::string & name);
            TransportPluginPtr findByProtocol(const std::string & protocol);

            M_string getRegisteredPlugins() const;
            std::vector<std::string> getRegisteredFilters() const;

            bool loadStaticPlugins();
            bool loadDynamicPlugins();
            bool shutdown(); // Unload all plugins
        protected:

            bool loadPlugin(TransportPluginPtr plugin);
            bool unloadPlugin(TransportPluginPtr plugin);
            bool loadFilter(TransportFilterPluginPtr plugin);
            bool unloadFilter(TransportFilterPluginPtr plugin);
            bool executeFilters(const TransportFilters & transport_hints, bool forward, 
                    const SerializedMessage & src, SerializedMessage & dest);

        protected:
            struct Loaders;
            boost::shared_ptr<Loaders> loaders_;

            typedef std::map<std::string, TransportPluginPtr> TransportMap;
            TransportMap name_map_;
            TransportMap protocol_map_;
            typedef std::map<std::string, TransportFilterPluginPtr> FilterMap;
            FilterMap filter_map_;


    };
};



#endif // TRANSPORT_PLUGIN_MANAGER_H
