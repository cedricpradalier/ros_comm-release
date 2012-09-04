

#include <ros/transport_plugin_manager.h>
#include <ros/transport_plugin.h>
#include <ros/transport/tcp_transport_plugin.h>
#include <ros/transport/udp_transport_plugin.h>
#include <boost/algorithm/string.hpp>

#define USE_CLASS_LOADER
#ifdef USE_CLASS_LOADER
#include <pluginlib/class_loader.h>
#endif

using namespace boost::algorithm;

namespace ros
{

    struct TransportPluginManager::Loaders {
#ifdef USE_CLASS_LOADER
        boost::shared_ptr< pluginlib::ClassLoader<ros::TransportPlugin> > transports_;
        boost::shared_ptr< pluginlib::ClassLoader<ros::TransportFilterPlugin> > filters_;
#endif
        Loaders() {
#ifdef USE_CLASS_LOADER
            try {
                transports_.reset(new pluginlib::ClassLoader<TransportPlugin>("ros_transport_plugins", "ros::TransportPlugin"));
                filters_.reset(new pluginlib::ClassLoader<TransportFilterPlugin>("ros_transport_plugins", "ros::TransportFilterPlugin"));
            } catch(pluginlib::LibraryLoadException& ex){
                // if it fails here, then we have a big problem
                ROS_ERROR("Failed to instantiate class loader. Error: %s", ex.what());
                transports_.reset();
                filters_.reset();
            }
#endif
        };

        ~Loaders() {
            transports_.reset();
            filters_.reset();
        }
    };


    boost::mutex g_transport_plugin_mutex;
    TransportPluginManagerPtr g_transport_plugin;
    const TransportPluginManagerPtr& TransportPluginManager::instance()
    {
        if (!g_transport_plugin)
        {
            boost::mutex::scoped_lock lock(g_transport_plugin_mutex);
            if (!g_transport_plugin)
            {
                g_transport_plugin.reset(new TransportPluginManager);
                g_transport_plugin->loadStaticPlugins();
                // I'd rather do that in ros::init
                // g_transport_plugin->loadDynamicPlugins();
            }
        }

        return g_transport_plugin;
    }

    void TransportPluginManager::registerTransport(TransportPluginPtr plugin) {
        const TransportPluginManagerPtr& tpm = instance();
        tpm->loadPlugin(plugin);
    }

    void TransportPluginManager::unregisterTransport(TransportPluginPtr plugin) {
        const TransportPluginManagerPtr& tpm = instance();
        if (tpm) {
            tpm->unloadPlugin(plugin);
        }
    }

    void TransportPluginManager::registerFilter(TransportFilterPluginPtr plugin) {
        const TransportPluginManagerPtr& tpm = instance();
        tpm->loadFilter(plugin);
    }

    void TransportPluginManager::unregisterFilter(TransportFilterPluginPtr plugin) {
        const TransportPluginManagerPtr& tpm = instance();
        if (tpm) {
            tpm->unloadFilter(plugin);
        }
    }

    bool TransportPluginManager::applyFilters(const TransportFilters & transport_filters, 
                    const SerializedMessage & src, SerializedMessage & dest) {
        const TransportPluginManagerPtr& tpm = instance();
        if (tpm) {
            return tpm->executeFilters(transport_filters, true, src, dest);
        } else {
            ROS_ERROR("TransportPluginManager::applyFilters cannot be applied without an instance");
            return false;
        }
    }

    bool TransportPluginManager::unapplyFilters(const TransportFilters & transport_filters, 
                    const SerializedMessage & src, SerializedMessage & dest) {
        const TransportPluginManagerPtr& tpm = instance();
        if (tpm) {
            return tpm->executeFilters(transport_filters, false, src, dest);
        } else {
            ROS_ERROR("TransportPluginManager::unapplyFilters cannot be applied without an instance");
            return false;
        }
    }

    TransportPluginManager::TransportPluginManager() {
    }

    TransportPluginManager::~TransportPluginManager() {
        shutdown();
    }

    bool TransportPluginManager::shutdown() {
        boost::mutex::scoped_lock lock(g_transport_plugin_mutex);
        // Explicitly empty the maps to make sure that the objects get cleared
        // before the class loaders
        name_map_.clear();
        protocol_map_.clear();
        filter_map_.clear();
        loaders_.reset();
        return true;
    }

    bool TransportPluginManager::loadDynamicPlugins() {
        ROS_DEBUG("Loading dynamic plugins");
        loaders_.reset(new Loaders());
#ifdef USE_CLASS_LOADER
        if (loaders_->transports_) {
            std::vector< std::string > plugin_names = loaders_->transports_->getDeclaredClasses();
            for (unsigned int i=0;i < plugin_names.size(); i++) {
                try{
                    TransportPluginPtr transport = loaders_->transports_->createInstance(plugin_names[i]);
                    loadPlugin(transport);
                    ROS_DEBUG("Registered transport plugin %s for protocol %s",
                            transport->getName().c_str(),transport->getProtocolName().c_str());
                } catch(pluginlib::LibraryLoadException& ex){
                    //handle the class failing to load
                    ROS_ERROR("The library failed to load for some reason. Error: %s", ex.what());
                } catch(pluginlib::PluginlibException& ex){
                    //handle the class failing to load
                    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
                }
            }
        } 

        if (loaders_->filters_) {
            std::vector< std::string > filter_names = loaders_->filters_->getDeclaredClasses();
            for (unsigned int i=0;i < filter_names.size(); i++) {
                try{
                    TransportFilterPluginPtr filter = loaders_->filters_->createInstance(filter_names[i]);
                    loadFilter(filter);
                    ROS_DEBUG("Registered transport filter plugin %s", filter->getName().c_str());
                } catch(pluginlib::LibraryLoadException& ex){
                    //handle the class failing to load
                    ROS_ERROR("The library failed to load for some reason. Error: %s", ex.what());
                } catch(pluginlib::PluginlibException& ex){
                    //handle the class failing to load
                    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
                }
            }
        }
#endif
        return true;
    }

    std::vector<std::string> TransportPluginManager::getRegisteredFilters() const {
        std::vector<std::string> result;
        FilterMap::const_iterator it;
        for (it = filter_map_.begin();it != filter_map_.end(); it++) {
            result.push_back(it->first);
        }
        return result;
    }

    typedef std::map<TransportPluginPtr, std::string> InvTransportMap;
    M_string TransportPluginManager::getRegisteredPlugins() const {
        M_string result;
        InvTransportMap imap;
        TransportMap::const_iterator it;
        for (it = protocol_map_.begin();it != protocol_map_.end(); it++) {
            imap.insert(InvTransportMap::value_type(it->second,it->first));
        }
        for (it = name_map_.begin();it != name_map_.end(); it++) {
            InvTransportMap::const_iterator fit = imap.find(it->second);
            assert (fit != imap.end());
            result.insert(M_string::value_type(it->first,fit->second));
        }
        return result;
    }

    TransportFilterPluginPtr TransportPluginManager::findFilterByName(const std::string & name) {
        FilterMap::iterator it = filter_map_.find(to_upper_copy(name));
        if (it == filter_map_.end()) {
            return TransportFilterPluginPtr();
        } else {
            return it->second;
        }
    }

    TransportPluginPtr TransportPluginManager::findByName(const std::string & name) {
        TransportMap::iterator it = name_map_.find(to_upper_copy(name));
        if (it == name_map_.end()) {
            return TransportPluginPtr();
        } else {
            return it->second;
        }
    }

    TransportPluginPtr TransportPluginManager::findByProtocol(const std::string & protocol){
        TransportMap::iterator it = protocol_map_.find(to_upper_copy(protocol));
        if (it == protocol_map_.end()) {
            return TransportPluginPtr();
        } else {
            return it->second;
        }
    }



    bool TransportPluginManager::loadFilter(TransportFilterPluginPtr plugin)
    {
        filter_map_.insert(FilterMap::value_type(to_upper_copy(plugin->getName()),plugin));
        return true;
    }

    bool TransportPluginManager::unloadFilter(TransportFilterPluginPtr plugin)
    {
        FilterMap::iterator it = filter_map_.find(to_upper_copy(plugin->getName()));
        if (it != filter_map_.end()) {
            filter_map_.erase(it);
        }
        return true;
    }

    bool TransportPluginManager::executeFilters(const TransportFilters & transport_filters, 
            bool forward, const SerializedMessage & src, SerializedMessage & dest) {
        const std::vector<FilterDescription> & vf = transport_filters.getFilterDescriptions();
        dest = src;
        if (forward) {
            std::vector<FilterDescription>::const_iterator it;
            for (it=vf.begin(); it != vf.end(); it++) {
                TransportFilterPluginPtr tfp = findFilterByName(it->getName());
                if (tfp) {
                    SerializedMessage newdest;
                    if (!tfp->apply(*it, forward, dest, newdest)) {
                        // No error message here, we assume that the TFP would have
                        // done it
                        return false;
                    } else {
                        dest = newdest;
                    }
                } else {
                    ROS_ERROR("Cannot find filter '%s'",it->getName().c_str());
                    return false;
                }
            }
        } else {
            std::vector<FilterDescription>::const_reverse_iterator it;
            for (it=vf.rbegin(); it != vf.rend(); it++) {
                TransportFilterPluginPtr tfp = findFilterByName(it->getName());
                if (tfp) {
                    SerializedMessage newdest;
                    if (!tfp->apply(*it, forward, dest, newdest)) {
                        // No error message here, we assume that the TFP would have
                        // done it
                        return false;
                    } else {
                        dest = newdest;
                    }
                } else {
                    ROS_ERROR("Cannot find filter '%s'",it->getName().c_str());
                    return false;
                }
            }
        }
        return true;
    }

    bool TransportPluginManager::loadPlugin(TransportPluginPtr plugin)
    {
        name_map_.insert(TransportMap::value_type(to_upper_copy(plugin->getName()),plugin));
        protocol_map_.insert(TransportMap::value_type(to_upper_copy(plugin->getProtocolName()),plugin));
        return true;
    }

    bool TransportPluginManager::unloadPlugin(TransportPluginPtr plugin)
    {
        TransportMap::iterator it = name_map_.find(to_upper_copy(plugin->getName()));
        if (it != name_map_.end()) {
            name_map_.erase(it);
        }
        it = protocol_map_.find(to_upper_copy(plugin->getProtocolName()));
        if (it != protocol_map_.end()) {
            protocol_map_.erase(it);
        }
        return true;
    }

    bool TransportPluginManager::loadStaticPlugins()
    {
        loadPlugin(TransportPluginPtr(new TCPTransportPlugin()));
        loadPlugin(TransportPluginPtr(new UDPTransportPlugin()));

        return true;
    }

}
