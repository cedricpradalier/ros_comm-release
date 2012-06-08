
#include "ros/transport_hints.h"
#include "ros/transport_plugin_manager.h"
#include "ros/file_log.h"

#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <ctype.h>



namespace ros {

    TransportHints& TransportHints::plugin(const std::string & name)
    {
        std::string NAME = boost::algorithm::to_upper_copy(name);
        // ROS_INFO("TransportHints::plugin %s to %s",name.c_str(),NAME.c_str());
        const TransportPluginManagerPtr & tpm = TransportPluginManager::instance();
        TransportPluginPtr tp_it = tpm->findByName(NAME);
        if (tp_it) {
            // ROS_INFO("TransportHints %s", tp_it->getName().c_str());
            transports_.push_back(tp_it->getDefaultTransportDescription());
            // ROS_INFO("Pushed back %s",transports_[transports_.size()-1]->getName().c_str());
        } else {
            ROS_WARN("[TransportHints] Plugin [%s] is not registered", NAME.c_str());
        }
        return *this;
    }

    V_string TransportHints::getTransports() const {
        V_string res;
        for (unsigned int i=0;i<transports_.size();i++) {
            res.push_back(transports_[i].getName());
        }
        return res;
    }

    M_string TransportHints::getOptions() const {
        M_string res;
        for (unsigned int i=0;i<transports_.size();i++) {
            M_stringPtr tmp = transports_[i].getOptionMap();
            res.insert(tmp->begin(),tmp->end());
        }
        return res;
    }

    std::string TransportFilters::getFilterString() const {
        std::string res;
        if (filters_.size() == 0) {
            return res;
        }
        res = filters_[0].getName();
        for (unsigned int i=1;i<filters_.size();i++) {
            res = res + "|" + filters_[i].getFilterString();
        }
        return res;
    }

    V_string TransportFilters::getFilterNames() const {
        V_string res;
        for (unsigned int i=0;i<filters_.size();i++) {
            res.push_back(filters_[i].getName());
        }
        return res;
    }

    bool TransportFilters::createFiltersFromString(const std::string & filters) {
        TransportPluginManagerPtr tpm = TransportPluginManager::instance();
        filters_.clear();
        if (filters.empty()) {
            return true;
        }

        std::vector<std::string> filter_list;
        boost::algorithm::split(filter_list, filters, boost::is_any_of("|"));
        for (unsigned int i=0;i<filter_list.size();i++) {
            std::string filter_name = FilterDescription::extractFilterName(filter_list[i]);
            TransportFilterPluginPtr tfp = tpm->findFilterByName(filter_name);
            if (tfp) {
                FilterDescription fd = tfp->getDefaultFilterDescription();
                if (fd.setParametersFromString(filter_list[i])) {
                    filters_.push_back(fd);
                } else {
                    ROS_ERROR("createFiltersFromString: '%s' error while parsing parameters '%s'",
                            filter_name.c_str(), filter_list[i].c_str());
                    return false;
                }
            } else {
                ROS_ERROR("createFiltersFromString: cannot find filter '%s'",filter_name.c_str());
                return false;
            }
        }
        return true;
    }

};

