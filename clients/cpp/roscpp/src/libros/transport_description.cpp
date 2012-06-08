
#include "ros/transport_description.h"
#include <boost/algorithm/string.hpp>

namespace ros {

    // Not super clean
    static std::string escape(std::string in) {
        std::string out;
        for (unsigned int i=0;i<in.size();i++) {
            switch (in[i]) {
                case ':' : out += "%3a"; break;
                case '|' : out += "%7c"; break;
                case '%' : out += "%25"; break;
                case '=' : out += "%3d"; break;
                default: out += in[i]; break;
            }
        }
        return out;
    }

    static std::string unescape(std::string in) {
        std::string out;
        for (unsigned int i=0;i<in.size();i++) {
            if (in[i] == '%') {
                if (i+2>=in.size()) {
                    break;
                }
                int c = 0;
                sscanf(in.c_str()+i+1,"%x",&c);
                if (!c) {
                    break;
                }
                out+=c;
                i+=2;
            } else {
                out += in[i];
            }
        }
        return out;
    }


    std::string FilterDescription::getFilterString() const {
        std::string result = escape(getName()) + ":";
        for (M_string::const_iterator it=parameters_->begin();it!=parameters_->end();it++) {
            result += escape(it->first) + "=" + escape(it->second) + ":";
        }
        return result;
    }

    std::string FilterDescription::extractFilterName(const std::string & s) {
        if (s.empty()) {
            return s;
        } else {
            // Overkill, but performance is not critical here
            // and the string should be small anyway
            std::vector<std::string> strs;
            boost::split(strs, s, boost::is_any_of(":"));
            return strs[0];
        }
    }

    bool FilterDescription::setParametersFromString(const std::string & s) {
        std::vector<std::string> strs;
        boost::split(strs, s, boost::is_any_of(":"));
        if (strs.size()<2) {
            return true;
        }
        for (unsigned int i=0;i<strs.size();i++) {
            std::vector<std::string> param;
            if (strs[i].empty()) {
                break;
            }
            boost::split(param, strs[i], boost::is_any_of("="));
            if (param.size() != 2) {
                ROS_ERROR("Invalid parameter in filter description string '%s'",s.c_str());
                return false;
            }
            setParameter(unescape(param[0]),unescape(param[1]));
        }
        return true;
    }
};

