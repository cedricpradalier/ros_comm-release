#ifndef ROSCPP_TRANSPORT_DESCRIPTION_H
#define ROSCPP_TRANSPORT_DESCRIPTION_H

#include <string>
#include <boost/lexical_cast.hpp>
#include "ros/common.h"
#include "ros/header.h"

namespace ros
{
    /**
     * \brief Provides a way of describe transport filter hints to ros::NodeHandle::subscribe() and
     * Shall be used by TransportHints mostly
     * Warning: this class should not be virtualised. Some objects of it will
     * be inserted into TransportHints which might be destroyed only after
     * shutdown, so after unloading the plugins, leading to a segfault. 
     * */
    class ROSCPP_DECL FilterDescription {
        protected:
            std::string name_;
            M_stringPtr parameters_;

        public:
            FilterDescription(const std::string & name) 
                : name_(name),parameters_(new M_string()) {}
            ~FilterDescription() {}

            const std::string & getName() const {return name_;}

            /**
             * \brief Returns a map of textual representation of the filter parameters
             * */
            M_stringPtr getParameterMap() const {return parameters_;}

            /**
             * \brief Read the filter parameters from a textual representation
             * */
            bool setParametersFromMap(const M_string & map) {
                *parameters_ = map;
                return true;
            }

            /**
             * \brief Return the filter name followed by its parameters in the form of 
             * name=value, separated by colons
             * */
            std::string getFilterString() const; 

            /**
             * \brief Set the parameters from the filter string description
             * */
            bool setParametersFromString(const std::string & s); 

            /** 
             * \brief Extract the filter name from a filter string
             * */
            static std::string extractFilterName(const std::string & s);

            void setParameter(const std::string & name, const std::string & value) {
                (*parameters_)[name] = value;
            }

            bool getParameter(const std::string & name, std::string & value) const {
                M_string::const_iterator it = parameters_->find(name);
                if (it == parameters_->end()) {
                    return false;
                }
                value = it->second;
                return true;
            }

    };

    typedef boost::shared_ptr<FilterDescription> FilterDescriptionPtr;

    /**
     * \brief Provides a way of describe network transport hints to ros::NodeHandle::subscribe() and
     * Shall be used by TransportHints mostly
     * Warning: this class should not be virtualised. Some objects of it will
     * be inserted into TransportHints which might be destroyed only after
     * shutdown, so after unloading the plugins, leading to a segfault. 
     * */
    class ROSCPP_DECL TransportDescription {
        protected:
            std::string name_;
            M_stringPtr parameters_;
        public:
            TransportDescription(const std::string & name) 
                : name_(name), parameters_(new M_string()) {}
            ~TransportDescription() {}

            const std::string & getName() const {return name_;}

            M_stringPtr getOptionMap() const {return parameters_;}

            void setParameter(const std::string & name, const std::string & value) {
                (*parameters_)[name] = value;
            }

            bool getParameter(const std::string & name, std::string & value) const {
                M_string::const_iterator it = parameters_->find(name);
                if (it == parameters_->end()) {
                    return false;
                }
                value = it->second;
                return true;
            }
    };

    typedef boost::shared_ptr<TransportDescription> TransportDescriptionPtr;

    class ROSCPP_DECL TCPTransportDescription : public TransportDescription {
        public:
            TCPTransportDescription(bool no_delay = false) :
                TransportDescription("TCP") { 
                    setNoDelay(no_delay);
                }

            TCPTransportDescription(const TransportDescription & t) :
                TransportDescription(t) {}

            ~TCPTransportDescription() {}

            bool getNoDelay() const {
                M_string::const_iterator it = parameters_->find("tcp_nodelay");
                if (it == parameters_->end()) {
                    return false;
                }
                return ((it->second)=="true")?true:false;
            }
            void setNoDelay(bool no_delay) {
                setParameter("tcp_nodelay",no_delay?"true":"false");
            }
    };
    typedef boost::shared_ptr<TCPTransportDescription> TCPTransportDescriptionPtr;
                

    class ROSCPP_DECL UDPTransportDescription : public TransportDescription {
        public:
            UDPTransportDescription(size_t max_datagram_size = 0) :
                TransportDescription("UDP") {
                    setMaxDatagramSize(max_datagram_size);
                }
            UDPTransportDescription(const TransportDescription & t) :
                TransportDescription(t) {}

            ~UDPTransportDescription() {}

            size_t getMaxDatagramSize() const {
                M_string::const_iterator it = parameters_->find("max_datagram_size");
                if (it == parameters_->end()) {
                    return 0;
                }
                try {
                    return boost::lexical_cast<int>(it->second);
                } catch (boost::bad_lexical_cast&) {
                    return 0;
                } 
            }
            void setMaxDatagramSize(size_t max_datagram_size) {
                char tmp[128];
                sprintf(tmp,"%d",max_datagram_size);
                (*parameters_)["max_datagram_size"] = tmp;
            }
    };
    typedef boost::shared_ptr<UDPTransportDescription> UDPTransportDescriptionPtr;
                
    class ROSCPP_DECL IntraTransportDescription : public TransportDescription {
        public:
            IntraTransportDescription() :
                TransportDescription("INTRA") {}

            ~IntraTransportDescription() {}

            M_string getOptionMap() const {
                return M_string();
            }
    };
    typedef boost::shared_ptr<IntraTransportDescription> IntraTransportDescriptionPtr;
                

};


#endif // ROSCPP_TRANSPORT_DESCRIPTION_H
