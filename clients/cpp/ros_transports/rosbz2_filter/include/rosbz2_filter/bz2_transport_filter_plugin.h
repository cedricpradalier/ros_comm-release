#ifndef ROS_BZ2_TRANSPORT_FILTER_PLUGIN_H
#define ROS_BZ2_TRANSPORT_FILTER_PLUGIN_H

#include <ros/transport_plugin.h>

namespace rosbz2_filter {


    class BZ2FilterDescription : public ros::FilterDescription {
        public:
            BZ2FilterDescription() :
                FilterDescription("BZ2") {}
            BZ2FilterDescription(const ros::FilterDescription & f) :
                FilterDescription(f) {}

            virtual ~BZ2FilterDescription();
    };
    typedef boost::shared_ptr<BZ2FilterDescription> BZ2FilterDescriptionPtr;
                

    class BZ2TransportFilterPlugin : public ros::TransportFilterPlugin {
        public:
            BZ2TransportFilterPlugin() : ros::TransportFilterPlugin("BZ2") {}
            virtual ~BZ2TransportFilterPlugin();


            virtual bool apply(const ros::FilterDescription & filter_description, bool forward, 
                    const ros::SerializedMessage & src, ros::SerializedMessage & dest);

            virtual ros::FilterDescription getDefaultFilterDescription() const {
                return BZ2FilterDescription();
            }
        protected:
            bool compress(const ros::SerializedMessage & src, ros::SerializedMessage & dest);
            bool decompress(const ros::SerializedMessage & src, ros::SerializedMessage & dest);
    };

    typedef boost::shared_ptr<BZ2TransportFilterPlugin> BZ2TransportFilterPluginPtr;

};

#endif // ROS_BZ2_TRANSPORT_FILTER_PLUGIN_H
