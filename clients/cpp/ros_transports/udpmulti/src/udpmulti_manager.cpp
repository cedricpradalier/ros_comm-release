
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include "udpmulti/UDPMultRegisterTopic.h"
#include "udpmulti/UDPMultClearAll.h"
#include "udpmulti/UDPMultGetTopicList.h"

using namespace XmlRpc;

typedef std::map< std::string, unsigned int ,std::less<std::string> > TopicPortMap;
unsigned int availablePort = 1024;
std::string multicast_address = "239.255.0.1";
TopicPortMap registeredTopic;
boost::mutex main_mutex;
int rpc_port = 11323;

void request_multicast_channel(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    std::string caller = params[0];
    if (params.size() != 3 ||
            params[1].getType() != XmlRpcValue::TypeString || 
            params[2].getType() != XmlRpcValue::TypeString) {
        ROS_ERROR("[UDPMULTI MANAGER] Invalid XML-RPC request from %s",caller.c_str());
        result[0] = 0;
        return;
    }
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    result[0] = 1;
    result[1] = multicast_address;
    std::string topic_name = params[1];
    std::string filter_string = params[2];
    std::string record_name = topic_name+"["+filter_string+"]";
	TopicPortMap::iterator it = registeredTopic.find(record_name);
    if (it != registeredTopic.end()) {
        result[2] = (int)(it->second);
    } else {
        result[2] = int(availablePort);
        registeredTopic.insert(std::pair<std::string,unsigned int>(record_name,availablePort));
        availablePort += 1;
    }
	ROS_INFO("Registered port %d for topic %s", (int)result[2],record_name.c_str());
}


bool register_topic(udpmulti::UDPMultRegisterTopic::Request &req, 
		udpmulti::UDPMultRegisterTopic::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    std::string record_name = req.topic+"["+req.filter+"]";
	TopicPortMap::iterator it = registeredTopic.find(record_name);
    if (it != registeredTopic.end()) {
        res.multicast_address = multicast_address;
        res.port = it->second;
    } else {
        res.multicast_address = multicast_address;
        res.port = availablePort++;
        registeredTopic.insert(std::pair<std::string,unsigned int>(record_name,res.port));
    }
	ROS_INFO("Registered port %d for topic %s", res.port,record_name.c_str());
	
	return true;
}

bool clear_all_topics(udpmulti::UDPMultClearAll::Request &req, 
		udpmulti::UDPMultClearAll::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    registeredTopic.clear();
    availablePort = 1024;
	ROS_INFO("Deleted all topic-port association");
	res.result = 0;
	return true;
}

bool get_topic_list(udpmulti::UDPMultGetTopicList::Request &req, 
		udpmulti::UDPMultGetTopicList::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    TopicPortMap::const_iterator it;
    unsigned int i=0;
    res.multicast_address = multicast_address;
    res.topics.resize(registeredTopic.size());
    for (it=registeredTopic.begin();it!=registeredTopic.end();it++,i++) {
        udpmulti::UDPMultTopic tpc;
        std::vector<std::string> filter_list;
        boost::algorithm::split(filter_list, it->first, boost::is_any_of("[]"));
        assert(filter_list.size()==2);
        tpc.topic = filter_list[0];
        tpc.filter = filter_list[1];
        tpc.port = it->second;
        res.topics[i] = tpc;
    }
	return true;
}


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "udpmulti_manager");
	ros::NodeHandle n("udpmulti_manager");
    n.param<std::string>("multicast_address",multicast_address,"239.255.0.1");

    ros::XMLRPCManager xmlrpc_manager;
    xmlrpc_manager.bind("requestMulticastChannel", request_multicast_channel);
    xmlrpc_manager.start(rpc_port);

   ros::ServiceServer regTopicSrv = n.advertiseService("register_topic",register_topic);
   ros::ServiceServer clearAllSrv = n.advertiseService("clear_all_topics",clear_all_topics);
   ros::ServiceServer getListSrv = n.advertiseService("get_topics",get_topic_list);
   ROS_INFO("udpmulti_manager started");

   ros::spin();

}



