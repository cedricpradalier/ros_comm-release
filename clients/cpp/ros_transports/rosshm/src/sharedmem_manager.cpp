
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <boost/thread.hpp>

#include "rosshm/SHMReleaseMemory.h"
#include "rosshm/SHMClearAll.h"
#include "rosshm/SHMGetBlocks.h"
#include "rosshm/SharedMemBlock.h"
#include "rosshm/SharedMemoryBlock.h"

using namespace boost::interprocess;
using namespace ros;
using namespace rosshm;

boost::mutex main_mutex;
managed_shared_memory *segment = NULL;
SharedMemoryBlock *blockmgr= NULL;
std::string segment_name = SharedMemoryBlock::default_segment_name;
int segment_size = 1000000;
int rpc_port = 11322;


bool release_memory(rosshm::SHMReleaseMemory::Request &req, 
		rosshm::SHMReleaseMemory::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    shm_handle shm = blockmgr->findHandle(*segment,req.topic.c_str());
	if (!shm.is_valid()) {
		ROS_ERROR("Trying to release topic %s not managed by sharedmem_manager",
				req.topic.c_str());
		res.result = -1;
	} else {
        blockmgr->resetBlock(*segment, shm);
		ROS_INFO("Released topic %s",req.topic.c_str());
	}
	return true;
}

bool clear_memory(rosshm::SHMClearAll::Request &req, 
		rosshm::SHMClearAll::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    blockmgr->resetAllBlocks(*segment);
	ROS_INFO("Deleted all handles");
	res.result = 0;
	return true;
}

bool get_blocks(rosshm::SHMGetBlocks::Request &req, 
		rosshm::SHMGetBlocks::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	res.blocks = blockmgr->getBlockList();
	return true;
}

void requestSegmentName(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    result[0] = 1;
    result[1] = segment_name;
}


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "sharedmem_manager");
	ros::NodeHandle n("sharedmem_manager");
	n.param<int>("rpc_port",rpc_port,11322);
	n.param<int>("segment_size",segment_size,1000000);
	n.param<std::string>("segment_name",segment_name,segment_name);

    XMLRPCManager xmlrpc_manager;
    xmlrpc_manager.bind("requestSegmentName", requestSegmentName);
    xmlrpc_manager.start(rpc_port);

   //Remove shared memory on construction and destruction
    struct shm_remove {
        std::string name_;
        shm_remove(const std::string & name) : name_(name) { 
            shared_memory_object::remove(name_.c_str()); 
        }
        ~shm_remove(){ 
            ROS_INFO("Destroying shared memory object");
            shared_memory_object::remove(name_.c_str()); 
        }
    } remover(segment_name);

   //Managed memory segment that allocates portions of a shared memory
   //segment with the default management algorithm
   managed_shared_memory managed_shm(create_only, segment_name.c_str(), segment_size);
   segment = &managed_shm;
   blockmgr = segment->find_or_construct<SharedMemoryBlock>("Manager")();
   ROS_INFO("Created segment %p, and constructed block %p",segment,blockmgr);

   ros::ServiceServer releaseMemSrv = n.advertiseService("release_memory",release_memory);
   ros::ServiceServer clearMemSrv = n.advertiseService("clear_memory",clear_memory);
   ros::ServiceServer getBlocksSrv = n.advertiseService("get_blocks",get_blocks);

   ROS_INFO("Created shared memory with %d bytes",segment_size);

   ros::spin();

   ROS_INFO("Exiting from spin, block %p",blockmgr);
   if (blockmgr) {
       ROS_INFO("Clearing all shared memory blocks");
       blockmgr->resetAllBlocks(*segment);
       blockmgr = NULL;
   }
#if 0
   // This create an abort. Not sure why, but it is not critical since we
   // destroy the memory segment just after
   if (segment) {
       ROS_INFO("Destroying manager");
       segment->destroy<SharedMemoryBlock>("Manager");
       segment = NULL;
   }
#endif
}



