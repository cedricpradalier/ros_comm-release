#ifndef SHARED_MEMORY_BLOCK_H
#define SHARED_MEMORY_BLOCK_H

#include <cassert>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <ros/ros.h>
#include <ros/file_log.h>
#include "rosshm/SharedMemoryBlockDescriptor.h"
#include "rosshm/SharedMemBlock.h"

namespace rosshm {

    struct shm_handle {
        uint32_t handle;
        uint8_t *ptr;
        uint32_t resize_count;
        shm_handle() : handle(-1), ptr(NULL), resize_count(0) {}
        shm_handle(uint32_t h, uint32_t rcount, uint8_t *p) : handle(h), ptr(p), resize_count(rcount) {}
        bool is_valid() const {return ptr != NULL;}
    };

    class SharedMemoryBlock {
        public:
            static const unsigned int number_of_shared_blocks = 100;
            static const std::string default_segment_name;
        protected:
            //Mutex to protect access to the queue
            boost::interprocess::interprocess_mutex      mutex;
            boost::interprocess::interprocess_condition  cond;
            int32_t num_clients;
            SharedMemoryBlockDescriptor descriptors[number_of_shared_blocks];


        public:

            SharedMemoryBlock() : num_clients(0) {}

            shm_handle findHandle(boost::interprocess::managed_shared_memory & segment, const char * name) ;


            shm_handle allocateBlock(boost::interprocess::managed_shared_memory & segment, 
                    const char * name, uint32_t size) ;

            void resetBlock(boost::interprocess::managed_shared_memory & segment, shm_handle & shm) ;
            void resetAllBlocks(boost::interprocess::managed_shared_memory & segment) ;

            void reallocateBlock(boost::interprocess::managed_shared_memory & segment, 
                    shm_handle & shm, uint32_t size) ;

            bool wait_data(boost::interprocess::managed_shared_memory & segment,
                    shm_handle & src, ros::SerializedMessage & msg) {
                {
                    ROS_DEBUG("Locking %d",src.handle);
                    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> 
                        lock(descriptors[src.handle].mutex); 
                    if (!descriptors[src.handle].wait_data_and_register_client(lock))  {
                        return false;
                    }
                    if (!ros::ok()) {
                        lock.unlock();
                        descriptors[src.handle].unregister_client();
                        return false;
                    }
                    register_global_client();
                    ROS_DEBUG("Unlocking %d",src.handle);
                }
                boost::shared_array<uint8_t> buf(new uint8_t[descriptors[src.handle].size_]);
                deserialize(segment,src,buf.get());
                msg = ros::SerializedMessage(buf,descriptors[src.handle].size_);
                unregister_global_client();
                ROS_DEBUG("Unregistering %d",src.handle);
                descriptors[src.handle].unregister_client();
                if (!ros::ok()) return false;
                return true;
            }

            void serialize(boost::interprocess::managed_shared_memory & segment,
                    shm_handle & dest, const uint8_t *buffer, unsigned int size) {
                boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(descriptors[dest.handle].mutex);
                ROS_DEBUG("serialize: locked %d, checking clients",dest.handle);
                descriptors[dest.handle].check_clients(lock);
                ROS_DEBUG("serialize: locked %d, clients checked",dest.handle);
                register_global_client();
                ROS_DEBUG("serialize: global clients checked");

                assert(dest.handle < number_of_shared_blocks);
                if (dest.resize_count != descriptors[dest.handle].resize_count_) {
                    std::pair<uint8_t *, std::size_t> ret = segment.find<uint8_t>(descriptors[dest.handle].name_);
                    dest.resize_count = descriptors[dest.handle].resize_count_;
                    dest.ptr = ret.first;
                }
                ROSCPP_LOG_DEBUG("Serialising to %p, %d(%d) bytes",
                        dest.ptr,descriptors[dest.handle].size_,size);
                memcpy(dest.ptr,buffer,size);
                unregister_global_client();
                ROS_DEBUG("serialize: global clients released");
                descriptors[dest.handle].signal_data();
                ROS_DEBUG("serialize: unlocking %d",dest.handle);
            }

            std::vector<rosshm::SharedMemBlock> getBlockList() const ;
        protected:

            void deserialize(boost::interprocess::managed_shared_memory & segment,
                    shm_handle & src, uint8_t * buffer) {
                assert(src.handle < number_of_shared_blocks);
                if (src.resize_count != descriptors[src.handle].resize_count_) {
                    std::pair<uint8_t *, std::size_t> ret = segment.find<uint8_t>(descriptors[src.handle].name_);
                    src.resize_count = descriptors[src.handle].resize_count_;
                    src.ptr = ret.first;
                }
                ROSCPP_LOG_DEBUG("Deserialising from %p, %d bytes",src.ptr,descriptors[src.handle].size_);
                memcpy(buffer,src.ptr,descriptors[src.handle].size_);
            }


            void hexdump(const std::string & prefix, 
                    const boost::shared_array<uint8_t> & buf, unsigned int size) {
                printf("%s: %p [ ",prefix.c_str(), buf.get());
                for (unsigned int i=0;i<size;i++) {
                    printf("%02X ",buf[i]);
                }
                printf("]\n");
            }

            shm_handle connectBlock(boost::interprocess::managed_shared_memory & segment, uint32_t handle) ;

            void check_global_clients(boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> & lock) {
                if (num_clients) {
                    ROS_DEBUG("Lock_global wait");
                    cond.wait(lock);
                }
                ROS_DEBUG("Lock_global done");
            }


            void register_global_client() {
                ROS_DEBUG("register_global_client:: Locking global");
                boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex); 
                num_clients ++;
                ROS_DEBUG("Registered global client");
            }

            void unregister_global_client() {
                ROS_DEBUG("unregister_global_client:: Locking global");
                boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex); 
                num_clients --;
                assert(num_clients >= 0);
                if (num_clients == 0) {
                    ROS_DEBUG("Global lock is free");
                    cond.notify_all();
                }
                ROS_DEBUG("Unregistered global client");
            }


    };
} //namespace image_transport

#endif // SHARED_MEMORY_BLOCK_H
