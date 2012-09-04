
#include <bzlib.h>
#include "rosbz2_filter/bz2_transport_filter_plugin.h"
#include <pluginlib/class_list_macros.h>


using namespace ros;

PLUGINLIB_DECLARE_CLASS(rosbz2_filter, BZ2TransportFilterPlugin, rosbz2_filter::BZ2TransportFilterPlugin, ros::TransportFilterPlugin);

// #define HEX_DEBUG

namespace rosbz2_filter {

#ifdef HEX_DEBUG
    static void hexdump(const std::string & prefix, const uint8_t * buffer, unsigned int n) {
        printf("%s: %p [ ",prefix.c_str(),buffer);
        for (unsigned int i=0;i<n;i++) {
            printf("%02X ",buffer[i]);
        }
        printf(" ]\n");
    }
#endif

    BZ2TransportFilterPlugin::~BZ2TransportFilterPlugin() {
    }

    BZ2FilterDescription::~BZ2FilterDescription() {
    }

    bool BZ2TransportFilterPlugin::apply(const FilterDescription & filter_description, bool forward, 
                    const SerializedMessage & src, SerializedMessage & dest) {
        if (forward) {
            return compress(src,dest);
        } else {
            return decompress(src,dest);
        }
    }

    bool BZ2TransportFilterPlugin::compress( const SerializedMessage & src, SerializedMessage & dest) {
        int ret = 0;
        // Size following recommendation in bzip2 manual 101% + 600 bytes
        uint32_t destLen = (102*src.num_bytes)/100 + 600;
        boost::shared_array<uint8_t> buf(new uint8_t[destLen+8]);
#ifdef HEX_DEBUG
        hexdump("Compress src:",src.buf.get(),src.num_bytes);
#endif
        // Copy the first 4 bytes (original length) to the beginning of the
        // buffer + 4. This will be useful for decompressing
        memcpy(buf.get()+4,src.buf.get(),4);
        // Removed the first 4 bytes corresponding the data length
        ret = BZ2_bzBuffToBuffCompress((char*)buf.get()+8,&destLen, (char*)src.buf.get()+4, src.num_bytes-4, 5, 0, 30);
        if (ret != BZ_OK) {
            ROS_ERROR("BZ2_bzBuffToBuffCompress return %d",ret);
            return false;
        }
        // Finally set the 4 first byte to the data length
        *((uint32_t*)buf.get()) = destLen+4;
        dest = SerializedMessage(buf,destLen + 8);
#ifdef HEX_DEBUG
        hexdump("Compress dest:",dest.buf.get(),dest.num_bytes);
#endif
        printf("Message compression: from %d to %d bytes\n",src.num_bytes,dest.num_bytes);
        return true;
    }

    bool BZ2TransportFilterPlugin::decompress( const SerializedMessage & src, SerializedMessage & dest) {
        int ret = 0;
        assert(src.num_bytes >= 4);
        uint32_t destLen =  *((uint32_t*)src.buf.get());
        boost::shared_array<uint8_t> buf(new uint8_t[destLen]);
#ifdef HEX_DEBUG
        hexdump("Decompress src:",src.buf.get(),src.num_bytes);
#endif
        ret = BZ2_bzBuffToBuffDecompress((char*)buf.get(),&destLen, (char*)src.buf.get()+4, src.num_bytes-4, 0, 0);
        if (ret != BZ_OK) {
            ROS_ERROR("BZ2_bzBuffToBuffDecompress return %d",ret);
            return false;
        }
        // Finally set the 4 first byte to the data length
        dest = SerializedMessage(buf,destLen);
#ifdef HEX_DEBUG
        hexdump("Decompress dest:",dest.buf.get(),dest.num_bytes);
#endif
        return true;
    }

};
