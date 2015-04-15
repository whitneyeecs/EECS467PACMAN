/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __maebot_map_data_t_hpp__
#define __maebot_map_data_t_hpp__

#include <vector>
#include "maebot_occupancy_grid_t.hpp"
#include "maebot_processed_laser_scan_t.hpp"


class maebot_map_data_t
{
    public:
        int64_t    utime;

        maebot_occupancy_grid_t grid;

        int32_t    path_num;

        std::vector< float > path_x;

        std::vector< float > path_y;

        maebot_processed_laser_scan_t scan;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "maebot_map_data_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int maebot_map_data_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int maebot_map_data_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int maebot_map_data_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t maebot_map_data_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* maebot_map_data_t::getTypeName()
{
    return "maebot_map_data_t";
}

int maebot_map_data_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->grid._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->path_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->path_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->path_x[0], this->path_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->path_num > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->path_y[0], this->path_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->scan._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int maebot_map_data_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->grid._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->path_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->path_num) {
        this->path_x.resize(this->path_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->path_x[0], this->path_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->path_num) {
        this->path_y.resize(this->path_num);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->path_y[0], this->path_num);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->scan._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int maebot_map_data_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += this->grid._getEncodedSizeNoHash();
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, this->path_num);
    enc_size += __float_encoded_array_size(NULL, this->path_num);
    enc_size += this->scan._getEncodedSizeNoHash();
    return enc_size;
}

int64_t maebot_map_data_t::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == maebot_map_data_t::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)maebot_map_data_t::getHash };

    int64_t hash = 0x02dce663390413baLL +
         maebot_occupancy_grid_t::_computeHash(&cp) +
         maebot_processed_laser_scan_t::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif