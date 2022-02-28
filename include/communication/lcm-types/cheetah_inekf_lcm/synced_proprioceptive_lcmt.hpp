/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __cheetah_inekf_lcm_synced_proprioceptive_lcmt_hpp__
#define __cheetah_inekf_lcm_synced_proprioceptive_lcmt_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>

namespace cheetah_inekf_lcm
{

class synced_proprioceptive_lcmt
{
    public:
        int8_t     num_legs;

        double     timestamp;

        std::vector< int8_t > contact;

        float      q[12];

        float      qd[12];

        float      p[12];

        float      v[12];

        float      tau_est[12];

        float      quat[4];

        float      rpy[3];

        float      omega[3];

        float      acc[3];

        int64_t    good_packets;

        int64_t    bad_packets;

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
         * @param maxlen The maximum number of bytes to read while decoding.
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
         * Returns "synced_proprioceptive_lcmt"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int synced_proprioceptive_lcmt::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int synced_proprioceptive_lcmt::decode(const void *buf, int offset, int maxlen)
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

int synced_proprioceptive_lcmt::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t synced_proprioceptive_lcmt::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* synced_proprioceptive_lcmt::getTypeName()
{
    return "synced_proprioceptive_lcmt";
}

int synced_proprioceptive_lcmt::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_legs, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_legs > 0) {
        tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->contact[0], this->num_legs);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->q[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->qd[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->p[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->v[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_est[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->quat[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->rpy[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->omega[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->acc[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->good_packets, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->bad_packets, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int synced_proprioceptive_lcmt::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_legs, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_legs) {
        this->contact.resize(this->num_legs);
        tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->contact[0], this->num_legs);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->q[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->qd[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->p[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->v[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_est[0], 12);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->quat[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->rpy[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->omega[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->acc[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->good_packets, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->bad_packets, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int synced_proprioceptive_lcmt::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, this->num_legs);
    enc_size += __float_encoded_array_size(NULL, 12);
    enc_size += __float_encoded_array_size(NULL, 12);
    enc_size += __float_encoded_array_size(NULL, 12);
    enc_size += __float_encoded_array_size(NULL, 12);
    enc_size += __float_encoded_array_size(NULL, 12);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 3);
    enc_size += __float_encoded_array_size(NULL, 3);
    enc_size += __float_encoded_array_size(NULL, 3);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t synced_proprioceptive_lcmt::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0xe5b50cf5e957c239LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
