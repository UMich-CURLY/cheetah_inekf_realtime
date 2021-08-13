/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package cheetah_inekf_lcm;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class pose_t implements lcm.lcm.LCMEncodable
{
    public long seq;
    public double stamp;
    public String frame_id;
    public double body[];
 
    public pose_t()
    {
        body = new double[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x81a59b4b48f12127L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(cheetah_inekf_lcm.pose_t.class))
            return 0L;
 
        classes.add(cheetah_inekf_lcm.pose_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        char[] __strbuf = null;
        outs.writeLong(this.seq); 
 
        outs.writeDouble(this.stamp); 
 
        __strbuf = new char[this.frame_id.length()]; this.frame_id.getChars(0, this.frame_id.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.body[a]); 
        }
 
    }
 
    public pose_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public pose_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static cheetah_inekf_lcm.pose_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        cheetah_inekf_lcm.pose_t o = new cheetah_inekf_lcm.pose_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        this.seq = ins.readLong();
 
        this.stamp = ins.readDouble();
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.frame_id = new String(__strbuf);
 
        this.body = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.body[a] = ins.readDouble();
        }
 
    }
 
    public cheetah_inekf_lcm.pose_t copy()
    {
        cheetah_inekf_lcm.pose_t outobj = new cheetah_inekf_lcm.pose_t();
        outobj.seq = this.seq;
 
        outobj.stamp = this.stamp;
 
        outobj.frame_id = this.frame_id;
 
        outobj.body = new double[(int) 3];
        System.arraycopy(this.body, 0, outobj.body, 0, 3); 
        return outobj;
    }
 
}
