/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package cheetah_inekf_lcm;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class imu_t implements lcm.lcm.LCMEncodable
{
    public float quat[];
    public float rpy[];
    public float omega[];
    public float acc[];
    public long good_packets;
    public long bad_packets;
 
    public imu_t()
    {
        quat = new float[4];
        rpy = new float[3];
        omega = new float[3];
        acc = new float[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x710a98f509c97d55L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(cheetah_inekf_lcm.imu_t.class))
            return 0L;
 
        classes.add(cheetah_inekf_lcm.imu_t.class);
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
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.quat[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.rpy[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.omega[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.acc[a]); 
        }
 
        outs.writeLong(this.good_packets); 
 
        outs.writeLong(this.bad_packets); 
 
    }
 
    public imu_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public imu_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static cheetah_inekf_lcm.imu_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        cheetah_inekf_lcm.imu_t o = new cheetah_inekf_lcm.imu_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.quat = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.quat[a] = ins.readFloat();
        }
 
        this.rpy = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.rpy[a] = ins.readFloat();
        }
 
        this.omega = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.omega[a] = ins.readFloat();
        }
 
        this.acc = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.acc[a] = ins.readFloat();
        }
 
        this.good_packets = ins.readLong();
 
        this.bad_packets = ins.readLong();
 
    }
 
    public cheetah_inekf_lcm.imu_t copy()
    {
        cheetah_inekf_lcm.imu_t outobj = new cheetah_inekf_lcm.imu_t();
        outobj.quat = new float[(int) 4];
        System.arraycopy(this.quat, 0, outobj.quat, 0, 4); 
        outobj.rpy = new float[(int) 3];
        System.arraycopy(this.rpy, 0, outobj.rpy, 0, 3); 
        outobj.omega = new float[(int) 3];
        System.arraycopy(this.omega, 0, outobj.omega, 0, 3); 
        outobj.acc = new float[(int) 3];
        System.arraycopy(this.acc, 0, outobj.acc, 0, 3); 
        outobj.good_packets = this.good_packets;
 
        outobj.bad_packets = this.bad_packets;
 
        return outobj;
    }
 
}
