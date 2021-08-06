/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package cheetah_inekf_lcm;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class legcontrol_t implements lcm.lcm.LCMEncodable
{
    public float q[];
    public float qd[];
    public float p[];
    public float v[];
    public float tau_est[];
 
    public legcontrol_t()
    {
        q = new float[12];
        qd = new float[12];
        p = new float[12];
        v = new float[12];
        tau_est = new float[12];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xa7d2775a407deca7L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(cheetah_inekf_lcm.legcontrol_t.class))
            return 0L;
 
        classes.add(cheetah_inekf_lcm.legcontrol_t.class);
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
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.q[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.qd[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.p[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.v[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.tau_est[a]); 
        }
 
    }
 
    public legcontrol_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public legcontrol_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static cheetah_inekf_lcm.legcontrol_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        cheetah_inekf_lcm.legcontrol_t o = new cheetah_inekf_lcm.legcontrol_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.q = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.q[a] = ins.readFloat();
        }
 
        this.qd = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.qd[a] = ins.readFloat();
        }
 
        this.p = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.p[a] = ins.readFloat();
        }
 
        this.v = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.v[a] = ins.readFloat();
        }
 
        this.tau_est = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.tau_est[a] = ins.readFloat();
        }
 
    }
 
    public cheetah_inekf_lcm.legcontrol_t copy()
    {
        cheetah_inekf_lcm.legcontrol_t outobj = new cheetah_inekf_lcm.legcontrol_t();
        outobj.q = new float[(int) 12];
        System.arraycopy(this.q, 0, outobj.q, 0, 12); 
        outobj.qd = new float[(int) 12];
        System.arraycopy(this.qd, 0, outobj.qd, 0, 12); 
        outobj.p = new float[(int) 12];
        System.arraycopy(this.p, 0, outobj.p, 0, 12); 
        outobj.v = new float[(int) 12];
        System.arraycopy(this.v, 0, outobj.v, 0, 12); 
        outobj.tau_est = new float[(int) 12];
        System.arraycopy(this.tau_est, 0, outobj.tau_est, 0, 12); 
        return outobj;
    }
 
}
