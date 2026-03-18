package org.marsroboticsassociation.controllib.motion.ruckig;

/**
 * Java wrapper for a Ruckig&lt;DynamicDOFs&gt; instance.
 *
 * <p>Usage (e.g. 1-DOF, 20 ms cycle):
 * <pre>{@code
 * try (RuckigController r = new RuckigController(1, 0.02)) {
 *     RuckigInput  in  = new RuckigInput(1);
 *     RuckigOutput out = new RuckigOutput(1);
 *     in.currentPosition[0]     = 0;
 *     in.targetPosition[0]      = 1;
 *     in.maxVelocity[0]         = 1;
 *     in.maxAcceleration[0]     = 2;
 *     in.maxJerk[0]             = 10;
 *     RuckigResult result;
 *     do {
 *         result = r.update(in, out);
 *         // apply out.newPosition[0] ...
 *         in.currentPosition[0]     = out.newPosition[0];
 *         in.currentVelocity[0]     = out.newVelocity[0];
 *         in.currentAcceleration[0] = out.newAcceleration[0];
 *     } while (result == RuckigResult.WORKING);
 * }
 * }</pre>
 *
 * <p>Requires the native library {@code ruckig_jni} to be loaded. On Android this
 * comes from the RuckigNative AAR; on desktop it must be on {@code java.library.path}.
 */
public class RuckigController implements AutoCloseable {

    static {
        System.loadLibrary("ruckig_jni");
    }

    private final long handle;
    private final int dofs;
    private final double[] durationBuf = new double[1];

    public RuckigController(int dofs, double cycleTime) {
        this.dofs = dofs;
        this.handle = nativeCreate(dofs, cycleTime);
    }

    /**
     * Advance the trajectory by one cycle.
     *
     * @param in  kinematic limits and current/target state (read)
     * @param out next kinematic state (written)
     * @return    {@link RuckigResult#WORKING} until target reached,
     *            then {@link RuckigResult#FINISHED}
     */
    public RuckigResult update(RuckigInput in, RuckigOutput out) {
        int code = nativeUpdate(handle,
                in.currentPosition,     in.currentVelocity,     in.currentAcceleration,
                in.targetPosition,      in.targetVelocity,      in.targetAcceleration,
                in.maxVelocity,         in.maxAcceleration,     in.maxJerk,
                out.newPosition,        out.newVelocity,        out.newAcceleration,
                durationBuf);
        out.duration = durationBuf[0];
        return RuckigResult.fromCode(code);
    }

    @Override
    public void close() {
        nativeDestroy(handle);
    }

    // -------------------------------------------------------------------------
    // Native methods — implemented in RuckigNative/src/main/cpp/ruckig_jni.cpp
    // -------------------------------------------------------------------------

    private native long nativeCreate(int dofs, double cycleTime);

    private native int nativeUpdate(long handle,
            double[] currentPos,  double[] currentVel,  double[] currentAcc,
            double[] targetPos,   double[] targetVel,   double[] targetAcc,
            double[] maxVel,      double[] maxAcc,      double[] maxJerk,
            double[] outPos,      double[] outVel,      double[] outAcc,
            double[] outDuration);

    private native void nativeDestroy(long handle);
}
