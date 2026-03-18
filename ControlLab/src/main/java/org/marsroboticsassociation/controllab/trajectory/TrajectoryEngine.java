package org.marsroboticsassociation.controllab.trajectory;

import org.marsroboticsassociation.controllib.motion.PositionTrajectoryManager;
import org.marsroboticsassociation.controllib.motion.VelocityTrajectoryManager;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Simulation engine that drives one trajectory planner at 20 ms per tick.
 * Designed to be called from the Swing EDT by a javax.swing.Timer.
 *
 * Params are staged via setXxxParams() and applied to the underlying
 * planner only when applyParamsAndGoTo() is called (button press).
 */
public class TrajectoryEngine {

    private static final TelemetryAddData NO_OP = (c, f, v) -> {};
    private static final double CYCLE_S  = 0.020; // 20 ms
    private static final long   CYCLE_NS = (long)(CYCLE_S * 1e9);

    // ------------------------------------------------------------------
    // Ruckig availability check (done once at class load)
    // ------------------------------------------------------------------
    private static final boolean RUCKIG_AVAILABLE;
    static {
        boolean ok = false;
        try {
            Class.forName("org.marsroboticsassociation.controllib.motion.ruckig.RuckigController");
            try (var r = new org.marsroboticsassociation.controllib.motion.ruckig.RuckigController(1, CYCLE_S)) {
                ok = true;
            }
        } catch (UnsatisfiedLinkError | Exception e) {
            ok = false;
        }
        RUCKIG_AVAILABLE = ok;
    }

    public static boolean isRuckigAvailable() {
        return RUCKIG_AVAILABLE;
    }

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    private TrajectoryType type;

    // --- SCurvePosition ---
    private PositionTrajectoryManager posManager;
    private double pendingVMax = 10, pendingAAccel = 5, pendingADecel = 5, pendingJMax = 50;

    // --- SCurveVelocity ---
    private VelocityTrajectoryManager velManager;
    private double pendingAMax = 1197, pendingJInc = 2669, pendingJDec = 800;

    // --- Ruckig ---
    private org.marsroboticsassociation.controllib.motion.ruckig.RuckigController ruckig;
    private org.marsroboticsassociation.controllib.motion.ruckig.RuckigInput  rIn;
    private org.marsroboticsassociation.controllib.motion.ruckig.RuckigOutput rOut;
    private double  pendingRuckigVMax = 10, pendingRuckigAMax = 5, pendingRuckigJMax = 50;
    private boolean ruckigFinished = true;

    // --- Simulated wall clock (nanoseconds) ---
    private long clockNs = 0;

    // --- Cached outputs ---
    private double lastP = 0, lastV = 0, lastA = 0;
    private double currentTarget = 0;
    private boolean moving = false;

    public TrajectoryEngine(TrajectoryType type) {
        this.type = type;
        buildPlanner();
    }

    // ------------------------------------------------------------------
    // Param staging (call any time; applied only on next button press)
    // ------------------------------------------------------------------

    public void setPositionParams(double vMax, double aAccel, double aDecel, double jMax) {
        this.pendingVMax   = vMax;
        this.pendingAAccel = aAccel;
        this.pendingADecel = aDecel;
        this.pendingJMax   = jMax;
    }

    public void setVelocityParams(double aMax, double jInc, double jDec) {
        this.pendingAMax = aMax;
        this.pendingJInc = jInc;
        this.pendingJDec = jDec;
    }

    public void setRuckigParams(double vMax, double aMax, double jMax) {
        this.pendingRuckigVMax = vMax;
        this.pendingRuckigAMax = aMax;
        this.pendingRuckigJMax = jMax;
    }

    // ------------------------------------------------------------------
    // Button-press action: apply staged params + command new target
    // ------------------------------------------------------------------

    public void applyParamsAndGoTo(double target) {
        this.currentTarget = target;
        this.moving = true;
        switch (type) {
            case SCURVE_POSITION:
                posManager.updateConfig(pendingVMax, pendingAAccel, pendingADecel, pendingJMax);
                posManager.setTarget(target);
                break;
            case SCURVE_VELOCITY:
                velManager.updateConfig(pendingAMax, pendingJInc, pendingJDec);
                velManager.setTarget(target);
                break;
            case RUCKIG:
                if (rIn != null) {
                    rIn.maxVelocity[0]        = pendingRuckigVMax;
                    rIn.maxAcceleration[0]    = pendingRuckigAMax;
                    rIn.maxJerk[0]            = pendingRuckigJMax;
                    rIn.targetPosition[0]     = target;
                    rIn.targetVelocity[0]     = 0;
                    rIn.targetAcceleration[0] = 0;
                    ruckigFinished = false;
                }
                break;
        }
    }

    // ------------------------------------------------------------------
    // Simulation tick (call every 20 ms from Swing Timer)
    // ------------------------------------------------------------------

    public void tick() {
        if (!moving) return;
        clockNs += CYCLE_NS;

        switch (type) {
            case SCURVE_POSITION:
                posManager.update();
                lastP = posManager.getPosition();
                lastV = posManager.getVelocity();
                lastA = posManager.getAcceleration();
                if (Math.abs(lastP - currentTarget) < 0.01 && Math.abs(lastV) < 0.01) {
                    moving = false;
                }
                break;

            case SCURVE_VELOCITY:
                velManager.update();
                lastP = 0;
                lastV = velManager.getVelocity();
                lastA = velManager.getAcceleration();
                if (Math.abs(lastV - currentTarget) < 1.0 && Math.abs(lastA) < 1.0) {
                    moving = false;
                }
                break;

            case RUCKIG:
                if (ruckig == null || ruckigFinished) {
                    moving = false;
                    return;
                }
                var result = ruckig.update(rIn, rOut);
                rIn.currentPosition[0]     = rOut.newPosition[0];
                rIn.currentVelocity[0]     = rOut.newVelocity[0];
                rIn.currentAcceleration[0] = rOut.newAcceleration[0];
                lastP = rOut.newPosition[0];
                lastV = rOut.newVelocity[0];
                lastA = rOut.newAcceleration[0];
                if (result == org.marsroboticsassociation.controllib.motion.ruckig.RuckigResult.FINISHED) {
                    ruckigFinished = true;
                    moving = false;
                }
                break;
        }
    }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    public double getPosition()     { return lastP; }
    public double getVelocity()     { return lastV; }
    public double getAcceleration() { return lastA; }
    public boolean isMoving()       { return moving; }
    public boolean hasPosition()    { return type != TrajectoryType.SCURVE_VELOCITY; }
    public TrajectoryType getType() { return type; }

    // ------------------------------------------------------------------
    // Type switch: tear down old planner, build new one
    // ------------------------------------------------------------------

    public void switchType(TrajectoryType newType) {
        if (this.type == newType) return;
        tearDownPlanner();
        this.type      = newType;
        this.moving    = false;
        this.lastP = 0; this.lastV = 0; this.lastA = 0;
        this.clockNs   = 0;
        buildPlanner();
    }

    // ------------------------------------------------------------------
    // Internal planner lifecycle
    // ------------------------------------------------------------------

    private void buildPlanner() {
        switch (type) {
            case SCURVE_POSITION:
                posManager = new PositionTrajectoryManager(
                        pendingVMax, pendingAAccel, pendingADecel, pendingJMax,
                        0.001, NO_OP, () -> clockNs);
                break;
            case SCURVE_VELOCITY:
                velManager = new VelocityTrajectoryManager(
                        pendingAMax, pendingJInc, 1.0, NO_OP, () -> clockNs);
                velManager.updateConfig(pendingAMax, pendingJInc, pendingJDec);
                break;
            case RUCKIG:
                if (RUCKIG_AVAILABLE) {
                    ruckig = new org.marsroboticsassociation.controllib.motion.ruckig.RuckigController(1, CYCLE_S);
                    rIn    = new org.marsroboticsassociation.controllib.motion.ruckig.RuckigInput(1);
                    rOut   = new org.marsroboticsassociation.controllib.motion.ruckig.RuckigOutput(1);
                    rIn.maxVelocity[0]     = pendingRuckigVMax;
                    rIn.maxAcceleration[0] = pendingRuckigAMax;
                    rIn.maxJerk[0]         = pendingRuckigJMax;
                    ruckigFinished = true;
                }
                break;
        }
    }

    private void tearDownPlanner() {
        if (ruckig != null) {
            ruckig.close();
            ruckig = null;
        }
        posManager = null;
        velManager = null;
    }
}
