package org.marsroboticsassociation.controllib.motion;

import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import java.util.function.LongSupplier;

public class PositionTrajectoryManager {

    private double vMax;
    private double aMaxAccel;
    private double aMaxDecel;
    private double jMax;

    private final LongSupplier clock;
    private final TelemetryAddData telemetry;
    private final SetOnChange<Double> targetPosition;
    private PositionTrajectory currentTrajectory;
    private long startTime;
    private double lastP;
    private double lastV;
    private double lastA;
    private double pendingTarget;

    /** Production constructor — uses System.nanoTime as the clock. */
    public PositionTrajectoryManager(double vMax, double aMaxAccel, double aMaxDecel,
                                     double jMax, double pChangeTolerance,
                                     TelemetryAddData telemetry) {
        this(vMax, aMaxAccel, aMaxDecel, jMax, pChangeTolerance, telemetry, System::nanoTime);
    }

    /** Test constructor — allows injecting a simulated clock. */
    public PositionTrajectoryManager(double vMax, double aMaxAccel, double aMaxDecel,
                                     double jMax, double pChangeTolerance,
                                     TelemetryAddData telemetry, LongSupplier clock) {
        this.clock    = clock;
        this.telemetry = telemetry;
        pendingTarget  = Double.NaN;
        updateConfig(vMax, aMaxAccel, aMaxDecel, jMax);
        lastP = 0.0;
        lastV = 0.0;
        lastA = 0.0;
        startTime = clock.getAsLong();
        changeTrajectory(0);
        targetPosition = SetOnChange.ofDouble(0.0, pChangeTolerance, (p) -> {
            pendingTarget = p;
            update();
        });
    }

    /** Update motion limits. Takes effect on the next trajectory plan. */
    public void updateConfig(double vMax, double aMaxAccel, double aMaxDecel, double jMax) {
        this.vMax      = Math.abs(vMax);
        this.aMaxAccel = Math.abs(aMaxAccel);
        this.aMaxDecel = Math.abs(aMaxDecel);
        this.jMax      = Math.abs(jMax);
    }

    /** Plan a new SCurvePosition from current state to the given target position. */
    private void changeTrajectory(double pTarget) {
        currentTrajectory = new SCurvePosition(
                lastP, pTarget, lastV, lastA,
                vMax, aMaxAccel, aMaxDecel, jMax);
        startTime     = clock.getAsLong();
        pendingTarget = Double.NaN;
    }

    /**
     * Sample the trajectory at the current time and cache position/velocity/acceleration.
     * Call once per control loop.
     */
    public void update() {
        double seconds = (clock.getAsLong() - startTime) / 1e9;

        lastP = currentTrajectory.getPosition(seconds);
        lastV = currentTrajectory.getVelocity(seconds);
        lastA = currentTrajectory.getAcceleration(seconds);

        telemetry.addData("trajectory position",     "%.3f", lastP);
        telemetry.addData("trajectory velocity",     "%.3f", lastV);
        telemetry.addData("trajectory acceleration", "%.1f", lastA);

        if (!Double.isNaN(pendingTarget)) {
            changeTrajectory(pendingTarget);
        }
    }

    /**
     * Discard the current trajectory and restart from a directly measured state.
     * Acceleration is assumed zero.
     */
    public void resetFromMeasurement(double measuredP, double measuredV) {
        resetFromMeasurement(measuredP, measuredV, 0.0);
    }

    /**
     * Discard the current trajectory and restart from directly measured position,
     * velocity, and acceleration.
     */
    public void resetFromMeasurement(double measuredP, double measuredV, double measuredA) {
        this.lastP = measuredP;
        this.lastV = measuredV;
        this.lastA = measuredA;
        this.pendingTarget = Double.NaN;
        changeTrajectory(targetPosition.get());
    }

    /** Command a new target position. */
    public void setTarget(double pTarget) {
        targetPosition.set(pTarget);
    }

    public double getTarget() {
        return targetPosition.get();
    }

    public double getPosition() {
        return lastP;
    }

    public double getVelocity() {
        return lastV;
    }

    public double getAcceleration() {
        return lastA;
    }
}
