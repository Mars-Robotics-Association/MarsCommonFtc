package org.marsroboticsassociation.controllib.motion;

import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import java.util.function.LongSupplier;

public class PositionTrajectoryManager {

    /**
     * Factory for creating {@link PositionTrajectory} instances. Implement this to swap in a
     * different trajectory type (e.g. {@link SinCurvePosition}) without changing the manager.
     *
     * <p>Example usage:
     *
     * <pre>
     *   new PositionTrajectoryManager(..., SinCurvePosition::new);
     * </pre>
     */
    @FunctionalInterface
    public interface TrajectoryFactory {
        PositionTrajectory create(
                double p0,
                double pTarget,
                double v0,
                double a0,
                double vMax,
                double aMaxAccel,
                double aMaxDecel,
                double jMax);
    }

    private double vMax;
    private double aMaxAccel;
    private double aMaxDecel;
    private double jMax;

    private final LongSupplier clock;
    private final TelemetryAddData telemetry;
    private final TrajectoryFactory factory;
    private final SetOnChange<Double> targetPosition;
    private PositionTrajectory currentTrajectory;
    private long startTime;
    private double lastP;
    private double lastV;
    private double lastA;
    private double pendingTarget;

    /** Production constructor — uses System.nanoTime as the clock and SCurvePosition. */
    public PositionTrajectoryManager(
            double vMax,
            double aMaxAccel,
            double aMaxDecel,
            double jMax,
            double pChangeTolerance,
            TelemetryAddData telemetry) {
        this(
                vMax,
                aMaxAccel,
                aMaxDecel,
                jMax,
                pChangeTolerance,
                telemetry,
                System::nanoTime,
                SCurvePosition::new);
    }

    /** Test constructor — allows injecting a simulated clock. Uses SCurvePosition. */
    public PositionTrajectoryManager(
            double vMax,
            double aMaxAccel,
            double aMaxDecel,
            double jMax,
            double pChangeTolerance,
            TelemetryAddData telemetry,
            LongSupplier clock) {
        this(
                vMax,
                aMaxAccel,
                aMaxDecel,
                jMax,
                pChangeTolerance,
                telemetry,
                clock,
                SCurvePosition::new);
    }

    /** Production constructor with custom trajectory factory. Uses System.nanoTime as the clock. */
    public PositionTrajectoryManager(
            double vMax,
            double aMaxAccel,
            double aMaxDecel,
            double jMax,
            double pChangeTolerance,
            TelemetryAddData telemetry,
            TrajectoryFactory factory) {
        this(
                vMax,
                aMaxAccel,
                aMaxDecel,
                jMax,
                pChangeTolerance,
                telemetry,
                System::nanoTime,
                factory);
    }

    /** Base constructor — all others delegate here. */
    public PositionTrajectoryManager(
            double vMax,
            double aMaxAccel,
            double aMaxDecel,
            double jMax,
            double pChangeTolerance,
            TelemetryAddData telemetry,
            LongSupplier clock,
            TrajectoryFactory factory) {
        this.clock = clock;
        this.telemetry = telemetry;
        this.factory = factory;
        pendingTarget = Double.NaN;
        updateConfig(vMax, aMaxAccel, aMaxDecel, jMax);
        lastP = 0.0;
        lastV = 0.0;
        lastA = 0.0;
        startTime = clock.getAsLong();
        changeTrajectory(0);
        targetPosition =
                SetOnChange.ofDouble(
                        0.0,
                        pChangeTolerance,
                        (p) -> {
                            pendingTarget = p;
                            update();
                        });
    }

    /** Update motion limits. Takes effect on the next trajectory plan. */
    public void updateConfig(double vMax, double aMaxAccel, double aMaxDecel, double jMax) {
        this.vMax = Math.abs(vMax);
        this.aMaxAccel = Math.abs(aMaxAccel);
        this.aMaxDecel = Math.abs(aMaxDecel);
        this.jMax = Math.abs(jMax);
    }

    /**
     * Plan a new trajectory from the currently sampled state to the given target position.
     *
     * <p>The manager carries forward position, velocity, and acceleration only. This makes
     * mid-motion replans continuous in {@code p/v/a}, but not necessarily in jerk.
     */
    private void changeTrajectory(double pTarget) {
        currentTrajectory =
                factory.create(lastP, pTarget, lastV, lastA, vMax, aMaxAccel, aMaxDecel, jMax);
        startTime = clock.getAsLong();
        pendingTarget = Double.NaN;
    }

    /**
     * Sample the trajectory at the current time and cache position/velocity/acceleration. Call once
     * per control loop.
     */
    public void update() {
        double seconds = (clock.getAsLong() - startTime) / 1e9;

        lastP = currentTrajectory.getPosition(seconds);
        lastV = currentTrajectory.getVelocity(seconds);
        lastA = currentTrajectory.getAcceleration(seconds);

        telemetry.addData("trajectory position", "%.3f", lastP);
        telemetry.addData("trajectory velocity", "%.3f", lastV);
        telemetry.addData("trajectory acceleration", "%.1f", lastA);

        if (!Double.isNaN(pendingTarget)) {
            changeTrajectory(pendingTarget);
        }
    }

    /**
     * Discard the current trajectory and restart from a directly measured state. Acceleration is
     * assumed zero.
     */
    public void resetFromMeasurement(double measuredP, double measuredV) {
        resetFromMeasurement(measuredP, measuredV, 0.0);
    }

    /**
     * Discard the current trajectory and restart from directly measured position, velocity, and
     * acceleration.
     */
    public void resetFromMeasurement(double measuredP, double measuredV, double measuredA) {
        this.lastP = measuredP;
        this.lastV = measuredV;
        this.lastA = measuredA;
        this.pendingTarget = Double.NaN;
        changeTrajectory(targetPosition.get());
    }

    /**
     * Command a new target position.
     *
     * <p>If the target changes while a trajectory is already in progress, the next plan begins
     * from the sampled current {@code p/v/a} state. That preserves position, velocity, and
     * acceleration continuity across the replan, while still allowing the new trajectory to choose
     * its own jerk profile from that state.
     */
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

    public PositionTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }
}
