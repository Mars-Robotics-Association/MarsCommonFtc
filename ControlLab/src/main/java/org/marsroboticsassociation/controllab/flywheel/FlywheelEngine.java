package org.marsroboticsassociation.controllab.flywheel;

import org.marsroboticsassociation.controllib.control.FlywheelSimple;
import org.marsroboticsassociation.controllib.control.FlywheelStateSpace;
import org.marsroboticsassociation.controllib.control.VelocityMotorPF;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Simulation engine that drives a flywheel motor sim and its controller.
 */
public class FlywheelEngine implements IMotor {

    static final double NOMINAL_DT = 0.016; // 16 ms
    private final java.util.Random random = new java.util.Random();

    private FlywheelControllerType type;
    private FlywheelMotorSim sim;

    // Controllers
    private FlywheelSimple simple;
    private VelocityMotorPF pf;
    private VelocityMotorPF.VelocityMotorPFConfig pfConfig;
    private FlywheelStateSpace ss;

    // Controller Params
    private double kV = 12.5 / 2632.1;
    private double kA = 12.5 / 2087.9;
    private double kS = 0.8931;
    private double kP = 0.010;
    private double velLpfCutoffHz = 4.0;

    // Physical Plant Params (the "Real Robot")
    private double plantKV = 12.5 / 2632.1;
    private double plantKA = 12.5 / 2087.9;
    private double plantKS = 0.8931;

    private double targetTps = 0;
    private double currentPower = 0;
    private long elapsedNanos = 0;
    private double elapsedSec = 0;

    private final TelemetryAddData noOp = (c, f, v) -> {};

    public FlywheelEngine(FlywheelControllerType type) {
        this.type = type;
        this.sim = new FlywheelMotorSim(plantKV, plantKA);
        this.sim.setDisturbanceVoltage(-plantKS);
        rebuildController();
    }

    public void setParams(double kV, double kA, double kS, double kP, double velLpfCutoffHz) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kP = kP;
        this.velLpfCutoffHz = velLpfCutoffHz;

        // Apply to existing controllers if possible
        if (simple != null) {
            FlywheelSimple.PARAMS.kV = kV;
            FlywheelSimple.PARAMS.kA = kA;
            FlywheelSimple.PARAMS.kS = kS;
            FlywheelSimple.PARAMS.kP = kP;
            FlywheelSimple.PARAMS.velLpfCutoffHz = velLpfCutoffHz;
        }
        if (pf != null) {
            pfConfig.kV = kV;
            pfConfig.kA = kA;
            pfConfig.kS = kS;
            pfConfig.kP = kP;
            pfConfig.measurementLpfCutoffHz = velLpfCutoffHz;
        }
        if (ss != null) {
            FlywheelStateSpace.PARAMS.kV = kV * 28 / (2 * Math.PI); // Convert to rad/s
            FlywheelStateSpace.PARAMS.kA = Math.max(kA, 1e-6) * 28 / (2 * Math.PI);
            rebuildController(); // StateSpace plant is defined at construction, so we rebuild.
        }
    }

    public void setPlantParams(double plantKV, double plantKA, double plantKS) {
        this.plantKV = plantKV;
        this.plantKA = plantKA;
        this.plantKS = plantKS;
        // Update the physical simulation plant
        this.sim = new FlywheelMotorSim(plantKV, Math.max(plantKA, 1e-6), sim.getPositionTicks(), sim.getTrueVelocityTps());
        this.sim.setDisturbanceVoltage(-plantKS);
    }

    public double getPlantKV() { return plantKV; }
    public double getPlantKA() { return plantKA; }
    public double getPlantKS() { return plantKS; }

    private void rebuildController() {
        switch (type) {
            case FLYWHEEL_SIMPLE:
                FlywheelSimple.PARAMS.kV = kV;
                FlywheelSimple.PARAMS.kA = kA;
                FlywheelSimple.PARAMS.kS = kS;
                FlywheelSimple.PARAMS.kP = kP;
                FlywheelSimple.PARAMS.velLpfCutoffHz = velLpfCutoffHz;
                simple = new FlywheelSimple(noOp, () -> elapsedNanos, this);
                pf = null;
                pfConfig = null;
                ss = null;
                break;
            case VELOCITY_MOTOR_PF:
                pfConfig = new VelocityMotorPF.VelocityMotorPFConfig();
                pfConfig.kV = kV;
                pfConfig.kA = kA;
                pfConfig.kS = kS;
                pfConfig.kP = kP;
                pfConfig.measurementLpfCutoffHz = velLpfCutoffHz;
                pf = new VelocityMotorPF(noOp, 1.0, 28, 0.01, pfConfig, this, () -> elapsedNanos);
                simple = null;
                ss = null;
                break;
            case FLYWHEEL_STATE_SPACE:
                FlywheelStateSpace.PARAMS.kV = kV * 28 / (2 * Math.PI);
                FlywheelStateSpace.PARAMS.kA = Math.max(kA, 1e-6) * 28 / (2 * Math.PI);
                // Note: kS and kP aren't directly used by SS in the same way, but let's assume kV/kA represent the plant.
                ss = new FlywheelStateSpace(this, noOp);
                simple = null;
                pf = null;
                pfConfig = null;
                break;
        }
    }

    public void setType(FlywheelControllerType type) {
        if (this.type == type) return;
        this.type = type;
        rebuildController();
        setTarget(targetTps);
    }

    public void setTarget(double tps) {
        this.targetTps = tps;
        if (simple != null) simple.setTps(tps);
        if (pf != null) pf.setTPS(tps);
        if (ss != null) ss.setTps(tps);
    }

    public void tick() {
        // Simulate real-world jitter: 16ms +/- 4ms
        double dt = NOMINAL_DT + (random.nextDouble() - 0.5) * 0.008;
        elapsedNanos += (long) (dt * 1e9);

        // 1. Controller update (uses simulated motor readings)
        if (simple != null) simple.update();
        if (pf != null) pf.update(dt);
        if (ss != null) ss.update(dt, 12.0);

        // 2. Sim plant update
        sim.step(dt, currentPower, 12.0);
        elapsedSec += dt;
    }

    // --- IMotor implementation ---

    @Override
    public double getVelocity() {
        return sim.getVelocityTps();
    }

    @Override
    public void setPower(double power) {
        this.currentPower = power;
    }

    @Override
    public double getHubVoltage() {
        return 12.0;
    }

    @Override
    public String getName() {
        return "flywheel";
    }

    @Override
    public void setVelocity(double tps) {
        setTarget(tps);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        // Not used in this sim
    }

    public int getPositionTicks() {
        return sim.getPositionTicks();
    }

    // --- Accessors for chart ---

    public double getMeasuredVelocity() {
        return sim.getVelocityTps();
    }

    public double getTrueVelocity() {
        return sim.getTrueVelocityTps();
    }

    public double getProfiledVelocity() {
        if (simple != null) return simple.getProfiledVelocity();
        if (pf != null) return pf.getProfiledVelocity();
        if (ss != null) return targetTps; // SS doesn't have a motion profile in this version
        return 0;
    }

    public double getFilteredVelocity() {
        if (simple != null) return simple.getFilteredVelocity();
        if (pf != null) return pf.getTpsFiltered();
        if (ss != null) return ss.getEstimatedTps();
        return 0;
    }

    public double getPower() {
        return currentPower;
    }

    public double getTarget() {
        return targetTps;
    }

    public double getElapsedSec() {
        return elapsedSec;
    }

    public double getKV() { return kV; }
    public double getKA() { return kA; }
    public double getKS() { return kS; }
    public double getKP() { return kP; }
    public double getVelLpfCutoffHz() { return velLpfCutoffHz; }

    public void reset() {
        sim.reset(0);
        elapsedSec = 0;
        elapsedNanos = 0;
        if (simple != null) simple.setTps(0);
        if (pf != null) pf.stop();
        if (ss != null) ss.setTps(0);
    }
}
