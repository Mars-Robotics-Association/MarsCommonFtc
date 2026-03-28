package org.marsroboticsassociation.controllab.flywheel;

import org.marsroboticsassociation.controllib.control.FlywheelSimple;
import org.marsroboticsassociation.controllib.control.FlywheelStateSpace;
import org.marsroboticsassociation.controllib.control.VelocityMotorPF;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.motion.SCurveVelocity;
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

    // FlywheelSimple-specific params (NaN = use PARAMS default)
    private double simpleMaxAccel = Double.NaN;

    // VelocityMotorPF-specific params (NaN = use config default)
    private double pfAccelMax = Double.NaN;
    private double pfJerkIncreasing = Double.NaN;
    private double pfJerkDecreasing = Double.NaN;

    // FlywheelStateSpace-specific params
    private double ssModelStdDev = FlywheelStateSpace.PARAMS.modelStdDevRadPerSec;
    private double ssMeasurementStdDev = FlywheelStateSpace.PARAMS.measurementStdDevRadPerSec;

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
                if (!Double.isNaN(simpleMaxAccel)) FlywheelSimple.PARAMS.maxAccel = simpleMaxAccel;
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
                if (!Double.isNaN(pfAccelMax)) pfConfig.accelMax = pfAccelMax;
                if (!Double.isNaN(pfJerkIncreasing)) pfConfig.jerkIncreasing = pfJerkIncreasing;
                if (!Double.isNaN(pfJerkDecreasing)) pfConfig.jerkDecreasing = pfJerkDecreasing;
                pf = new VelocityMotorPF(noOp, 1.0, 28, 0.01, pfConfig, this, () -> elapsedNanos);
                simple = null;
                ss = null;
                break;
            case FLYWHEEL_STATE_SPACE:
                FlywheelStateSpace.PARAMS.kV = kV * 28 / (2 * Math.PI);
                FlywheelStateSpace.PARAMS.kA = Math.max(kA, 1e-6) * 28 / (2 * Math.PI);
                FlywheelStateSpace.PARAMS.modelStdDevRadPerSec = ssModelStdDev;
                FlywheelStateSpace.PARAMS.measurementStdDevRadPerSec = ssMeasurementStdDev;
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

    public void setSimpleParams(double maxAccel) {
        if (!Double.isNaN(maxAccel)) this.simpleMaxAccel = maxAccel;
        if (simple != null && !Double.isNaN(this.simpleMaxAccel)) {
            FlywheelSimple.PARAMS.maxAccel = this.simpleMaxAccel;
        }
    }

    public double getSimpleMaxAccel() {
        return simple != null ? FlywheelSimple.PARAMS.maxAccel : simpleMaxAccel;
    }

    public void setPFParams(double accelMax, double jerkIncreasing, double jerkDecreasing) {
        this.pfAccelMax = accelMax;
        this.pfJerkIncreasing = jerkIncreasing;
        this.pfJerkDecreasing = jerkDecreasing;
        if (pf != null && pfConfig != null) {
            pfConfig.accelMax = accelMax;
            pfConfig.jerkIncreasing = jerkIncreasing;
            pfConfig.jerkDecreasing = jerkDecreasing;
        }
    }

    public double getPFBAccelMax() {
        return pf != null ? pfConfig.accelMax : (!Double.isNaN(pfAccelMax) ? pfAccelMax : 2500.0);
    }

    public double getPFJerkIncreasing() {
        return pf != null ? pfConfig.jerkIncreasing : (!Double.isNaN(pfJerkIncreasing) ? pfJerkIncreasing : 2000.0);
    }

    public double getPFJerkDecreasing() {
        return pf != null ? pfConfig.jerkDecreasing : (!Double.isNaN(pfJerkDecreasing) ? pfJerkDecreasing : 1000.0);
    }

    public void setSSParams(double modelStdDev, double measurementStdDev) {
        this.ssModelStdDev = modelStdDev;
        this.ssMeasurementStdDev = measurementStdDev;
        if (ss != null) {
            FlywheelStateSpace.PARAMS.modelStdDevRadPerSec = modelStdDev;
            FlywheelStateSpace.PARAMS.measurementStdDevRadPerSec = measurementStdDev;
            rebuildController();
            ss.setTps(targetTps);
        }
    }

    public double getSSModelStdDev() { return ssModelStdDev; }
    public double getSSMeasurementStdDev() { return ssMeasurementStdDev; }

    public void reset() {
        sim.reset(0);
        elapsedSec = 0;
        elapsedNanos = 0;
        if (simple != null) simple.setTps(0);
        if (pf != null) pf.stop();
        if (ss != null) ss.setTps(0);
    }

    /**
     * Randomize plant, zero tuning params, auto-tune profile, reset sim.
     * @param targetA the "A" target velocity for profile auto-tuning
     * @param targetB the "B" target velocity for profile auto-tuning
     */
    public void newChallenge(double targetA, double targetB) {
        // 1. Randomize plant params (FTC-reasonable ranges)
        //    kV: 12.5/3000 .. 12.5/1000  (different motor speeds)
        //    kA: 12.5/4000 .. 12.5/800   (different flywheel inertias)
        //    kS: 0.3 .. 1.5              (different friction)
        double maxTps = 1000 + random.nextDouble() * 2000; // 1000..3000 TPS
        this.plantKV = 12.5 / maxTps;
        double accelDenom = 800 + random.nextDouble() * 3200; // 800..4000
        this.plantKA = 12.5 / accelDenom;
        this.plantKS = 0.3 + random.nextDouble() * 1.2; // 0.3..1.5 V

        // 2. Rebuild sim with new plant
        this.sim = new FlywheelMotorSim(plantKV, plantKA);
        this.sim.setDisturbanceVoltage(-plantKS);

        // 3. Zero tuning params (leave cutoff alone)
        this.kS = 0;
        this.kV = 0;
        this.kA = 0;
        this.kP = 0;

        // 4. Auto-tune profile params from new plant
        double v0 = Math.min(targetA, targetB);
        double v1 = Math.max(targetA, targetB);
        double jInc = Double.isNaN(pfJerkIncreasing) ? 2000 : pfJerkIncreasing;
        double voltage = 12.0;

        double aMax = SCurveVelocity.findMaxAMax(v0, v1, jInc, voltage, plantKS, plantKV, plantKA);
        double jDec = SCurveVelocity.findMaxJDec(v0, v1, 0, aMax, jInc, voltage, plantKS, plantKV, plantKA);

        this.pfAccelMax = aMax;
        this.pfJerkDecreasing = jDec;

        // 5. Reset sim and rebuild controller with zeroed params
        this.elapsedSec = 0;
        this.elapsedNanos = 0;
        this.currentPower = 0;
        rebuildController();
        setTarget(0); // propagate zero target to newly built controller
    }
}
