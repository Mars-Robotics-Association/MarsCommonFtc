package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.sim.ArmMotorSim;
import org.marsroboticsassociation.controllib.sim.EncoderSim;

/**
 * {@link ArmPlant} backed by the rigid single-inertia {@link ArmMotorSim}. The motor and load are
 * one body, so the motor-side link tracks the load exactly and the teeth are always "engaged".
 */
class RigidArmPlant implements ArmPlant {

    private final ArmPlantConfig cfg;
    private ArmMotorSim sim;

    RigidArmPlant(ArmPlantConfig cfg, double initialAngleRad) {
        this.cfg = cfg;
        rebuild(initialAngleRad);
    }

    private void rebuild(double initialAngleRad) {
        sim = new ArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                cfg.ticksPerRev, cfg.gearRatio,
                cfg.encoderZeroOffsetRad, cfg.minAngleRad, cfg.maxAngleRad,
                initialAngleRad);
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage);
        applyEncoder();
    }

    private void applyEncoder() {
        switch (cfg.encoderKind) {
            case CONTROL_HUB:
                sim.setEncoder(EncoderSim.controlHub(cfg.encoderSeed));
                break;
            case EXPANSION_HUB:
                sim.setEncoder(EncoderSim.expansionHub(cfg.encoderSeed));
                break;
            case NONE:
            default:
                sim.setEncoder(new EncoderSim());
                break;
        }
    }

    @Override public void step(double dt, double power, double hubVoltage) {
        sim.step(dt, power, hubVoltage);
    }

    @Override public int getPositionTicks()            { return sim.getPositionTicks(); }
    @Override public double getVelocityTps()           { return sim.getVelocityTps(); }
    @Override public double getTruePositionRad()       { return sim.getTruePositionRad(); }
    @Override public double getTrueVelocityRadPerSec() { return sim.getTrueVelocityRadPerSec(); }
    // Rigid: the motor side is the load side.
    @Override public double getMotorPositionRad()      { return sim.getTruePositionRad(); }
    @Override public boolean isEngaged()               { return true; }
    @Override public double getBacklashRad()           { return 0.0; }

    @Override public void seedFrom(double loadRad, double loadVel) {
        rebuild(loadRad);
    }

    @Override public void applyLiveParams() {
        // Only the disturbance is settable in place on the rigid sim.
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage);
    }
}
