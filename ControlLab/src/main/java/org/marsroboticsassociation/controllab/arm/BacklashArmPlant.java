package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim;
import org.marsroboticsassociation.controllib.sim.EncoderSim;

/**
 * {@link ArmPlant} backed by the two-inertia {@link BacklashArmMotorSim}. The motor and load are
 * split across a gearbox dead band, so the motor-side link leads or lags the load across the lash
 * and {@link #isEngaged()} flips as the teeth separate and re-contact.
 */
class BacklashArmPlant implements ArmPlant {

    private final ArmPlantConfig cfg;
    private BacklashArmMotorSim sim;

    BacklashArmPlant(ArmPlantConfig cfg, double initialAngleRad) {
        this.cfg = cfg;
        rebuild(initialAngleRad);
    }

    private void rebuild(double initialAngleRad) {
        sim = new BacklashArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                cfg.ticksPerRev, cfg.gearRatio,
                cfg.encoderZeroOffsetRad, cfg.minAngleRad, cfg.maxAngleRad,
                initialAngleRad, cfg.backlashRad);
        applyLiveParams();
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
    @Override public double getMotorPositionRad()      { return sim.getMotorPositionRad(); }
    @Override public boolean isEngaged()               { return sim.isEngaged(); }
    @Override public double getBacklashRad()           { return sim.getBacklashRad(); }
    @Override public double restComplianceRadPerVolt() { return sim.getRestComplianceRadPerVolt(); }

    @Override public void seedFrom(double loadRad, double loadVel) {
        rebuild(loadRad);
    }

    @Override public void applyLiveParams() {
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage);
        sim.setContactStiffness(cfg.contactStiffness);
        sim.setContactDamping(cfg.contactDamping);
        sim.setLoadFriction(cfg.loadViscousFriction, cfg.loadStaticFriction);
    }
}
