package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.sim.EncoderSim;
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim;

/**
 * {@link ArmPlant} backed by the three-inertia {@link FlexArmMotorSim}: motor and hub split across
 * the gearbox dead band (like the backlash plant), plus a lightly damped structural flex spring
 * between the hub and the tip. The tip is reported as the true load — it is what visibly bounces on
 * the way down.
 */
class FlexArmPlant implements ArmPlant {

    private final ArmPlantConfig cfg;
    private FlexArmMotorSim sim;

    FlexArmPlant(ArmPlantConfig cfg, double initialAngleRad) {
        this.cfg = cfg;
        rebuild(initialAngleRad);
    }

    private void rebuild(double initialAngleRad) {
        sim = new FlexArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                cfg.ticksPerRev, cfg.gearRatio,
                cfg.encoderZeroOffsetRad, cfg.minAngleRad, cfg.maxAngleRad,
                initialAngleRad, cfg.backlashRad, cfg.flexHz, cfg.flexZeta);
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

    @Override public void seedFrom(double loadRad, double loadVel) {
        rebuild(loadRad);
    }

    @Override public void applyLiveParams() {
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage);
        sim.setContactStiffness(cfg.contactStiffness);
        sim.setContactDamping(cfg.contactDamping);
        sim.setLoadFriction(cfg.loadViscousFriction, cfg.loadStaticFriction);
        sim.setFlex(cfg.flexHz, cfg.flexZeta);
    }
}
