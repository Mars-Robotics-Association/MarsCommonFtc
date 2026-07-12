package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.control.ArmController;
import org.marsroboticsassociation.controllib.control.VerticalArmController;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import java.util.function.LongSupplier;

/**
 * Lineage A adapter: wraps {@link ArmController} (PD) or {@link VerticalArmController} (LQR), both of
 * which drive an {@link IMotor} port via {@code void update(dt)}. An inner {@code IMotor} view reads
 * the current {@link ArmPlant}'s encoder and stashes the controller's commanded power.
 *
 * <p>The wrapped controllers seed their Kalman + trajectory from {@code motor.getPosition()} at
 * construction, so rebuilding this adapter automatically reseeds from the current arm pose (no jump
 * home). Because those controllers snap their estimate to the nearest hard stop the first time
 * {@code setTarget} is called while coasting, the target after a rebuild is re-applied via a latch
 * <em>after</em> the next {@code update()} — once the controller has established TRACKING mode
 * mid-range, so a rebuild that happens away from a hard stop does not cause a snap.
 */
class LineageAArmAdapter implements ArmControlAdapter {

    private static final TelemetryAddData NO_OP = (caption, format, value) -> {};

    private final ArmControllerType type;
    private final LongSupplier clock;
    private final double hubVoltageNominal;

    private ArmPlant plant;
    private double lastPower = 0.0;

    private ArmController armPd;
    private VerticalArmController armLqr;

    private double targetRad;
    private boolean hasTarget = false;
    private boolean reapplyTargetPending = false;

    /** IMotor view over whatever plant is currently installed. */
    private final IMotor motorView = new IMotor() {
        @Override public String getName()               { return "arm"; }
        @Override public int    getPosition()           { return plant.getPositionTicks(); }
        @Override public double getVelocity()           { return plant.getVelocityTps(); }
        @Override public void   setPower(double power)  { lastPower = power; }
        @Override public double getHubVoltage()         { return hubVoltageNominal; }
        @Override public void   setVelocity(double tps) {}
        @Override public void   setVelocityPIDFCoefficients(double p, double i, double d, double f) {}
    };

    LineageAArmAdapter(ArmControllerType type, ArmPlant plant, LongSupplier clock,
                       double hubVoltageNominal) {
        if (type != ArmControllerType.ARM_PD && type != ArmControllerType.ARM_LQR) {
            throw new IllegalArgumentException("LineageAArmAdapter handles ARM_PD/ARM_LQR, got " + type);
        }
        this.type = type;
        this.plant = plant;
        this.clock = clock;
        this.hubVoltageNominal = hubVoltageNominal;
        build();
    }

    /** (Re)construct the wrapped controller, reseeding its estimate from the current plant pose. */
    private void build() {
        if (type == ArmControllerType.ARM_PD) {
            armPd = new ArmController(motorView, NO_OP, clock);
            armLqr = null;
        } else {
            armLqr = new VerticalArmController(motorView, NO_OP, clock);
            armPd = null;
        }
    }

    /** Rebuild after a gain edit: reseed pose from the plant, re-apply the target after next update. */
    void rebuild() {
        build();
        if (hasTarget) reapplyTargetPending = true;
    }

    /**
     * Queue a target to be applied after the next {@code update()} rather than immediately, so a
     * freshly built controller establishes TRACKING mode mid-range before {@code setTarget} runs and
     * avoids the wake-from-coast hard-stop snap. Used by the engine right after (re)building.
     */
    void deferTarget(double rad) {
        targetRad = rad;
        hasTarget = true;
        reapplyTargetPending = true;
    }

    @Override public void setPlant(ArmPlant plant) {
        this.plant = plant;
    }

    @Override public void setTargetRad(double rad) {
        targetRad = rad;
        hasTarget = true;
        reapplyTargetPending = false;
        if (armPd != null) armPd.setTarget(rad, hubVoltageNominal);
        else armLqr.setTarget(rad, hubVoltageNominal);
    }

    @Override public void step(double dt, double hubVoltage) {
        if (armPd != null) armPd.update(dt, hubVoltage);
        else armLqr.update(dt, hubVoltage);

        // Re-apply a target queued by a rebuild, now that the controller has run one cycle and
        // established its mode (TRACKING mid-range, avoiding the wake-from-coast hard-stop snap).
        if (reapplyTargetPending && hasTarget) {
            reapplyTargetPending = false;
            if (armPd != null) armPd.setTarget(targetRad, hubVoltageNominal);
            else armLqr.setTarget(targetRad, hubVoltageNominal);
        }
    }

    @Override public double commandedPower() { return lastPower; }

    @Override public double estimatedPosRad() {
        return armPd != null ? armPd.getEstimatedPositionRad() : armLqr.getEstimatedPositionRad();
    }

    @Override public double estimatedVelRad() {
        return armPd != null ? armPd.getEstimatedVelocityRadPerSec()
                             : armLqr.getEstimatedVelocityRadPerSec();
    }

    @Override public double trajPosRad() {
        return armPd != null ? armPd.getTrajectoryPositionRad() : armLqr.getTrajectoryPositionRad();
    }

    @Override public double trajVelRad() {
        return armPd != null ? armPd.getTrajectoryVelocityRadPerSec()
                             : armLqr.getTrajectoryVelocityRadPerSec();
    }

    @Override public String modeLabel() {
        String mode = armPd != null ? armPd.getMode().name() : armLqr.getMode().name();
        return type.name() + " / " + mode;
    }
}
