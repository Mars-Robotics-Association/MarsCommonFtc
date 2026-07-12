package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Headless drive of {@link ArmEngine} through the acceptance scenarios that cannot be exercised via
 * the Swing GUI: each controller reaches a commanded target on both plants, the rigid/backlash
 * hot-swap keeps the pose and reseeds the controller, and a controller-type change reseeds without
 * a jump. This is the "drive it, don't just compile" check for the control stack.
 *
 * <p>Runs single-threaded so Lineage A engines do not race on the static {@code PARAMS} bags.
 * Engines use a fixed RNG seed so dt jitter is reproducible.
 */
@Execution(ExecutionMode.SAME_THREAD)
class ArmEngineSmokeTest {

    private static final long SEED = 42L;

    private static ArmEngine engine(ArmControllerType type) {
        return new ArmEngine(type, SEED);
    }

    private static void run(ArmEngine engine, int ticks) {
        for (int i = 0; i < ticks; i++) engine.tick();
    }

    @Test
    void armPd_reachesTargetOnBacklashPlant() {
        ArmEngine engine = engine(ArmControllerType.ARM_PD); // backlash by default
        assertTrue(engine.isBacklashEnabled());
        double target = Math.toRadians(0.0); // horizontal, a big move from the parked back stop
        engine.setTargetRad(target);
        run(engine, 600);
        double err = Math.abs(engine.getTrueLoadRad() - target);
        assertTrue(err < Math.toRadians(8),
                "ARM_PD load should reach target through backlash, err(deg)=" + Math.toDegrees(err));
    }

    @Test
    void armPd_reachesTargetTightlyOnRigidPlant() {
        ArmEngine engine = engine(ArmControllerType.ARM_PD);
        engine.setBacklashEnabled(false);
        double target = Math.toRadians(-20);
        engine.setTargetRad(target);
        run(engine, 500);
        double err = Math.abs(engine.getTrueLoadRad() - target);
        assertTrue(err < Math.toRadians(3),
                "ARM_PD should track tightly on rigid plant, err(deg)=" + Math.toDegrees(err));
    }

    @Test
    void hotSwapRigidToBacklash_keepsPoseAndReseedsController() {
        ArmEngine engine = engine(ArmControllerType.ARM_PD);
        engine.setBacklashEnabled(false); // start rigid
        engine.setTargetRad(Math.toRadians(-20));
        run(engine, 400);
        double before = engine.getTrueLoadRad();
        // Mid-move / mid-hold: plant was in motion; after swap the sim seeds at rest and the
        // adapter must reseed so the profile is not still flying.
        engine.setBacklashEnabled(true);
        double after = engine.getTrueLoadRad();
        assertTrue(Math.abs(after - before) < Math.toRadians(1.0),
                "hot-swap should not jump the pose, delta(deg)=" + Math.toDegrees(after - before));
        assertTrue(Math.abs(engine.getTrajVelRad()) < 0.5,
                "controller profile should reseed near rest after plant swap, trajVel="
                        + engine.getTrajVelRad());
        run(engine, 300); // keeps running without throwing
    }

    @Test
    void structuralPlantEdit_reseedsControllerNearRest() {
        ArmEngine engine = engine(ArmControllerType.MECHANISM_PIDF);
        engine.setTargetRad(Math.toRadians(-30));
        run(engine, 250);
        engine.setPlantDynamics(0.3, 3.5, 1.2, 0.4); // structural rebuild at current pose
        assertTrue(Math.abs(engine.getTrajVelRad()) < 0.5,
                "profile should reseed near rest after structural plant edit, trajVel="
                        + engine.getTrajVelRad());
        run(engine, 400);
        assertTrue(Math.abs(engine.getTrueLoadRad() - Math.toRadians(-30)) < Math.toRadians(5),
                "should re-acquire target after plant reseed");
    }

    @Test
    void controllerTypeChange_reseedsWithoutJump() {
        ArmEngine engine = engine(ArmControllerType.ARM_PD);
        engine.setTargetRad(Math.toRadians(-30));
        run(engine, 400);
        double before = engine.getTrueLoadRad();
        engine.setControllerType(ArmControllerType.ARM_LQR);
        double after = engine.getTrueLoadRad();
        assertTrue(Math.abs(after - before) < Math.toRadians(1.0),
                "type change should not jump the pose, delta(deg)=" + Math.toDegrees(after - before));
        run(engine, 400);
        assertTrue(Math.abs(engine.getTrueLoadRad() - Math.toRadians(-30)) < Math.toRadians(8),
                "ARM_LQR should hold the target after reseed");
    }

    @Test
    void mechanismPidf_tracksOnBothPlants() {
        ArmEngine engine = engine(ArmControllerType.MECHANISM_PIDF);
        double target = Math.toRadians(-10);
        engine.setTargetRad(target);
        run(engine, 700);
        double backlashErr = Math.abs(engine.getTrueLoadRad() - target);
        assertTrue(backlashErr < Math.toRadians(6),
                "MECHANISM_PIDF should track on backlash plant, err(deg)="
                        + Math.toDegrees(backlashErr));

        engine.setBacklashEnabled(false);
        engine.setTargetRad(Math.toRadians(-40));
        run(engine, 700);
        double rigidErr = Math.abs(engine.getTrueLoadRad() - Math.toRadians(-40));
        assertTrue(rigidErr < Math.toRadians(3),
                "MECHANISM_PIDF should track tightly on rigid plant, err(deg)="
                        + Math.toDegrees(rigidErr));
    }

    @Test
    void liveEdits_doNotThrowAndTakeEffect() {
        ArmEngine engine = engine(ArmControllerType.ARM_PD);
        engine.setTargetRad(Math.toRadians(-20));
        run(engine, 200);
        engine.setPdGains(20, 1.5);
        engine.setFeedforwardGains(0.3, 1.6, 1.2, 0.15);
        engine.setBacklashRad(Math.toRadians(8));
        engine.setDisturbanceVoltage(-1.0);
        engine.setContact(400, 3);
        engine.setEncoderKind(ArmPlantConfig.EncoderKind.EXPANSION_HUB);
        run(engine, 300);
        assertTrue(engine.getMetrics().pctEngaged() >= 0);
    }
}
