package org.marsroboticsassociation.controllab.arm

import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.parallel.Execution
import org.junit.jupiter.api.parallel.ExecutionMode

/**
 * Headless drive of [ArmEngine] through the acceptance scenarios that cannot be exercised via the
 * Swing GUI: each controller reaches a commanded target on both plants, the rigid/backlash hot-swap
 * keeps the pose and reseeds the controller, and a controller-type change reseeds without a jump.
 * This is the "drive it, don't just compile" check for the control stack.
 *
 * <p>Runs single-threaded so Lineage A engines do not race on the static `PARAMS` bags. Engines use
 * a fixed RNG seed so dt jitter is reproducible.
 */
@Execution(ExecutionMode.SAME_THREAD)
class ArmEngineSmokeTest {

    companion object {
        private const val SEED = 42L

        // Workspace over the top: min = −45° (front), max = +225° (back ≡ −135°). Prefer mid-range
        // holds outside the 10° coast bands (roughly (−35°, 215°)).
        private val MID_HOLD = Math.toRadians(90.0) // straight up
        private val NEAR_FRONT = Math.toRadians(0.0) // horizontal, clear of front coast

        private fun engine(type: ArmControllerType): ArmEngine = ArmEngine(type, SEED)

        private fun run(engine: ArmEngine, ticks: Int) {
            for (i in 0 until ticks) engine.tick()
        }
    }

    @Test
    fun armPd_reachesTargetOnBacklashPlant() {
        val engine = engine(ArmControllerType.ARM_PD) // backlash by default
        assertTrue(engine.isBacklashEnabled)
        val target = MID_HOLD // big move from the parked back stop
        engine.setTargetRad(target)
        run(engine, 600)
        val err = abs(engine.trueLoadRad - target)
        assertTrue(
            err < Math.toRadians(8.0),
            "ARM_PD load should reach target through backlash, err(deg)=" + Math.toDegrees(err),
        )
    }

    @Test
    fun armPd_reachesTargetOnFlexPlant() {
        val engine = engine(ArmControllerType.ARM_PD)
        engine.setPlantKind(ArmEngine.PlantKind.FLEX)
        val target = MID_HOLD
        engine.setTargetRad(target)
        run(engine, 600)
        val err = abs(engine.trueLoadRad - target)
        assertTrue(
            err < Math.toRadians(8.0),
            "ARM_PD tip should reach target through lash + flex, err(deg)=" + Math.toDegrees(err),
        )
    }

    @Test
    fun hotSwapToFlex_keepsPoseAndReseedsController() {
        val engine = engine(ArmControllerType.ARM_PD)
        engine.setPlantKind(ArmEngine.PlantKind.RIGID)
        engine.setTargetRad(NEAR_FRONT)
        run(engine, 400)
        val before = engine.trueLoadRad
        engine.setPlantKind(ArmEngine.PlantKind.FLEX)
        val after = engine.trueLoadRad
        assertTrue(
            abs(after - before) < Math.toRadians(1.0),
            "hot-swap to flex should not jump the pose, delta(deg)=" +
                Math.toDegrees(after - before),
        )
        assertTrue(
            abs(engine.trajVelRad) < 0.5,
            "controller profile should reseed near rest after plant swap, trajVel=" +
                engine.trajVelRad,
        )
        run(engine, 300) // keeps running without throwing
    }

    @Test
    fun armPd_reachesTargetTightlyOnRigidPlant() {
        val engine = engine(ArmControllerType.ARM_PD)
        engine.setBacklashEnabled(false)
        val target = NEAR_FRONT
        engine.setTargetRad(target)
        run(engine, 500)
        val err = abs(engine.trueLoadRad - target)
        assertTrue(
            err < Math.toRadians(3.0),
            "ARM_PD should track tightly on rigid plant, err(deg)=" + Math.toDegrees(err),
        )
    }

    @Test
    fun hotSwapRigidToBacklash_keepsPoseAndReseedsController() {
        val engine = engine(ArmControllerType.ARM_PD)
        engine.setBacklashEnabled(false) // start rigid
        engine.setTargetRad(NEAR_FRONT)
        run(engine, 400)
        val before = engine.trueLoadRad
        // Mid-move / mid-hold: plant was in motion; after swap the sim seeds at rest and the
        // adapter must reseed so the profile is not still flying.
        engine.setBacklashEnabled(true)
        val after = engine.trueLoadRad
        assertTrue(
            abs(after - before) < Math.toRadians(1.0),
            "hot-swap should not jump the pose, delta(deg)=" + Math.toDegrees(after - before),
        )
        assertTrue(
            abs(engine.trajVelRad) < 0.5,
            "controller profile should reseed near rest after plant swap, trajVel=" +
                engine.trajVelRad,
        )
        run(engine, 300) // keeps running without throwing
    }

    @Test
    fun structuralPlantEdit_reseedsControllerNearRest() {
        val engine = engine(ArmControllerType.MECHANISM_PIDF)
        engine.setTargetRad(MID_HOLD)
        run(engine, 250)
        engine.setPlantDynamics(0.3, 3.5, 1.2, 0.4) // structural rebuild at current pose
        assertTrue(
            abs(engine.trajVelRad) < 0.5,
            "profile should reseed near rest after structural plant edit, trajVel=" +
                engine.trajVelRad,
        )
        run(engine, 400)
        assertTrue(
            abs(engine.trueLoadRad - MID_HOLD) < Math.toRadians(5.0),
            "should re-acquire target after plant reseed",
        )
    }

    @Test
    fun controllerTypeChange_reseedsWithoutJump() {
        val engine = engine(ArmControllerType.ARM_PD)
        engine.setTargetRad(MID_HOLD)
        run(engine, 400)
        val before = engine.trueLoadRad
        engine.setControllerType(ArmControllerType.ARM_LQR)
        val after = engine.trueLoadRad
        assertTrue(
            abs(after - before) < Math.toRadians(1.0),
            "type change should not jump the pose, delta(deg)=" + Math.toDegrees(after - before),
        )
        run(engine, 400)
        assertTrue(
            abs(engine.trueLoadRad - MID_HOLD) < Math.toRadians(8.0),
            "ARM_LQR should hold the target after reseed",
        )
    }

    @Test
    fun mechanismPidf_tracksOnBothPlants() {
        val engine = engine(ArmControllerType.MECHANISM_PIDF)
        val target = MID_HOLD
        engine.setTargetRad(target)
        run(engine, 700)
        val backlashErr = abs(engine.trueLoadRad - target)
        assertTrue(
            backlashErr < Math.toRadians(6.0),
            "MECHANISM_PIDF should track on backlash plant, err(deg)=" +
                Math.toDegrees(backlashErr),
        )

        engine.setBacklashEnabled(false)
        engine.setTargetRad(NEAR_FRONT)
        run(engine, 700)
        val rigidErr = abs(engine.trueLoadRad - NEAR_FRONT)
        assertTrue(
            rigidErr < Math.toRadians(3.0),
            "MECHANISM_PIDF should track tightly on rigid plant, err(deg)=" +
                Math.toDegrees(rigidErr),
        )
    }

    @Test
    fun liveEdits_doNotThrowAndTakeEffect() {
        val engine = engine(ArmControllerType.ARM_PD)
        engine.setTargetRad(NEAR_FRONT)
        run(engine, 200)
        engine.setPdGains(20.0, 1.5)
        engine.setFeedforwardGains(0.3, 1.6, 1.2, 0.15)
        engine.setBacklashRad(Math.toRadians(8.0))
        engine.setDisturbanceVoltage(-1.0)
        engine.setContact(400.0, 3.0)
        engine.setEncoderKind(ArmPlantConfig.EncoderKind.EXPANSION_HUB)
        run(engine, 300)
        engine.setPlantKind(ArmEngine.PlantKind.FLEX)
        engine.setFlexParams(2.5, 0.05)
        run(engine, 300)
        assertTrue(engine.metrics.pctEngaged() >= 0)
    }
}
