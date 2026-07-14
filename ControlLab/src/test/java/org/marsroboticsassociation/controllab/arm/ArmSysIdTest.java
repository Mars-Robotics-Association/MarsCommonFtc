package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The simulated sysid must recover the plant's true feedforward coefficients (especially {@code kA},
 * the inertia the model-based controller needs) from encoder logs alone.
 */
@Execution(ExecutionMode.SAME_THREAD)
class ArmSysIdTest {

    @Test
    void recoversDefaultHeavyArmParams() {
        ArmPlantConfig cfg = new ArmPlantConfig(); // kS=0.3, kG=3.5, kV=1.2, kA=0.35
        ArmSysId.Result r = ArmSysId.characterize(cfg);
        System.out.printf("sysid: kS=%.3f kV=%.3f kA=%.3f kCos=%.3f kSin=%.3f  R2=%.4f  n=%d%n",
                r.kS, r.kV, r.kA, r.kCos, r.kSin, r.rSquared, r.samples);

        assertTrue(r.rSquared > 0.99, "fit should be excellent, R2=" + r.rSquared);
        assertEquals(0.35, r.kA, 0.35 * 0.15, "kA within 15%");
        assertEquals(3.5, r.kCos, 3.5 * 0.10, "kCos (gravity) within 10%");
        assertEquals(1.2, r.kV, 1.2 * 0.15, "kV within 15%");
        assertEquals(0.3, r.kS, 0.20, "kS within 0.2 V");
        assertEquals(0.0, r.kSin, 0.25, "kSin ~ 0 (plant gravity is pure cosine)");
    }

    @Test
    void engineAppliesIdentifiedInertiaToMechanismModel() {
        // The payoff: user cranks the plant inertia the mechanism model doesn't know, then sysid
        // recovers it and applying it corrects the model's kA / gravity.
        ArmEngine e = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 7L);
        e.setPlantDynamics(0.3, 2.8, 1.2, 0.7); // heavier + different gravity than the model default
        ArmSysId.Result r = e.runSysId();
        e.applyIdentifiedGains(r);
        assertEquals(0.7, e.getMechGains().kA, 0.7 * 0.15, "mechanism kA should match the new plant");
        assertEquals(2.8, e.getMechGains().kCos, 2.8 * 0.10, "mechanism gravity should match the plant");
    }

    @Test
    void recoversAModifiedInertia() {
        ArmPlantConfig cfg = new ArmPlantConfig();
        cfg.kA = 0.6;   // user cranked the inertia; sysid must find the new value
        cfg.kG = 2.5;
        ArmSysId.Result r = ArmSysId.characterize(cfg);
        System.out.printf("sysid(kA=0.6,kG=2.5): kA=%.3f kCos=%.3f R2=%.4f%n", r.kA, r.kCos, r.rSquared);
        assertEquals(0.6, r.kA, 0.6 * 0.15, "kA within 15%");
        assertEquals(2.5, r.kCos, 2.5 * 0.10, "kCos within 10%");
    }

    @Test
    void recoversThroughBacklashFromTheMotorEncoder() {
        // A real robot's gearbox has unavoidable lash. Because the runs are steady and
        // unidirectional the teeth stay engaged, so the motor encoder still sees the coupled
        // inertia + gravity and the fit holds — even through a large 12 deg backlash.
        ArmPlantConfig cfg = new ArmPlantConfig(); // kA=0.35, kG=3.5
        cfg.backlashRad = Math.toRadians(12);
        ArmSysId.Result r = ArmSysId.characterize(cfg, /* throughBacklash= */ true);
        System.out.printf("sysid(backlash 12°): kS=%.3f kV=%.3f kA=%.3f kCos=%.3f R2=%.4f%n",
                r.kS, r.kV, r.kA, r.kCos, r.rSquared);
        assertTrue(r.rSquared > 0.99, "fit through backlash should still be excellent, R2=" + r.rSquared);
        assertEquals(0.35, r.kA, 0.35 * 0.15, "kA recovered through backlash within 15%");
        assertEquals(3.5, r.kCos, 3.5 * 0.10, "gravity recovered through backlash within 10%");
    }

    @Test
    void recoversThroughFlexFromTheMotorEncoder() {
        // Arm structural flex on top of the lash. The steady one-directional runs keep the flex
        // spring at its quasi-static deflection, so kV and kA (what the model-based controller
        // needs for damping/inertia) still come out usable. But the encoder measures the MOTOR
        // angle while gravity acts on the tip hanging a flex-deflection (~3 deg here) plus the
        // lash below it: the gravity regressor is angle-shifted, so kCos is underestimated and the
        // direction-dependent part of the shift is misattributed to kS. That bias is real — a
        // robot with a flexy arm biases its sysid the same way — so this asserts the honest
        // envelope rather than tight recovery.
        ArmPlantConfig cfg = new ArmPlantConfig(); // kA=0.35, kG=3.5, flex 3 Hz zeta 0.03
        ArmSysId.Result r = ArmSysId.characterize(cfg, ArmEngine.PlantKind.FLEX);
        System.out.printf("sysid(flex 3Hz): kS=%.3f kV=%.3f kA=%.3f kCos=%.3f R2=%.4f%n",
                r.kS, r.kV, r.kA, r.kCos, r.rSquared);
        assertTrue(r.rSquared > 0.99, "fit through flex should still be strong, R2=" + r.rSquared);
        assertEquals(1.2, r.kV, 1.2 * 0.15, "kV recovered through flex within 15%");
        assertEquals(0.35, r.kA, 0.35 * 0.20, "kA recovered through flex within 20%");
        assertEquals(3.5, r.kCos, 3.5 * 0.25,
                "gravity through flex is underestimated by the tip deflection but stays in the ballpark");
    }

    @Test
    void engineSysIdUsesTheBacklashPlantWhenEnabled() {
        // With backlash on (the default), the engine identifies through the motor-side encoder and
        // still corrects the mechanism model's inertia.
        ArmEngine e = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 11L);
        assertTrue(e.isBacklashEnabled());
        e.setPlantDynamics(0.3, 2.8, 1.2, 0.55);
        e.applyIdentifiedGains(e.runSysId());
        assertEquals(0.55, e.getMechGains().kA, 0.55 * 0.15, "kA identified through backlash");
        assertEquals(2.8, e.getMechGains().kCos, 2.8 * 0.10, "gravity identified through backlash");
    }
}
