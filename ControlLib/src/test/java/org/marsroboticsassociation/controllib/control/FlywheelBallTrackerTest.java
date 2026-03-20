package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for FlywheelBallTracker's augmented Kalman filter and ball-detection logic.
 *
 * <p>Measurements are injected directly — no motor hardware or plant simulation is required.
 * All scenarios use a fixed dt = 10 ms (100 Hz), matching a typical robot loop period.
 * 10 TPS Gaussian noise is added to every measurement to exercise the filter under realistic
 * encoder jitter (matching the noise level used by the other flywheel test suites).
 *
 * <p>Physics recap: the disturbance state {@code d} represents ball drag in TPS-equivalent
 * units. A sustained measurement drop below the setpoint drives {@code d} positive. When
 * {@code d} exceeds {@code detectionThreshold} (default 50 TPS) the ball is considered engaged;
 * it clears when {@code d} falls below {@code exitThreshold} (default 20 TPS).
 */
class FlywheelBallTrackerTest {

    private static final double DT           = 0.010;   // 10 ms loop period
    private static final double SETPOINT_TPS = 2000.0;  // typical flywheel setpoint
    private static final double NOISE_STD_TPS = 10.0;
    private static final long   SEED          = 42L;

    private VelocityMotorPF.VelocityMotorPFConfig motorConfig;
    private FlywheelBallTracker tracker;
    private Random rng;

    @BeforeEach
    void setUp() {
        // Isolate tests: fresh default config and a new tracker each time.
        FlywheelBallTracker.config = new FlywheelBallTracker.Config();
        motorConfig = new VelocityMotorPF.VelocityMotorPFConfig();
        tracker = new FlywheelBallTracker(motorConfig);
        rng = new Random(SEED);
    }

    /** Feed a constant measurement + noise for {@code steps} iterations at {@link #SETPOINT_TPS}. */
    private void run(double measurementTps, int steps) {
        for (int i = 0; i < steps; i++) {
            double noisy = measurementTps + rng.nextGaussian() * NOISE_STD_TPS;
            tracker.update(noisy, SETPOINT_TPS, DT);
        }
    }

    // ── initial state ─────────────────────────────────────────────────────────

    @Test
    void freshFilter_allStateIsZero() {
        assertEquals(0.0, tracker.getOmegaEst(),    "omega must start at 0");
        assertEquals(0.0, tracker.getDisturbance(), "disturbance must start at 0");
        assertFalse(tracker.isBallDetected(),       "ball must not be detected on a fresh filter");
    }

    // ── steady state, no ball ─────────────────────────────────────────────────

    @Test
    void steadyState_noBall_notDetected() {
        // 2 s of measurement matching the setpoint — no ball drag present.
        run(SETPOINT_TPS, 200);
        assertFalse(tracker.isBallDetected(),
                "ball must not be falsely detected at nominal steady-state velocity");
    }

    @Test
    void steadyState_omegaEstimateConverges() {
        // After 5 s the Kalman omega estimate should closely track the true velocity.
        run(SETPOINT_TPS, 500);
        System.out.printf("steadyState_omegaEstimateConverges: omegaEst=%.1f TPS%n",
                tracker.getOmegaEst());
        assertEquals(SETPOINT_TPS, tracker.getOmegaEst(), 100.0,
                "omega estimate must converge within 100 TPS of the true velocity after 5 s");
    }

    // ── ball detection ────────────────────────────────────────────────────────

    @Test
    void measurementDrop_raisesDAndDetectsBall() {
        // Warm up at steady state so the filter has converged.
        run(SETPOINT_TPS, 200);
        assertFalse(tracker.isBallDetected(), "no ball before injection");

        // Simulate ball drag: measurement drops 500 TPS below the setpoint for 20 steps (200 ms).
        // This drives the disturbance state positive and above detectionThreshold.
        run(SETPOINT_TPS - 500.0, 20);

        System.out.printf("measurementDrop_raisesDAndDetectsBall: d=%.1f TPS%n",
                tracker.getDisturbance());
        assertTrue(tracker.isBallDetected(),
                "ball must be detected after a sustained measurement drop");
        assertTrue(tracker.getDisturbance() > FlywheelBallTracker.config.detectionThreshold,
                "disturbance must exceed detectionThreshold when ball is detected");
    }

    // ── hysteresis ────────────────────────────────────────────────────────────

    @Test
    void hysteresis_detectionPersistsThenClears() {
        // Warm up and inject ball event.
        run(SETPOINT_TPS, 200);
        run(SETPOINT_TPS - 500.0, 20);
        assertTrue(tracker.isBallDetected(), "ball should be detected after injection");

        // Return to normal measurement for 3 steps (~30 ms).
        // d decays by ~8 % per step (tauD = 0.12 s), so it should still be above
        // exitThreshold (20 TPS) — ballDetected must remain latched.
        run(SETPOINT_TPS, 3);
        assertTrue(tracker.isBallDetected(),
                "ballDetected must stay true while disturbance is still above exitThreshold");

        // Allow 300 steps (~3 s) for d to decay fully below exitThreshold.
        run(SETPOINT_TPS, 300);
        System.out.printf("hysteresis_detectionPersistsThenClears: d=%.1f TPS after recovery%n",
                tracker.getDisturbance());
        assertFalse(tracker.isBallDetected(),
                "ballDetected must clear after disturbance decays below exitThreshold");
        assertTrue(tracker.getDisturbance() < FlywheelBallTracker.config.exitThreshold,
                "disturbance must be below exitThreshold when ballDetected is false");
    }

    // ── reset ─────────────────────────────────────────────────────────────────

    @Test
    void reset_clearsAllState() {
        // Build up state with a ball event so reset has something to clear.
        run(SETPOINT_TPS, 200);
        run(SETPOINT_TPS - 500.0, 20);
        assertTrue(tracker.isBallDetected(), "pre-condition: ball must be detected before reset");

        tracker.reset();

        assertEquals(0.0, tracker.getOmegaEst(),    "reset must zero the omega estimate");
        assertEquals(0.0, tracker.getDisturbance(), "reset must zero the disturbance");
        assertFalse(tracker.isBallDetected(),       "reset must clear ball detection");
    }
}
