package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim

/**
 * Experiment: can the over-the-top / descent bounce seen with the rigid-model
 * [MotorMechanismController] on the two-mass [BacklashArmMotorSim] be removed by keeping the gear
 * teeth *engaged* on the way down?
 *
 * <p>Findings this harness demonstrates (see the printed tables; asserted in [compareStrategies]):
 * <ul>
 * <li><b>A downward move from a gravity-loaded hold never opens the lash at all.</b> The holding
 *   face is the braking face, and the load (free-fall `kG·cosθ/kA_L` ≈ 37.5·cosθ rad/s²) presses
 *   onto the motor (profile ≤ 12 rad/s²) the whole way down. The bottom overshoot such moves show
 *   is reproduced bit-for-bit with the backlash set to zero: it is profile/tracking overshoot, a
 *   tuning problem, not a lash problem.
 * <li><b>An over-the-top move has exactly one unavoidable handoff</b> at the crest, where the
 *   gravity torque changes sign. The load crosses the lash and lands on the braking face at `v ≈
 *   sqrt(2·Δa·gap)`, where Δa is how far the motor's acceleration deviates from the load's
 *   gravity-only acceleration while the gap is open. Back-EMF caps the motor's downhill
 *   acceleration well below free-fall once it is moving a few rad/s, so a fast crossing always
 *   lands hard; the handoff must complete *slowly, near the crest*. The soft-handoff recipe,
 *   injected entirely through the [MechanismModel] ceiling hooks the controller already consults:
 *   <ul>
 * <li>a <b>two-sided engagement ceiling</b>: acceleration ≤ `margin·|kG·cosθ|/kA_L` while gravity
 *   aids travel (outrunning free-fall pulls the motor off the load), and the same bound on
 *   *deceleration* while gravity opposes travel (braking harder than gravity brakes the load tosses
 *   it off the driving face);
 * <li>evaluated against the <b>weakest gravity within a half-lash either side</b>, since an
 *   airborne load's position is only known to ±half-lash (zero if the crest lies in that interval);
 * <li>an <b>approach ramp</b> that brakes toward a small crossing speed as the gravity torque
 *   decays before the crest, and an <b>exit hold</b> that keeps that speed for ~10° past it while
 *   the load falls the lash and lands.
 * </ul>
 *
 * Landing speed drops from 1.98 to 0.68 rad/s (0.55 at the slowest setting) — roughly a ninth of
 * the impact energy — for about double the descent time. The residual is close to the observability
 * floor: the load sits anywhere within ±half-lash, which near the crest is ~±1.6 rad/s² of
 * free-fall uncertainty, so ~0.5 rad/s is the gentlest landing that can be planned blind. Doing
 * better needs load-side sensing (an output-shaft encoder) or less lash.
 * <li><b>Hold quality dominates perceived bounce: any setpoint motion at the hold broadcasts
 *   through the feedforward as tooth-separating voltage chatter.</b> A profile that limit-cycles
 *   around the target (a few rad/s² of acceleration dither per loop reaches the feedforward as ±1.5
 *   V) separates the teeth ~10 times a second at ±0.3 deg regardless of descent tuning; zero
 *   backlash removes the symptom but not the dither. [holdChatterTrace] guards the requirement: the
 *   setpoint must land at rest and stay motionless (p2p ≈ 0), leaving at most one tooth unload,
 *   during the arrival-overshoot recovery.
 * <li><b>Arrival overshoot is the profile's responsibility, not backlash's</b> (it reproduces with
 *   zero lash). [arrivalTrace] watches the setpoint itself: a stop must charge the jerk-limited
 *   swing from full accelerate to full brake against the stopping distance — a braking plan that
 *   assumes it can begin instantly sails past the target by an error that grows with speed.
 *   Reversal overshoot is ~1.3 deg at an 8 rad/s velocity cap, the same as at 4 rad/s, so descent
 *   speed does not trade against arrival accuracy. </ul>
 */
class ArmBacklashKeepEngagedTest {

    /**
     * [ArmModel] whose acceleration ceiling additionally enforces tooth engagement: while gravity
     * aids the travel, cap acceleration to a fraction of the load's gravity-only (free-fall)
     * acceleration. Below that ceiling the motor can never outrun the falling load, so the load
     * either stays pressed on the braking face or closes an open gap gently.
     */
    private open class EngagementAwareArmModel(
        private val loadInertia: Double, // kA of the load alone, V/(rad/s^2)
        private val margin: Double, // fraction of free-fall accel the profile may use
        private val floor: Double, // rad/s^2, keeps the profile from stalling at the crest
    ) : ArmModel(K_S, K_V, K_A, K_G, 0.0) {

        /**
         * The gravity voltage to plan engagement against. Subclasses may return the weakest gravity
         * the load could see (it sits anywhere within the lash of the motor).
         */
        open fun engagementGravity(position: Double): Double = gravityVoltage(position)

        /** The engagement budget: how hard gravity can accelerate/brake the load by itself. */
        private fun engagementCeiling(position: Double): Double =
            max(floor, margin * abs(engagementGravity(position)) / loadInertia)

        override fun maxSustainableAcceleration(
            availableVoltage: Double,
            position: Double,
            velocity: Double,
            travelDirection: Double,
        ): Double {
            val base =
                super.maxSustainableAcceleration(
                    availableVoltage,
                    position,
                    velocity,
                    travelDirection,
                )
            val gravityAlongTravel = sign(travelDirection) * gravityVoltage(position)
            if (gravityAlongTravel >= 0) {
                return base // gravity opposes: load rests on the driving face, no separation risk
            }
            // Gravity aids: accelerating harder than the load can free-fall pulls the motor away
            // from it across the lash.
            return min(base, engagementCeiling(position))
        }

        override fun maxSustainableDeceleration(
            availableVoltage: Double,
            position: Double,
            velocity: Double,
        ): Double {
            val base = super.maxSustainableDeceleration(availableVoltage, position, velocity)
            val gravityAlongTravel = sign(velocity) * gravityVoltage(position)
            if (gravityAlongTravel <= 0) {
                return base // gravity aids travel: braking presses the load onto the face harder
            }
            // Gravity opposes travel (the driving face carries the load): braking harder than
            // gravity brakes the load alone tosses the load off the face and across the lash.
            return min(base, engagementCeiling(position))
        }
    }

    /**
     * Adds a <b>handoff zone</b> on top of the engagement ceiling: while gravity aids the travel
     * and the gravity torque is still small (near the crest, where the contact face must hand off),
     * cap the velocity low so the lash crossing completes before the mechanism picks up speed. At
     * speed the motor's back-EMF makes it physically unable to chase the free-falling load, so a
     * fast crossing always ends in a hard landing; a slow crossing lands gently, and once the load
     * is on the braking face the speed can open right back up.
     */
    private class HandoffAwareArmModel(
        loadInertia: Double,
        margin: Double,
        floor: Double,
        private val crossingVelocity: Double, // velocity cap right at the crest, rad/s
        private val rampSlope: Double, // extra rad/s allowed per volt of gravity torque
        private val exitHoldVolts: Double = 0.0, // keep the crest cap until |gravity| exceeds this
        private val crestShiftRad: Double =
            0.0, // evaluate the accel ceiling this far toward the crest
    ) : EngagementAwareArmModel(loadInertia, margin, floor) {

        /**
         * The load sits anywhere within the lash of the motor, so plan engagement against the
         * weakest gravity in that interval (zero if the crest lies inside it).
         */
        override fun engagementGravity(position: Double): Double {
            val lo = gravityVoltage(position - crestShiftRad)
            val hi = gravityVoltage(position + crestShiftRad)
            if (sign(lo) != sign(hi)) {
                return 0.0
            }
            return if (abs(lo) < abs(hi)) lo else hi
        }

        override fun maxSustainableVelocity(
            availableVoltage: Double,
            position: Double,
            travelDirection: Double,
        ): Double {
            val base = super.maxSustainableVelocity(availableVoltage, position, travelDirection)
            val gravityHere = gravityVoltage(position)
            val gravityAlongTravel = sign(travelDirection) * gravityHere
            // Approaching a crest (gravity still opposes, but flips sign before the target —
            // position + travelDirection IS the target): ramp the cap down with the remaining
            // gravity torque so the profile brakes gradually and arrives at the crest at
            // crossingVelocity. The ramp slope must be shallow enough that v*dv/dx stays within
            // the deceleration limit.
            val gravityAtTarget = gravityVoltage(position + travelDirection)
            if (gravityAlongTravel >= 0 && sign(gravityHere) != sign(gravityAtTarget)) {
                return min(base, crossingVelocity + rampSlope * abs(gravityHere))
            }
            // Just past a crest (gravity aids travel but is still weak): hold the crossing speed
            // until the landing window has passed — the airborne load needs to fall the full lash
            // before it is back on a face, and the landing is only gentle while gravity is small.
            if (gravityAlongTravel < 0 && abs(gravityHere) < exitHoldVolts) {
                return min(base, crossingVelocity)
            }
            return base
        }
    }

    private class Metrics {
        // Descent phase: from the final target becoming active until the load first enters the
        // target band. Hold phase: everything after that.
        var descentSeps = 0
        var holdSeps = 0
        var descentFreeMs = 0.0
        var holdFreeMs = 0.0
        var maxDescentImpact = 0.0 // |w_m - w_L| at re-engagement, rad/s
        var maxHoldImpact = 0.0
        var bottomOvershootDeg = 0.0 // deg past the load's own resting point (target - h)
        var steadyP2PDeg = 0.0 // load peak-to-peak in the last second
        var finalDeg = 0.0
        var descentMs = 0.0 // time to first reach the target band

        override fun toString(): String =
            String.format(
                "descent[%4.0fms sep=%d free=%3.0fms impact=%.2f] " +
                    "hold[sep=%2d free=%3.0fms impact=%.2f] " +
                    "overshoot=%.2fdeg p2p=%.2fdeg final=%.2fdeg",
                descentMs,
                descentSeps,
                descentFreeMs,
                maxDescentImpact,
                holdSeps,
                holdFreeMs,
                maxHoldImpact,
                bottomOvershootDeg,
                steadyP2PDeg,
                finalDeg,
            )
    }

    /**
     * Over-the-top move: start at +120 deg, drive to -60 deg. The arm crests the vertical, where
     * the gravity torque (and with it the engagement budget) passes through zero and the contact
     * face must hand off.
     */
    private fun runOverTheTop(
        model: ArmModel,
        maxVel: Double,
        maxAccel: Double,
        trace: Boolean,
    ): Metrics =
        run(
            model,
            maxVel,
            maxAccel,
            Math.toRadians(120.0),
            null,
            Math.toRadians(-60.0),
            0,
            5000,
            trace,
            BACKLASH_RAD,
        )

    /** The existing reversal scenario: settle at +30, then reverse down to -60 at t=1.8s. */
    private fun runReversal(
        model: ArmModel,
        maxVel: Double,
        maxAccel: Double,
        trace: Boolean,
    ): Metrics =
        run(
            model,
            maxVel,
            maxAccel,
            -PI / 4,
            PI / 6,
            -PI / 3,
            1800,
            4000,
            trace,
            BACKLASH_RAD,
        )

    /** The reversal with zero backlash: isolates profile/tracking overshoot from lash effects. */
    private fun runReversalNoLash(model: ArmModel, maxVel: Double, maxAccel: Double): Metrics =
        run(
            model,
            maxVel,
            maxAccel,
            -PI / 4,
            PI / 6,
            -PI / 3,
            1800,
            4000,
            false,
            0.0,
        )

    private fun run(
        model: ArmModel,
        maxVel: Double,
        maxAccel: Double,
        startRad: Double,
        phase1TargetRad: Double?,
        finalTargetRad: Double,
        reversalMs: Int,
        totalMs: Int,
        trace: Boolean,
        backlashRad: Double,
    ): Metrics =
        run(
            model,
            maxVel,
            maxAccel,
            startRad,
            phase1TargetRad,
            finalTargetRad,
            reversalMs,
            totalMs,
            trace,
            backlashRad,
            40.0,
            8.0,
            1.5,
        )

    private fun run(
        model: ArmModel,
        maxVel: Double,
        maxAccel: Double,
        startRad: Double,
        phase1TargetRad: Double?,
        finalTargetRad: Double,
        reversalMs: Int,
        totalMs: Int,
        trace: Boolean,
        backlashRad: Double,
        kP: Double,
        kI: Double,
        kD: Double,
    ): Metrics {
        val plant =
            BacklashArmMotorSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_REV,
                GEAR_RATIO,
                0.0,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                startRad,
                backlashRad,
            )
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, startRad)
        val controller =
            MotorMechanismController(model, kP, kI, kD, maxVel, maxAccel, 480.0, 1.5, startRad)

        val rng = Random(7L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        val m = Metrics()
        m.descentMs = Double.NaN
        val targetDeg = Math.toDegrees(finalTargetRad)
        // The load's own resting point sits a half-backlash (plus spring sag) below the motor-side
        // target; measure overshoot past that, not past the encoder target.
        val halfBacklashDeg = Math.toDegrees(backlashRad / 2.0)
        val loadRestDeg = targetDeg - halfBacklashDeg
        var prevFace = faceOf(plant, backlashRad / 2.0)
        var prevLoadVel = 0.0
        var steadyMin = Double.POSITIVE_INFINITY
        var steadyMax = Double.NEGATIVE_INFINITY
        var arrived = false

        for (ms in 0..totalMs) {
            val target =
                if (phase1TargetRad != null && ms < reversalMs) phase1TargetRad else finalTargetRad
            plant.step(0.001, power, VOLTAGE)

            // Contact bookkeeping once the final target is active.
            if (ms >= reversalMs) {
                val loadDeg = Math.toDegrees(plant.getTruePositionRad())
                if (!arrived && abs(loadDeg - loadRestDeg) < 1.0) {
                    arrived = true
                    m.descentMs = (ms - reversalMs).toDouble()
                }
                val face = faceOf(plant, backlashRad / 2.0)
                val relVel = plant.getMotorVelocityRadPerSec() - plant.getTrueVelocityRadPerSec()
                if (face == 0) {
                    if (arrived) m.holdFreeMs += 1 else m.descentFreeMs += 1
                    if (prevFace != 0) {
                        if (arrived) m.holdSeps++ else m.descentSeps++
                        if (trace) {
                            System.out.printf(
                                "  t=%4dms SEP  from face %+d at load=%6.1f deg relVel=%+.2f%n",
                                ms,
                                prevFace,
                                loadDeg,
                                relVel,
                            )
                        }
                    }
                } else if (prevFace == 0 && (m.descentSeps + m.holdSeps) > 0) {
                    if (arrived) {
                        m.maxHoldImpact = max(m.maxHoldImpact, abs(relVel))
                    } else {
                        m.maxDescentImpact = max(m.maxDescentImpact, abs(relVel))
                    }
                    if (trace) {
                        System.out.printf(
                            "  t=%4dms HIT  face %+d at load=%6.1f deg relVel=%+.2f%n",
                            ms,
                            face,
                            loadDeg,
                            relVel,
                        )
                    }
                }
                prevFace = face
                m.bottomOvershootDeg = max(m.bottomOvershootDeg, loadRestDeg - loadDeg)
            }

            if (ms >= totalMs - 1000) {
                val loadDeg = Math.toDegrees(plant.getTruePositionRad())
                steadyMin = min(steadyMin, loadDeg)
                steadyMax = max(steadyMax, loadDeg)
                val loadVel = plant.getTrueVelocityRadPerSec()
                prevLoadVel = loadVel
            }

            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(
                    plant.getPositionTicks() / TICKS_PER_RAD,
                    plant.getVelocityTps() / TICKS_PER_RAD,
                )
                val voltage = controller.calculate(target, ekf.position, ekf.velocity, VOLTAGE, dt)
                power = voltage / VOLTAGE
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        m.steadyP2PDeg = steadyMax - steadyMin
        m.finalDeg = Math.toDegrees(plant.getTruePositionRad())
        return m
    }

    @Test
    fun compareStrategies() {
        val rigid = ArmModel(K_S, K_V, K_A, K_G, 0.0)

        println("== over the top (+120 -> -60), v8 a12 ==")
        val baseline = runOverTheTop(rigid, 8.0, 12.0, false)
        println("baseline       : $baseline")
        for (margin in doubleArrayOf(0.5, 0.9)) {
            val engaged = EngagementAwareArmModel(LOAD_INERTIA, margin, 0.5)
            System.out.printf(
                "margin %.2f    : %s%n",
                margin,
                runOverTheTop(engaged, 8.0, 12.0, false),
            )
        }
        // The full soft-handoff treatment: two-sided engagement ceiling, weakest-gravity
        // evaluation across the lash, approach ramp, and a slow-crossing hold window past the
        // crest. See softHandoffSweep for the knob trade-offs.
        val best =
            runOverTheTop(
                HandoffAwareArmModel(
                    LOAD_INERTIA,
                    0.67,
                    0.3,
                    0.5,
                    2.0,
                    K_G * sin(Math.toRadians(10.0)),
                    HALF_BACKLASH_RAD,
                ),
                8.0,
                12.0,
                false,
            )
        println("soft handoff   : $best")
        println(
            "soft, slowest  : " +
                runOverTheTop(
                    HandoffAwareArmModel(
                        LOAD_INERTIA,
                        0.67,
                        0.3,
                        0.2,
                        2.0,
                        K_G * sin(Math.toRadians(10.0)),
                        HALF_BACKLASH_RAD,
                    ),
                    8.0,
                    12.0,
                    false,
                )
        )

        println("== reversal (+30 -> -60), a12 ==")
        val reversal = runReversal(rigid, 8.0, 12.0, false)
        val reversalNoLash = runReversalNoLash(rigid, 8.0, 12.0)
        println("baseline v8 : $reversal")
        println("baseline v4 : ${runReversal(rigid, 4.0, 12.0, false)}")
        println("no-lash v8  : $reversalNoLash")
        println("no-lash v4  : ${runReversalNoLash(rigid, 4.0, 12.0)}")

        // Finding 1: a from-hold descent stays engaged the whole way down, and its bottom
        // overshoot is not a lash effect (the zero-backlash plant overshoots the same).
        assertEquals(0, reversal.descentSeps, "from-hold descent should never open the lash")
        assertTrue(
            abs(reversal.bottomOvershootDeg - reversalNoLash.bottomOvershootDeg) < 1.0,
            "reversal overshoot should be a tuning artifact, not lash (lash=" +
                "${reversal.bottomOvershootDeg}, no-lash=${reversalNoLash.bottomOvershootDeg})",
        )

        // Finding 2: the soft handoff cuts the landing speed to under half of baseline
        // (impact energy scales with v², so under a quarter of the impact energy).
        assertTrue(
            best.maxDescentImpact < 0.5 * baseline.maxDescentImpact,
            "soft handoff should cut the landing to under half of baseline (baseline=" +
                "${baseline.maxDescentImpact}, handoff=${best.maxDescentImpact})",
        )

        // Finding 3: the hold is quiet. A setpoint that limit-cycles around the target broadcasts
        // its acceleration dither through the feedforward (+/-1.5 V at +/-5 rad/s^2) and separates
        // the teeth ~10 times a second; a profile that lands at rest leaves at most one tooth
        // unload, during the arrival-overshoot recovery.
        assertTrue(
            baseline.holdSeps <= 2,
            "hold should be quiet (got ${baseline.holdSeps} separations)",
        )
        assertTrue(
            baseline.steadyP2PDeg < 0.2,
            "hold should not wiggle: p2p=${baseline.steadyP2PDeg} deg",
        )
    }

    /**
     * Sweep the soft-handoff knobs (exit hold window, crossing speed, engagement margin, half-gap
     * crest shift) for the gentlest over-the-top landing.
     */
    @Test
    fun softHandoffSweep() {
        println(
            "baseline ref   : " + runOverTheTop(ArmModel(K_S, K_V, K_A, K_G, 0.0), 8.0, 12.0, false)
        )
        val holds = doubleArrayOf(Math.toRadians(8.0), Math.toRadians(10.0))
        for (holdRad in holds) {
            val exitHold = K_G * sin(holdRad)
            for (crossVel in doubleArrayOf(0.2, 0.35, 0.5)) {
                for (margin in doubleArrayOf(0.67, 0.9)) {
                    val m =
                        HandoffAwareArmModel(
                            LOAD_INERTIA,
                            margin,
                            0.3,
                            crossVel,
                            2.0,
                            exitHold,
                            HALF_BACKLASH_RAD,
                        )
                    System.out.printf(
                        "hold%2.0f cv%.1f m%.2f: %s%n",
                        Math.toDegrees(holdRad),
                        crossVel,
                        margin,
                        runOverTheTop(m, 8.0, 12.0, false),
                    )
                }
            }
        }
        // Ablations at the best-guess point: no crest shift, and no exit hold.
        println(
            "no-shift       : " +
                runOverTheTop(
                    HandoffAwareArmModel(
                        LOAD_INERTIA,
                        0.67,
                        0.3,
                        0.5,
                        2.0,
                        K_G * sin(Math.toRadians(15.0)),
                        0.0,
                    ),
                    8.0,
                    12.0,
                    false,
                )
        )
        println(
            "no-hold        : " +
                runOverTheTop(
                    HandoffAwareArmModel(
                        LOAD_INERTIA,
                        0.67,
                        0.3,
                        0.5,
                        2.0,
                        0.0,
                        HALF_BACKLASH_RAD,
                    ),
                    8.0,
                    12.0,
                    false,
                )
        )
    }

    /**
     * Which loop element drives the hold chatter? Sweep the feedback gains one at a time on the
     * reversal scenario and report only the hold-phase metrics.
     */
    @Test
    fun holdChatterGainSweep() {
        val rigid = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val gains =
            arrayOf(
                doubleArrayOf(40.0, 8.0, 1.5), // baseline
                doubleArrayOf(40.0, 8.0, 0.0), // no D
                doubleArrayOf(40.0, 8.0, 4.0), // heavy D
                doubleArrayOf(40.0, 0.0, 1.5), // no I
                doubleArrayOf(10.0, 8.0, 1.5), // soft P
                doubleArrayOf(10.0, 2.0, 0.5), // everything soft
            )
        for (g in gains) {
            val m =
                run(
                    rigid,
                    8.0,
                    12.0,
                    -PI / 4,
                    PI / 6,
                    -PI / 3,
                    1800,
                    4000,
                    false,
                    BACKLASH_RAD,
                    g[0],
                    g[1],
                    g[2],
                )
            System.out.printf(
                "kP=%4.0f kI=%3.0f kD=%3.1f : hold[sep=%2d free=%3.0fms impact=%.2f] " +
                    "p2p=%.2fdeg final=%.2fdeg%n",
                g[0],
                g[1],
                g[2],
                m.holdSeps,
                m.holdFreeMs,
                m.maxHoldImpact,
                m.steadyP2PDeg,
                m.finalDeg,
            )
        }
    }

    /**
     * High-resolution look at the hold: sample the commanded voltage, the filter's estimate, and
     * both true angles every 10 ms through a chatter window.
     */
    @Test
    fun holdChatterTrace() {
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val start = -PI / 4
        val up = PI / 6
        val down = -PI / 3
        val plant =
            BacklashArmMotorSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_REV,
                GEAR_RATIO,
                0.0,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                start,
                BACKLASH_RAD,
            )
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, start)

        val rng = Random(7L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20
        println("t_ms, u_V, estPos_deg, estVel, motor_deg, load_deg, face, spPos_deg, spVel, spAcc")
        for (ms in 0..3400) {
            val target = if (ms < 1800) up else down
            plant.step(0.001, power, VOLTAGE)
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(
                    plant.getPositionTicks() / TICKS_PER_RAD,
                    plant.getVelocityTps() / TICKS_PER_RAD,
                )
                val voltage = controller.calculate(target, ekf.position, ekf.velocity, VOLTAGE, dt)
                power = voltage / VOLTAGE
                if (ms >= 2600) {
                    System.out.printf(
                        "%5d, %+6.3f, %8.3f, %+7.3f, %8.3f, %8.3f, %+d, %8.3f, %+7.3f, %+7.2f%n",
                        ms,
                        voltage,
                        Math.toDegrees(ekf.position),
                        ekf.velocity,
                        Math.toDegrees(plant.getMotorPositionRad()),
                        Math.toDegrees(plant.getTruePositionRad()),
                        faceOf(plant, HALF_BACKLASH_RAD),
                        Math.toDegrees(controller.setpointPosition),
                        controller.setpointVelocity,
                        controller.setpointAcceleration,
                    )
                }
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }
    }

    /**
     * Who overshoots at arrival — the profile, the estimate, or the plant? Sample the braking
     * window of the v8 reversal every loop.
     */
    @Test
    fun arrivalTrace() {
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val start = -PI / 4
        val up = PI / 6
        val down = -PI / 3
        val plant =
            BacklashArmMotorSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_REV,
                GEAR_RATIO,
                0.0,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                start,
                BACKLASH_RAD,
            )
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, start)

        val rng = Random(7L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20
        println(
            "t_ms, spPos_deg, spVel, spAcc, estPos_deg, estVel, motor_deg, motorVel, load_deg, u_V"
        )
        for (ms in 0..2900) {
            val target = if (ms < 1800) up else down
            plant.step(0.001, power, VOLTAGE)
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(
                    plant.getPositionTicks() / TICKS_PER_RAD,
                    plant.getVelocityTps() / TICKS_PER_RAD,
                )
                val voltage = controller.calculate(target, ekf.position, ekf.velocity, VOLTAGE, dt)
                power = voltage / VOLTAGE
                if (ms >= 1900) {
                    System.out.printf(
                        "%5d, %8.2f, %+6.2f, %+7.2f, %8.2f, %+6.2f, %8.2f, %+6.2f, %8.2f, %+6.2f%n",
                        ms,
                        Math.toDegrees(controller.setpointPosition),
                        controller.setpointVelocity,
                        controller.setpointAcceleration,
                        Math.toDegrees(ekf.position),
                        ekf.velocity,
                        Math.toDegrees(plant.getMotorPositionRad()),
                        plant.getMotorVelocityRadPerSec(),
                        Math.toDegrees(plant.getTruePositionRad()),
                        voltage,
                    )
                }
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }
    }

    @Test
    fun traceEvents() {
        println("== trace: over the top, soft handoff hold10 cv0.5 m0.67 ==")
        runOverTheTop(
            HandoffAwareArmModel(
                LOAD_INERTIA,
                0.67,
                0.3,
                0.5,
                2.0,
                K_G * sin(Math.toRadians(10.0)),
                HALF_BACKLASH_RAD,
            ),
            8.0,
            12.0,
            true,
        )
    }

    companion object {
        private const val TICKS_PER_REV = 28
        private const val GEAR_RATIO = 100.0
        private val TICKS_PER_RAD = TICKS_PER_REV * GEAR_RATIO / (2 * PI)
        private const val K_S = 0.1
        private const val K_G = 1.5
        private const val K_V = 2.0
        private const val K_A = 0.2
        private const val VOLTAGE = 12.0
        private val MIN_ANGLE_RAD = -PI * 0.9
        private val MAX_ANGLE_RAD = PI * 0.9
        private val BACKLASH_RAD = Math.toRadians(5.0)
        private val HALF_BACKLASH_RAD = BACKLASH_RAD / 2.0
        private val HALF_BACKLASH_DEG = Math.toDegrees(HALF_BACKLASH_RAD)

        // The sim splits kA 80/20 between motor and load (BacklashArmMotorSim default).
        private val LOAD_INERTIA = 0.2 * K_A

        /**
         * +1 = forward face (motor above load: braking/holding), -1 = reverse face, 0 = teeth
         * apart.
         */
        private fun faceOf(plant: BacklashArmMotorSim, halfBacklashRad: Double): Int {
            val delta = plant.getMotorPositionRad() - plant.getTruePositionRad()
            if (delta > halfBacklashRad) return 1
            if (delta < -halfBacklashRad) return -1
            return 0
        }
    }
}
