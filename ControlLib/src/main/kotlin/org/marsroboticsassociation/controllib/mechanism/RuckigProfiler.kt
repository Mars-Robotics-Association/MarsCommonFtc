package org.marsroboticsassociation.controllib.mechanism

import com.ruckig.InputParameter
import com.ruckig.Ruckig
import com.ruckig.Trajectory
import kotlin.math.max
import kotlin.math.min

/**
 * Online-trajectory-generation setpoint profiler backed by the Ruckig port (`com.ruckig`), the
 * [SetpointProfile] implementation.
 *
 * <p>Every [update] replans a time-optimal, jerk-limited trajectory from the setpoint's own
 * `(position, velocity, acceleration)` state to the target at rest, under the current limits, then
 * samples it `dt` ahead. This is Ruckig's intended usage pattern: the deceleration phase is
 * *planned* to land at `(target, 0, 0)` exactly, so a move finishes with a jerk ramp-out. Limits
 * rewritten between calls (back-EMF ceilings) are simply picked up by the next replan; states
 * outside the new limits (velocity or residual acceleration over a freshly lowered cap) are
 * absorbed by Ruckig's brake pre-trajectory rather than clamped.
 *
 * <p><b>Limit-frame mapping.</b> This class takes motion-frame limits per the [SetpointProfile]
 * contract (acceleration = speeding up, deceleration = braking, relative to the actual motion);
 * Ruckig's are signed. Each replan maps by the sign of the current velocity — moving in +,
 * `max_acceleration = maxAcceleration` and `min_acceleration = -maxDeceleration`; flipped when
 * moving in −; the direction of travel decides only at rest. Mapping by velocity sign (not travel
 * direction) matters for wrong-way motion: pushing back toward the target is physically braking and
 * must get the braking authority. A single plan containing a direction reversal still holds its
 * bounds constant through the reversal, so its post-reversal segment carries the pre-reversal
 * mapping; per-loop replanning refreshes the mapping, so the executed slice of each plan always
 * uses the physically correct authority. This is the deliberate trade documented in the port plan
 * (§7).
 *
 * <p><b>Back-EMF braking caveat.</b> A plan assumes its limits hold for the whole remaining move,
 * but [MechanismModel.maxSustainableDeceleration] shrinks as speed drops. Feeding this profiler the
 * instantaneous braking ceiling is therefore optimistic during a stop. Callers that rewrite limits
 * from the model each loop should pass a braking ceiling evaluated at low speed, which is a lower
 * bound on the authority available anywhere in the remaining brake ([ModelAwareRuckigProfiler] does
 * this itself).
 *
 * <p><b>Zero-authority limits.</b> Zero limits mean "no authority right now" (a back-EMF ceiling
 * lowers them to zero when the headroom runs out). Ruckig cannot plan with a zero limit, so limits
 * are floored at a tiny positive value: the resulting plan brakes normally and then crawls at
 * negligible speed — effectively a hold. If no valid plan exists from the exact current state
 * (per-loop ceiling rewrites can strand it just outside the fresh bounds with no matching
 * authority), the state is clamped into the band and replanned, and only if that also fails does
 * the setpoint hold for the step ([lastResult] exposes the Ruckig result code for debugging).
 *
 * <p>Steady-state `update` does not allocate. Not thread-safe.
 */
open class RuckigProfiler(
    maxVelocity: Double,
    maxAcceleration: Double,
    maxDeceleration: Double,
    maxJerk: Double,
    initialPosition: Double,
) : SetpointProfile {

    private val otg = Ruckig(1)
    private val input = InputParameter(1)
    private val trajectory = Trajectory(1)

    // Preallocated at_time outputs (the 3-array overload allocates internally; this one doesn't).
    private val pOut = DoubleArray(1)
    private val vOut = DoubleArray(1)
    private val aOut = DoubleArray(1)
    private val jOut = DoubleArray(1)
    private val sectionOut = IntArray(1)

    final override var position: Double
        protected set

    final override var velocity = 0.0
        protected set

    final override var acceleration = 0.0
        protected set

    private var maxVelocity: Double
    private var maxAcceleration: Double
    private var maxDeceleration: Double
    private var maxJerk: Double

    var lastResult = com.ruckig.Result.Finished
        private set

    init {
        this.maxVelocity = requireNonNegative(maxVelocity, "maxVelocity")
        this.maxAcceleration = requireNonNegative(maxAcceleration, "maxAcceleration")
        this.maxDeceleration = requireNonNegative(maxDeceleration, "maxDeceleration")
        this.maxJerk = requirePositiveJerk(maxJerk)
        this.position = initialPosition
        // The min_acceleration array is set per replan; allocate it once here.
        input.min_acceleration = DoubleArray(1)
        input.target_velocity[0] = 0.0
        input.target_acceleration[0] = 0.0
    }

    override fun reset(position: Double) {
        this.position = position
        this.velocity = 0.0
        this.acceleration = 0.0
    }

    override fun setMaxVelocity(maxVelocity: Double) {
        this.maxVelocity = requireNonNegative(maxVelocity, "maxVelocity")
    }

    override fun setMaxAcceleration(maxAcceleration: Double) {
        this.maxAcceleration = requireNonNegative(maxAcceleration, "maxAcceleration")
    }

    override fun setMaxDeceleration(maxDeceleration: Double) {
        this.maxDeceleration = requireNonNegative(maxDeceleration, "maxDeceleration")
    }

    override fun setMaxJerk(maxJerk: Double) {
        this.maxJerk = requirePositiveJerk(maxJerk)
    }

    override fun update(targetPosition: Double, dt: Double) {
        // Reject non-positive and NaN dt, and non-finite targets. The !(x > 0) form catches NaN.
        if (!(dt > 0) || !isFinite(targetPosition)) {
            return
        }
        if (position == targetPosition && velocity == 0.0 && acceleration == 0.0) {
            return // settled; nothing to plan
        }

        // Motion-frame -> signed-frame limit mapping, by the sign of the current velocity (the
        // SetpointProfile contract: acceleration limits speeding up, deceleration limits braking,
        // relative to the actual motion). Only
        // at rest, where nothing is being braked, does the direction of travel decide. Mapping by
        // travel direction instead deadlocks on wrong-way motion: pushing back toward the target
        // is physically braking (back-EMF aids it), but a travel-frame map hands that push the
        // acceleration authority — which a back-EMF ceiling drives to zero exactly when the
        // mechanism is moving fast the wrong way.
        val travel = targetPosition - position
        val movingPositive = if (velocity != 0.0) velocity > 0 else travel >= 0
        val accelCap = max(if (movingPositive) maxAcceleration else maxDeceleration, MIN_LIMIT)
        val decelCap = max(if (movingPositive) maxDeceleration else maxAcceleration, MIN_LIMIT)

        input.current_position[0] = position
        input.current_velocity[0] = velocity
        input.current_acceleration[0] = acceleration
        input.target_position[0] = targetPosition
        input.max_velocity[0] = max(maxVelocity, MIN_LIMIT)
        input.max_acceleration[0] = accelCap
        input.min_acceleration[0] = -decelCap
        input.max_jerk[0] = maxJerk // UNLIMITED_JERK (infinity) selects second-order profiles

        lastResult = otg.calculate(input, trajectory)
        if (lastResult < 0) {
            // Per-loop limit rewrites can strand the state just outside the fresh bounds while
            // the matching authority is collapsed — e.g. cruising at the back-EMF velocity
            // ceiling as it inches down each loop with the accel ceiling at zero. From that
            // exact state no jerk-limited plan exists (Ruckig needs a brake pre-trajectory it
            // has no authority for), and holding would freeze the profile forever, because the
            // held state reproduces the same inputs next loop. Cope by clamping the state into
            // the band and planning from there. The velocity nick is bounded
            // by how far the ceiling moved in one loop, so it stays negligible.
            input.current_velocity[0] =
                clamp(velocity, -input.max_velocity[0], input.max_velocity[0])
            input.current_acceleration[0] = clamp(acceleration, -decelCap, accelCap)
            lastResult = otg.calculate(input, trajectory)
            if (lastResult < 0) {
                return // still infeasible: hold this step
            }
        }

        if (dt >= trajectory.get_duration()) {
            // The whole remaining move fits in this step: land exactly.
            position = targetPosition
            velocity = 0.0
            acceleration = 0.0
            return
        }
        trajectory.at_time(dt, pOut, vOut, aOut, jOut, sectionOut)
        position = pOut[0]
        velocity = vOut[0]
        acceleration = aOut[0]
    }

    /** Ruckig [com.ruckig.Result] code of the most recent replan, for telemetry/debugging. */
    companion object {
        /** Pass as `maxJerk` to disable the jerk limit (second-order, bang-bang acceleration). */
        @JvmField val UNLIMITED_JERK: Double = Double.POSITIVE_INFINITY

        /**
         * Floor applied to zero limits so a plan always exists. Small enough that "crawling" at
         * this speed is indistinguishable from holding in any practical unit system.
         */
        internal const val MIN_LIMIT = 1e-9

        private fun isFinite(v: Double): Boolean {
            return !v.isNaN() && !v.isInfinite()
        }

        private fun clamp(value: Double, lo: Double, hi: Double): Double {
            return max(lo, min(hi, value))
        }

        /** Zero is allowed (it means "no authority right now"); negatives and NaN are rejected. */
        private fun requireNonNegative(value: Double, name: String): Double {
            if (!(value >= 0)) {
                throw IllegalArgumentException("$name must be non-negative; got $value")
            }
            return value
        }

        /** Strictly positive; [UNLIMITED_JERK] is the sanctioned way to ask for no limit. */
        private fun requirePositiveJerk(maxJerk: Double): Double {
            if (!(maxJerk > 0)) {
                throw IllegalArgumentException(
                    "maxJerk must be positive (use RuckigProfiler.UNLIMITED_JERK for no jerk" +
                        " limit); got " +
                        maxJerk
                )
            }
            return maxJerk
        }
    }
}
