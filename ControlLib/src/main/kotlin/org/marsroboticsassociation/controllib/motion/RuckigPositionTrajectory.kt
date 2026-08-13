package org.marsroboticsassociation.controllib.motion

import com.ruckig.InputParameter
import com.ruckig.Ruckig
import com.ruckig.Trajectory
import kotlin.math.abs
import kotlin.math.max

/**
 * [PositionTrajectory] backed by the Ruckig port (`com.ruckig`): the time-optimal, jerk-limited
 * profile from `(p0, v0, a0)` to the target state, planned once at construction. Drop-in comparable
 * with [SCurvePosition] — the primary constructor matches
 * [PositionTrajectoryManager.TrajectoryFactory], so swapping planners is
 * `PositionTrajectoryManager(..., ::RuckigPositionTrajectory)`.
 *
 * <p>Beyond [SCurvePosition], this planner supports a nonzero target velocity and acceleration
 * (pass-through moves) via the extended constructor, and handles initial states outside the limits
 * (velocity or acceleration over their caps) with a proper brake pre-trajectory instead of a
 * special-cased prefix.
 *
 * <p>**Limit-frame mapping.** `aMaxAccel`/`aMaxDecel` are travel-frame (speeding up vs braking),
 * like [SCurvePosition]; Ruckig's limits are signed. They are mapped once at construction by the
 * direction of `pTarget - p0` — the same convention [SCurvePosition] uses for its `dir` frame.
 *
 * <p>After [getTotalTime], samples hold the target state extrapolated at constant acceleration:
 * exactly `(pTarget, 0, 0)` for the rest-target constructor. Like [SCurvePosition] under the
 * manager, replans preserve p/v/a continuity but not jerk.
 *
 * <p>Getter calls cache the most recent sample time, so the manager's
 * position-then-velocity-then-acceleration pattern costs one trajectory evaluation per loop. Not
 * thread-safe.
 */
class RuckigPositionTrajectory : PositionTrajectory {

    private val trajectory = Trajectory(1)
    override val totalTime: Double

    // Sample cache: one at_time evaluation serves the p/v/a getter triple.
    private val p = DoubleArray(1)
    private val v = DoubleArray(1)
    private val a = DoubleArray(1)
    private val j = DoubleArray(1)
    private val section = IntArray(1)
    private var cachedT = Double.NaN

    /**
     * Plan to the target at rest. Signature matches [PositionTrajectoryManager.TrajectoryFactory].
     */
    constructor(
        p0: Double,
        pTarget: Double,
        v0: Double,
        a0: Double,
        vMax: Double,
        aMaxAccel: Double,
        aMaxDecel: Double,
        jMax: Double,
    ) : this(p0, pTarget, v0, a0, 0.0, 0.0, vMax, aMaxAccel, aMaxDecel, jMax)

    /** Plan to a target moving at `vf` with acceleration `af` (pass-through move). */
    constructor(
        p0: Double,
        pTarget: Double,
        v0: Double,
        a0: Double,
        vf: Double,
        af: Double,
        vMax: Double,
        aMaxAccel: Double,
        aMaxDecel: Double,
        jMax: Double,
    ) {
        // Same direction convention as SCurvePosition: ties (pTarget == p0) plan in +.
        val movingPositive = pTarget >= p0

        val input = InputParameter(1)
        input.current_position[0] = p0
        input.current_velocity[0] = v0
        input.current_acceleration[0] = a0
        input.target_position[0] = pTarget
        input.target_velocity[0] = vf
        input.target_acceleration[0] = af
        input.max_velocity[0] = abs(vMax)
        input.max_acceleration[0] = abs(if (movingPositive) aMaxAccel else aMaxDecel)
        input.min_acceleration = doubleArrayOf(-abs(if (movingPositive) aMaxDecel else aMaxAccel))
        input.max_jerk[0] = abs(jMax)

        val result = Ruckig(1).calculate(input, trajectory)
        if (result < 0) {
            throw IllegalArgumentException(
                "Ruckig could not plan (result $result): p0=$p0 pTarget=$pTarget v0=$v0 a0=$a0 vf=$vf af=$af vMax=$vMax aMaxAccel=$aMaxAccel aMaxDecel=$aMaxDecel jMax=$jMax"
            )
        }
        this.totalTime = trajectory.get_duration()
    }

    private fun sample(t: Double) {
        val clamped = max(0.0, t)
        if (clamped == cachedT) {
            return
        }
        trajectory.at_time(clamped, p, v, a, j, section)
        cachedT = clamped
    }

    override fun getPosition(t: Double): Double {
        sample(t)
        return p[0]
    }

    override fun getVelocity(t: Double): Double {
        sample(t)
        return v[0]
    }

    override fun getAcceleration(t: Double): Double {
        sample(t)
        return a[0]
    }

    override fun isZeroJerk(t: Double): Boolean {
        sample(t)
        return j[0] == 0.0
    }
}
