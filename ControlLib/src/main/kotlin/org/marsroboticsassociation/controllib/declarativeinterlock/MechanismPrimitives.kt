package org.marsroboticsassociation.controllib.declarativeinterlock

import java.util.function.Function

/** Whether reconciliation may reuse an existing command or must reissue it. */
enum class CommandMode {
    IF_NEEDED,
    FORCE,
}

/** Provides the current OpMode/runtime clock in seconds. */
fun interface SecondsClock {
    fun nowSeconds(): Double
}

/** A mechanism's assessment of one semantic condition in the current snapshot. */
class GoalProgress private constructor(
    val kind: Kind,
    val reason: String?,
) {
    enum class Kind {
        SATISFIED,
        IN_PROGRESS,
        UNCERTAIN,
        BLOCKED,
    }

    companion object {
        private val SATISFIED = GoalProgress(Kind.SATISFIED, null)
        private val IN_PROGRESS = GoalProgress(Kind.IN_PROGRESS, null)

        @JvmStatic
        fun satisfied(): GoalProgress = SATISFIED

        @JvmStatic
        fun inProgress(): GoalProgress = IN_PROGRESS

        @JvmStatic
        fun uncertain(reason: String): GoalProgress = GoalProgress(Kind.UNCERTAIN, reason)

        @JvmStatic
        fun blocked(reason: String): GoalProgress = GoalProgress(Kind.BLOCKED, reason)
    }
}

/** Immutable state captured after every mechanism has received its periodic call. */
class CoordinationSnapshot internal constructor(
    val nowSeconds: Double,
    private val states: Map<SelfReconcilingMechanism<*, *>, Any>,
) {
    @Suppress("UNCHECKED_CAST")
    fun <Goal : Any, State : Any> stateOf(mechanism: SelfReconcilingMechanism<Goal, State>): State =
        states.getValue(mechanism) as State
}

/**
 * A component which owns its commands, observations, estimates, and goal reconciliation.
 * Implementations are deliberately unaware of recipes and other mechanisms.
 */
abstract class SelfReconcilingMechanism<Goal : Any, State : Any>(
    val name: String,
) {
    abstract fun periodic(nowSeconds: Double)

    abstract val state: State

    abstract fun progress(goal: Goal, state: State): GoalProgress

    fun command(goal: Goal): Boolean = command(goal, CommandMode.IF_NEEDED)

    /** Returns true only when a hardware command was actually issued. */
    abstract fun command(goal: Goal, mode: CommandMode): Boolean
}

/** A semantic fact, optionally paired with a goal that can establish it. */
class MechanismCondition<Goal : Any, State : Any>(
    val mechanism: SelfReconcilingMechanism<Goal, State>,
    val description: String,
    val establishmentGoal: Goal?,
    private val assessment: Function<State, GoalProgress>,
) {
    internal val key: String = "${mechanism.name}:$description"

    fun hasGoal(): Boolean = establishmentGoal != null

    fun goalEquals(goal: Any?): Boolean = establishmentGoal == goal

    fun isFor(mechanism: SelfReconcilingMechanism<*, *>): Boolean = this.mechanism === mechanism

    internal fun assess(snapshot: CoordinationSnapshot): GoalProgress =
        assessment.apply(snapshot.stateOf(mechanism))

    internal fun command(mode: CommandMode): Boolean =
        establishmentGoal?.let { mechanism.command(it, mode) } ?: false

    override fun toString(): String = description
}
