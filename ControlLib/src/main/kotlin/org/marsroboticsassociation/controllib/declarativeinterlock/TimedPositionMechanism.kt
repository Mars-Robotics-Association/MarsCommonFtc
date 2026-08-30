package org.marsroboticsassociation.controllib.declarativeinterlock

import java.util.function.Consumer
import java.util.function.ToDoubleFunction

/** Self-reconciling adapter for a feedback-free, deadline-estimated position actuator. */
class TimedPositionMechanism<T : Any> constructor(
    name: String,
    clock: SecondsClock,
    params: TimedServo.Params,
    initiallySettled: Boolean = true,
    commandHardware: Consumer<T>,
    targetPosition: ToDoubleFunction<T>? = null,
) : SelfReconcilingMechanism<T, TimedPositionMechanism.State<T>>(name) {
    constructor(
        name: String,
        clock: SecondsClock,
        params: TimedServo.Params,
        commandHardware: Consumer<T>,
        targetPosition: ToDoubleFunction<T>,
    ) : this(name, clock, params, true, commandHardware, targetPosition)

    constructor(
        name: String,
        clock: SecondsClock,
        params: TimedServo.Params,
        commandHardware: Consumer<T>,
    ) : this(name, clock, params, true, commandHardware, null)

    /** Commanded target plus conservative deadline-based settled estimate. */
    class State<T> @JvmOverloads constructor(
        private val target: T?,
        private val settled: Boolean,
        private val modeledPosition: Double? = null,
    ) {
        fun getTarget(): T? = target

        fun isSettled(): Boolean = settled

        fun getModeledPosition(): Double? = modeledPosition
    }

    private val servo = TimedServo(clock, params, initiallySettled, commandHardware, targetPosition)

    override fun periodic(nowSeconds: Double) = Unit

    override val state: State<T>
        get() = State(servo.target, servo.isSettled, servo.modeledPosition)

    override fun progress(goal: T, state: State<T>): GoalProgress {
        if (state.getTarget() != goal) {
            return GoalProgress.uncertain("currently targeting ${state.getTarget()}")
        }
        if (state.isSettled()) {
            return GoalProgress.satisfied()
        }
        return GoalProgress.inProgress()
    }

    override fun command(goal: T, mode: CommandMode): Boolean {
        val shouldIssue = mode == CommandMode.FORCE || servo.target != goal
        if (shouldIssue) {
            servo.command(goal, mode == CommandMode.FORCE)
        }
        return shouldIssue
    }

    fun settledAt(target: T): MechanismCondition<T, State<T>> =
        MechanismCondition(this, "$name settled at $target", target) { progress(target, it) }

    fun settled(): MechanismCondition<T, State<T>> =
        MechanismCondition(this, "$name settled", null) {
            if (it.isSettled()) GoalProgress.satisfied() else GoalProgress.inProgress()
        }
}
