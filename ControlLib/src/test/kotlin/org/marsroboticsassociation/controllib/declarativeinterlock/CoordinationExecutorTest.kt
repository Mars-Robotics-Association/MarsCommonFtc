package org.marsroboticsassociation.controllib.declarativeinterlock

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import java.util.function.Consumer
import java.util.function.ToDoubleFunction

class CoordinationExecutorTest {
    private var clockSeconds = 0.0

    @Test
    fun stagedPlanAdvancesThroughSatisfiedConditions() {
        val mechanism = FakeMechanism("arm")
        val executor = CoordinationExecutor(CoordinationModel.builder().register(mechanism).build())

        executor.submit(
            CoordinationPlan.named("two stages")
                .achieve("first", mechanism.at("A"))
                .achieve("second", mechanism.at("B"))
                .build(),
        )

        executor.tick(0.0)
        assertEquals(listOf("arm:A:IF_NEEDED"), mechanism.commands)
        executor.tick(0.1)
        assertEquals(listOf("arm:A:IF_NEEDED", "arm:B:IF_NEEDED"), mechanism.commands)
        executor.tick(0.2)
        assertEquals(ExecutionState.SUCCEEDED, executor.status.state)
    }

    @Test
    fun invariantPrerequisitesAreExpandedAndCommandedFirst() {
        val gate = FakeMechanism("gate")
        val latch = FakeMechanism("latch")
        val executor =
            CoordinationExecutor(
                CoordinationModel.builder()
                    .register(latch)
                    .register(gate)
                    .invariant(
                        "open requires unlocked",
                        { it.goalEquals("OPEN") },
                        { listOf(latch.at("UNLOCKED")) },
                    )
                    .build(),
            )

        executor.submit(CoordinationPlan.named("open").achieve("open", gate.at("OPEN")).build())

        executor.tick(0.0)
        assertEquals(listOf("latch:UNLOCKED:IF_NEEDED"), latch.commands)
        assertEquals("required by 'open requires unlocked' for gate at OPEN", executor.status.commands.single().source)
        executor.tick(0.1)
        assertEquals(listOf("gate:OPEN:IF_NEEDED"), gate.commands)
    }

    @Test
    fun contradictoryDerivedGoalFailsBeforeHardwareWrites() {
        val mechanism = FakeMechanism("gate")
        val executor =
            CoordinationExecutor(
                CoordinationModel.builder()
                    .register(mechanism)
                    .invariant("open requires closed", { it.goalEquals("OPEN") }, { listOf(mechanism.at("CLOSED")) })
                    .build(),
            )

        executor.submit(CoordinationPlan.named("open").achieve("open", mechanism.at("OPEN")).build())
        executor.tick(0.0)

        assertEquals(ExecutionState.FAILED, executor.status.state)
        assertTrue(executor.status.failure.orEmpty().contains("Contradictory active goals"))
        assertTrue(mechanism.commands.isEmpty())
        assertFalse(executor.submit(CoordinationPlan.named("closed").achieve("closed", mechanism.at("CLOSED")).build()))
    }

    @Test
    fun forcedStageReissuesCommandOnce() {
        val mechanism = FakeMechanism("kicker")
        mechanism.target = "HOME"
        val executor = CoordinationExecutor(CoordinationModel.builder().register(mechanism).build())

        executor.submitRecovery(CoordinationPlan.named("safe").force("force home", mechanism.at("HOME")).build())

        executor.tick(0.0)
        assertEquals(listOf("kicker:HOME:FORCE"), mechanism.commands)
        executor.tick(0.1)
        assertEquals(listOf("kicker:HOME:FORCE"), mechanism.commands)
        assertEquals(ExecutionState.SUCCEEDED, executor.status.state)
    }

    @Test
    fun cancelDropsPlanStateButDoesNotMutateMechanismState() {
        val mechanism = FakeMechanism("arm")
        val executor = CoordinationExecutor(CoordinationModel.builder().register(mechanism).build())

        executor.submit(CoordinationPlan.named("move").achieve("move", mechanism.at("A")).build())
        executor.tick(0.0)
        executor.cancel()

        assertEquals("A", mechanism.target)
        assertEquals(ExecutionState.IDLE, executor.status.state)
        assertFalse(executor.isRunning)
    }

    @Test
    fun modeledServoHasZeroDurationForZeroDistanceMove() {
        val servo = modeledServo(startTarget = "A")

        servo.command("A", force = true)

        assertTrue(servo.isSettled)
        assertEquals(0.0, servo.modeledPosition ?: Double.NaN, 1e-9)
    }

    @Test
    fun modeledServoUsesTriangularProfileWhenMoveCannotReachMaxVelocity() {
        val servo = modeledServo(startTarget = "A")

        servo.command("B")

        assertFalse(servo.isSettled)
        clockSeconds = 10.5
        assertEquals(0.25, servo.modeledPosition ?: Double.NaN, 1e-9)
        clockSeconds = 11.0
        assertEquals(0.5, servo.modeledPosition ?: Double.NaN, 1e-9)
        assertTrue(servo.isSettled)
    }

    @Test
    fun modeledServoUsesTrapezoidalProfileForLongerMove() {
        val servo = modeledServo(startTarget = "A")

        servo.command("C")

        clockSeconds = 10.5
        assertEquals(0.25, servo.modeledPosition ?: Double.NaN, 1e-9)
        clockSeconds = 11.5
        assertEquals(1.0, servo.modeledPosition ?: Double.NaN, 1e-9)
        clockSeconds = 12.0
        assertEquals(1.0, servo.modeledPosition ?: Double.NaN, 1e-9)
        assertTrue(servo.isSettled)
    }

    @Test
    fun modeledServoRetargetsFromCurrentModeledPosition() {
        val servo = modeledServo(startTarget = "A")
        servo.command("C")
        clockSeconds = 10.5

        servo.command("B")

        assertEquals(0.25, servo.modeledPosition ?: Double.NaN, 1e-9)
        clockSeconds = 10.5 + 2.0 * kotlin.math.sqrt(0.25 / 2.0)
        assertEquals(0.5, servo.modeledPosition ?: Double.NaN, 1e-9)
        assertTrue(servo.isSettled)
    }

    private fun modeledServo(startTarget: String): TimedServo<String> {
        clockSeconds = 0.0
        val params = TimedServo.Params().apply {
            travelSeconds = 10.0
            maxVelocity = 1.0
            maxAcceleration = 2.0
        }
        val positions = mapOf("A" to 0.0, "B" to 0.5, "C" to 1.0)
        val commands = mutableListOf<String>()
        val servo: TimedServo<String> = TimedServo(
            clock = SecondsClock { clockSeconds },
            params = params,
            commandHardware = Consumer { commands += it },
            targetPosition = ToDoubleFunction { positions.getValue(it) },
        )
        servo.command(startTarget)
        clockSeconds = params.travelSeconds
        assertTrue(servo.isSettled)
        return servo
    }

    private data class FakeState(val target: String?)

    private class FakeMechanism(name: String) : SelfReconcilingMechanism<String, FakeState>(name) {
        var target: String? = null
        val commands = mutableListOf<String>()

        override fun periodic(nowSeconds: Double) = Unit

        override val state: FakeState
            get() = FakeState(target)

        override fun progress(goal: String, state: FakeState): GoalProgress =
            if (state.target == goal) GoalProgress.satisfied() else GoalProgress.inProgress()

        override fun command(goal: String, mode: CommandMode): Boolean {
            target = goal
            commands += "$name:$goal:$mode"
            return true
        }

        fun at(goal: String): MechanismCondition<String, FakeState> =
            MechanismCondition(this, "$name at $goal", goal) { progress(goal, it) }
    }
}
