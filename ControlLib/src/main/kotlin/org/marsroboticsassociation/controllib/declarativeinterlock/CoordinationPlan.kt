package org.marsroboticsassociation.controllib.declarativeinterlock

data class AchieveStage(
    val label: String,
    val conditions: List<MechanismCondition<*, *>>,
    val commandMode: CommandMode,
)

data class CoordinationPlan(
    val name: String,
    val stages: List<AchieveStage>,
) {
    class Builder internal constructor(private val name: String) {
        private val stages = mutableListOf<AchieveStage>()

        fun achieve(label: String, vararg conditions: MechanismCondition<*, *>): Builder =
            addStage(label, CommandMode.IF_NEEDED, conditions.toList())

        fun force(label: String, vararg conditions: MechanismCondition<*, *>): Builder =
            addStage(label, CommandMode.FORCE, conditions.toList())

        fun build(): CoordinationPlan {
            require(stages.isNotEmpty()) { "Plan '$name' has no stages" }
            return CoordinationPlan(name, stages.toList())
        }

        private fun addStage(
            label: String,
            commandMode: CommandMode,
            conditions: List<MechanismCondition<*, *>>,
        ): Builder {
            require(conditions.isNotEmpty()) { "Plan '$name' stage '$label' is empty" }
            rejectContradictoryGoals(name, label, conditions)
            stages += AchieveStage(label, conditions, commandMode)
            return this
        }
    }

    companion object {
        @JvmStatic
        fun named(name: String): Builder = Builder(name)
    }
}

private fun rejectContradictoryGoals(
    planName: String,
    stageLabel: String,
    conditions: List<MechanismCondition<*, *>>,
) {
    conditions.groupBy { it.mechanism }.forEach { (mechanism, sameMechanism) ->
        val goals = sameMechanism.mapNotNull { it.establishmentGoal }.distinct()
        require(goals.size <= 1) {
            "Plan '$planName' stage '$stageLabel' requests contradictory goals for ${mechanism.name}: $goals"
        }
    }
}
