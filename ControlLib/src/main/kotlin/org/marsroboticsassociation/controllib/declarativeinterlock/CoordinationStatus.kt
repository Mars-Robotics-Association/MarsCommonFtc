package org.marsroboticsassociation.controllib.declarativeinterlock

enum class ExecutionState {
    IDLE,
    RUNNING,
    SUCCEEDED,
    FAILED,
}

data class ConditionStatus(
    val description: String,
    val progress: GoalProgress,
    val source: String,
)

data class CommandStatus(
    val mechanism: String,
    val condition: String,
    val mode: CommandMode,
    val source: String,
)

data class CoordinationStatus(
    val request: String?,
    val stage: String?,
    val state: ExecutionState,
    val conditions: List<ConditionStatus>,
    val commands: List<CommandStatus>,
    val failure: String?,
)
