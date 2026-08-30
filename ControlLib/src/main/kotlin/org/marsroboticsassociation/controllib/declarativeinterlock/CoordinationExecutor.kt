package org.marsroboticsassociation.controllib.declarativeinterlock

/** Single-thread-confined, deterministic constraint-aware scheduler. */
class CoordinationExecutor(private val model: CoordinationModel) {
    private data class Request(
        val plan: CoordinationPlan,
        var stageIndex: Int = 0,
        val forcedCommandsIssued: MutableSet<String> = mutableSetOf(),
    )

    private data class ExpandedCondition(
        val condition: MechanismCondition<*, *>,
        val source: String,
        val requirements: MutableList<ExpandedCondition> = mutableListOf(),
    )

    private var request: Request? = null
    private var lastStatus = CoordinationStatus(
        request = null,
        stage = null,
        state = ExecutionState.IDLE,
        conditions = emptyList(),
        commands = emptyList(),
        failure = null,
    )

    val status: CoordinationStatus
        get() = lastStatus

    val isRunning: Boolean
        get() = request != null

    val isFaulted: Boolean
        get() = lastStatus.state == ExecutionState.FAILED

    fun submit(plan: CoordinationPlan): Boolean {
        if (isFaulted) return false
        start(plan)
        return true
    }

    /** Explicit operator recovery is the only request admitted while faulted. */
    fun submitRecovery(plan: CoordinationPlan) {
        start(plan)
    }

    fun cancel() {
        request = null
        lastStatus = CoordinationStatus(
            request = null,
            stage = null,
            state = ExecutionState.IDLE,
            conditions = emptyList(),
            commands = emptyList(),
            failure = null,
        )
    }

    fun tick(nowSeconds: Double) {
        model.mechanisms.forEach { it.periodic(nowSeconds) }
        val snapshot = CoordinationSnapshot(nowSeconds, model.mechanisms.associateWith { it.state })
        val active = request ?: return

        while (active.stageIndex < active.plan.stages.size) {
            val stage = active.plan.stages[active.stageIndex]
            val roots = try {
                expand(stage.conditions)
            } catch (error: IllegalStateException) {
                fail(active, stage, error.message ?: "Invalid dependency graph")
                return
            }
            val all = roots.flatMap(::flatten).distinctBy { it.condition.key }
            val contradiction = contradictionAmong(all.map { it.condition })
            if (contradiction != null) {
                fail(active, stage, contradiction)
                return
            }

            val assessments = all.associate { it.condition.key to it.condition.assess(snapshot) }
            val blocked = assessments.entries.firstOrNull { it.value.kind == GoalProgress.Kind.BLOCKED }
            if (blocked != null) {
                val reason = blocked.value.reason ?: "blocked"
                val description = all.first { it.condition.key == blocked.key }.condition.description
                fail(active, stage, "$description: $reason")
                return
            }

            val forceWasIssued =
                stage.commandMode != CommandMode.FORCE ||
                    roots.filter { it.condition.establishmentGoal != null }
                        .all { it.condition.key in active.forcedCommandsIssued }
            val authoredSatisfied =
                forceWasIssued &&
                    roots.all { assessments.getValue(it.condition.key).kind == GoalProgress.Kind.SATISFIED }
            lastStatus = statusFor(active, stage, all, assessments)
            if (authoredSatisfied) {
                active.stageIndex++
                active.forcedCommandsIssued.clear()
                if (active.stageIndex == active.plan.stages.size) {
                    request = null
                    lastStatus = lastStatus.copy(stage = null, state = ExecutionState.SUCCEEDED)
                    return
                }
                continue
            }

            val ordered = all.sortedBy { model.mechanisms.indexOf(it.condition.mechanism) }
            val commands = mutableListOf<CommandStatus>()
            ordered.forEach { node ->
                val progress = assessments.getValue(node.condition.key)
                if (node.condition.establishmentGoal == null) return@forEach
                if (progress.kind == GoalProgress.Kind.SATISFIED && stage.commandMode != CommandMode.FORCE) return@forEach
                if (node.requirements.any {
                    assessments.getValue(it.condition.key).kind != GoalProgress.Kind.SATISFIED
                }) {
                    return@forEach
                }
                if (stage.commandMode == CommandMode.FORCE && node.condition.key in active.forcedCommandsIssued) {
                    return@forEach
                }
                if (node.condition.command(stage.commandMode)) {
                    commands += CommandStatus(
                        mechanism = node.condition.mechanism.name,
                        condition = node.condition.description,
                        mode = stage.commandMode,
                        source = node.source,
                    )
                }
                if (stage.commandMode == CommandMode.FORCE) active.forcedCommandsIssued += node.condition.key
            }
            lastStatus = lastStatus.copy(commands = commands)
            return
        }
    }

    private fun start(plan: CoordinationPlan) {
        request = Request(plan)
        lastStatus = CoordinationStatus(
            request = plan.name,
            stage = plan.stages.first().label,
            state = ExecutionState.RUNNING,
            conditions = emptyList(),
            commands = emptyList(),
            failure = null,
        )
    }

    private fun expand(conditions: List<MechanismCondition<*, *>>): List<ExpandedCondition> {
        fun visit(condition: MechanismCondition<*, *>, source: String, path: Set<String>): ExpandedCondition {
            check(condition.key !in path) { "Invariant dependency cycle at ${condition.description}" }
            val node = ExpandedCondition(condition, source)
            model.invariants.filter { it.trigger.test(condition) }.forEach { invariant ->
                invariant.requirements.apply(condition).forEach { requirement ->
                    node.requirements += visit(
                        requirement,
                        "required by '${invariant.name}' for ${condition.description}",
                        path + condition.key,
                    )
                }
            }
            return node
        }
        return conditions.map { visit(it, "authored condition", emptySet()) }
    }

    private fun flatten(node: ExpandedCondition): List<ExpandedCondition> =
        listOf(node) + node.requirements.flatMap(::flatten)

    private fun contradictionAmong(conditions: List<MechanismCondition<*, *>>): String? {
        conditions.groupBy { it.mechanism }.forEach { (mechanism, sameMechanism) ->
            val goals = sameMechanism.mapNotNull { it.establishmentGoal }.distinct()
            if (goals.size > 1) return "Contradictory active goals for ${mechanism.name}: $goals"
        }
        return null
    }

    private fun statusFor(
        request: Request,
        stage: AchieveStage,
        conditions: List<ExpandedCondition>,
        assessments: Map<String, GoalProgress>,
    ) = CoordinationStatus(
        request = request.plan.name,
        stage = stage.label,
        state = ExecutionState.RUNNING,
        conditions = conditions.map {
            ConditionStatus(it.condition.description, assessments.getValue(it.condition.key), it.source)
        },
        commands = emptyList(),
        failure = null,
    )

    private fun fail(request: Request, stage: AchieveStage, reason: String) {
        this.request = null
        lastStatus = CoordinationStatus(
            request = request.plan.name,
            stage = stage.label,
            state = ExecutionState.FAILED,
            conditions = emptyList(),
            commands = emptyList(),
            failure = reason,
        )
    }
}
