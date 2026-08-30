package org.marsroboticsassociation.controllib.declarativeinterlock

import java.util.function.Function
import java.util.function.Predicate

/** A directional implication: pursuing [trigger] requires all [requirements]. */
class PhysicalInvariant(
    val name: String,
    internal val trigger: Predicate<MechanismCondition<*, *>>,
    internal val requirements: Function<MechanismCondition<*, *>, List<MechanismCondition<*, *>>>,
)

class CoordinationModel internal constructor(
    internal val mechanisms: List<SelfReconcilingMechanism<*, *>>,
    internal val invariants: List<PhysicalInvariant>,
) {
    class Builder {
        private val mechanisms = mutableListOf<SelfReconcilingMechanism<*, *>>()
        private val invariants = mutableListOf<PhysicalInvariant>()

        fun register(mechanism: SelfReconcilingMechanism<*, *>): Builder {
            require(mechanism !in mechanisms) { "Mechanism ${mechanism.name} is already registered" }
            require(mechanisms.none { it.name == mechanism.name }) {
                "Mechanism names must be unique: ${mechanism.name}"
            }
            mechanisms += mechanism
            return this
        }

        fun invariant(
            name: String,
            whenever: Predicate<MechanismCondition<*, *>>,
            require: Function<MechanismCondition<*, *>, List<MechanismCondition<*, *>>>,
        ): Builder {
            invariants += PhysicalInvariant(name, whenever, require)
            return this
        }

        fun build(): CoordinationModel = CoordinationModel(mechanisms.toList(), invariants.toList())
    }

    companion object {
        @JvmStatic
        fun builder(): Builder = Builder()
    }
}
