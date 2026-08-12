package org.marsroboticsassociation.controllib.util

import java.util.TreeMap

class LinInterpTable {
    private val map = TreeMap<Double, Double>()

    fun calculate(x: Double): Double {
        val ceiling = map.ceilingEntry(x)
        val lower = map.lowerEntry(x)
        return when {
            ceiling != null && lower != null ->
                linearInterpolation(x, lower.key, ceiling.key, lower.value, ceiling.value)
            ceiling != null -> ceiling.value
            lower != null -> lower.value
            else -> throw IllegalStateException("Table not initialized")
        }
    }

    fun add(x: Double, y: Double) {
        map[x] = y
    }

    companion object {
        @JvmStatic
        fun linearInterpolation(x: Double, x1: Double, x2: Double, y1: Double, y2: Double): Double {
            return if (x1 == x2) 0.0 else y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        }
    }
}
