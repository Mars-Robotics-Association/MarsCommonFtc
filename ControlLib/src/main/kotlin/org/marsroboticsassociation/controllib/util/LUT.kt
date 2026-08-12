package org.marsroboticsassociation.controllib.util

import java.util.TreeMap
import kotlin.math.abs

/** A lookup table. */
open class LUT<T : Number, R> : TreeMap<T, R>() {

    fun add(key: T, out: R) {
        put(key, out)
    }

    /** Returns the closest possible value for the given key, or null if the table is empty. */
    fun getClosest(key: T): R? {
        val ceil = ceilingEntry(key)
        val floor = floorEntry(key)

        return when {
            ceil != null && floor != null -> {
                val keyVal = key.toDouble()
                val ceilDiff = abs(ceil.key.toDouble() - keyVal)
                val floorDiff = abs(floor.key.toDouble() - keyVal)
                if (floorDiff < ceilDiff) floor.value else ceil.value
            }
            ceil != null -> ceil.value
            floor != null -> floor.value
            else -> null
        }
    }
}
