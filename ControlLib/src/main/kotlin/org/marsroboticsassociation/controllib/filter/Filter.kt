package org.marsroboticsassociation.controllib.filter

interface Filter {
    /**
     * Update filter with the next sample value using elapsed dt (seconds). If dt <= 0, filter may
     * decide to initialize or return the input.
     */
    fun update(x: Double, dt: Double): Double

    /** Derivative estimate, or NaN if filter type does not support it. */
    val rate: Double

    val value: Double

    /** Reset internal state. */
    fun reset()
}
