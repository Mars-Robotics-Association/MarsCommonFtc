package org.marsroboticsassociation.controllib.filter

interface LowPassFilter : Filter {
    fun setCutoffHz(cutoffHz: Double)
}
