package org.marsroboticsassociation.controllab.trajectory

@JvmRecord
data class TrajectorySvgModel(
    val xMin: Double,
    val xMax: Double,
    val minY: Double,
    val maxY: Double,
    val series: List<TrajectorySvgSeries>,
)
