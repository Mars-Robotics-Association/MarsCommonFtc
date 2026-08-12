package org.marsroboticsassociation.controllab.trajectory

import java.awt.BasicStroke
import java.awt.Color
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment

@JvmRecord
data class TrajectorySvgSeries(
    val label: String,
    val segments: List<TrajectoryCurveSegment>,
    val color: Color,
    val strokeWidth: Float,
    val stroke: BasicStroke?,
)
