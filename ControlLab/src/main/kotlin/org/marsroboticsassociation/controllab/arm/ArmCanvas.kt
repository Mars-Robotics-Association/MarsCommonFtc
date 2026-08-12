package org.marsroboticsassociation.controllab.arm

import java.awt.BasicStroke
import java.awt.Color
import java.awt.Dimension
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import java.awt.geom.Arc2D
import java.awt.geom.Ellipse2D
import java.awt.geom.Line2D
import java.util.function.DoubleConsumer
import javax.swing.JPanel
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Animated 2D arm drawing. Paints, about a central pivot:
 * - the **load link** at [ArmEngine.getTrueLoadRad] (the real arm),
 * - the **motor-side link** at [ArmEngine.getMotorRad] (thin, green when the teeth are engaged, red
 *   when separated) — so the backlash gap between them is visible,
 * - a dashed **target ghost** at [ArmEngine.getTargetRad],
 * - the hard-stop arc between the min and max angles.
 *
 * Angles are radians from horizontal (positive above). Clicking or dragging on the canvas sets the
 * target to the pointer angle (clamped to the hard stops) via the supplied callback.
 */
class ArmCanvas(
    private val engine: ArmEngine,
    private val onTargetRad: DoubleConsumer,
) : JPanel() {
    init {
        preferredSize = Dimension(420, 420)
        background = Color.WHITE

        val mouse =
            object : MouseAdapter() {
                override fun mousePressed(e: MouseEvent) {
                    setTargetFromPoint(e)
                }

                override fun mouseDragged(e: MouseEvent) {
                    setTargetFromPoint(e)
                }
            }
        addMouseListener(mouse)
        addMouseMotionListener(mouse)
    }

    private fun setTargetFromPoint(e: MouseEvent) {
        val px = width / 2
        val py = height / 2
        val dx = (e.x - px).toDouble()
        val dy = (py - e.y).toDouble() // invert: screen y grows downward
        // atan2 is in (−π, π]; unwrap into the plant range (which may extend past −π).
        val angle =
            unwrapIntoRange(
                kotlin.math.atan2(dy, dx),
                engine.getMinAngleRad(),
                engine.getMaxAngleRad(),
            )
        onTargetRad.accept(angle)
    }

    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val g2 = g as Graphics2D
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)

        val px = width / 2
        val py = height / 2
        val armLen = min(width, height) * 0.38

        // Hard-stop arc (from min to max angle).
        val minA = engine.getMinAngleRad()
        val maxA = engine.getMaxAngleRad()
        g2.color = ARC_COLOR
        g2.stroke = BasicStroke(2f)
        // Arc2D uses degrees CCW from +x with +y up (matches our angle convention).
        val startDeg = Math.toDegrees(minA)
        val extentDeg = Math.toDegrees(maxA - minA)
        val arcR = armLen * 1.06
        g2.draw(
            Arc2D.Double(
                px - arcR,
                py - arcR,
                2 * arcR,
                2 * arcR,
                startDeg,
                extentDeg,
                Arc2D.OPEN,
            )
        )
        drawStopTick(g2, px, py, minA, armLen)
        drawStopTick(g2, px, py, maxA, armLen)

        // Target ghost (dashed).
        g2.color = TARGET_COLOR
        g2.stroke =
            BasicStroke(
                2f,
                BasicStroke.CAP_ROUND,
                BasicStroke.JOIN_ROUND,
                10f,
                floatArrayOf(8f, 6f),
                0f,
            )
        drawLink(g2, px, py, engine.getTargetRad(), armLen)

        // Motor-side link (thin, colored by engagement).
        val engaged = engine.isEngaged()
        g2.color = if (engaged) ENGAGED_COLOR else SEPARATED_COLOR
        g2.stroke = BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND)
        drawLink(g2, px, py, engine.getMotorRad(), armLen * 0.92)

        // Load link (thick).
        g2.color = LOAD_COLOR
        g2.stroke = BasicStroke(8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND)
        drawLink(g2, px, py, engine.getTrueLoadRad(), armLen)

        // Pivot.
        g2.color = Color.DARK_GRAY
        g2.fill(Ellipse2D.Double(px - 7.0, py - 7.0, 14.0, 14.0))

        // Load end knob.
        val end = endPoint(px, py, engine.getTrueLoadRad(), armLen)
        g2.color = LOAD_COLOR
        g2.fill(Ellipse2D.Double(end[0] - 9, end[1] - 9, 18.0, 18.0))

        drawLegend(g2, engaged)
    }

    private fun drawLink(g2: Graphics2D, px: Int, py: Int, angleRad: Double, len: Double) {
        val end = endPoint(px, py, angleRad, len)
        g2.draw(Line2D.Double(px.toDouble(), py.toDouble(), end[0], end[1]))
    }

    private fun drawStopTick(g2: Graphics2D, px: Int, py: Int, angleRad: Double, len: Double) {
        val a = endPoint(px, py, angleRad, len * 1.0)
        val b = endPoint(px, py, angleRad, len * 1.12)
        g2.stroke = BasicStroke(2f)
        g2.color = ARC_COLOR
        g2.draw(Line2D.Double(a[0], a[1], b[0], b[1]))
    }

    private fun drawLegend(g2: Graphics2D, engaged: Boolean) {
        val x = 12
        val y = 18
        val dy = 16
        g2.font = g2.font.deriveFont(11f)
        g2.color = LOAD_COLOR
        g2.drawString("━ load (true arm)", x, y)
        g2.color = if (engaged) ENGAGED_COLOR else SEPARATED_COLOR
        g2.drawString("━ motor side (" + (if (engaged) "engaged" else "SEPARATED") + ")", x, y + dy)
        g2.color = TARGET_COLOR
        g2.drawString("╌ target", x, y + 2 * dy)
    }

    companion object {
        private val LOAD_COLOR = Color(0x2C, 0x7B, 0xE1)
        private val ENGAGED_COLOR = Color(0x2E, 0xA0, 0x43)
        private val SEPARATED_COLOR = Color(0xD0, 0x3A, 0x2E)
        private val TARGET_COLOR = Color(0x88, 0x88, 0x88)
        private val ARC_COLOR = Color(0xBB, 0xBB, 0xBB)

        /**
         * Map a principal angle into `[min, max]` by adding/subtracting 2π, then clamp. Needed when
         * the hard-stop span goes past −π (e.g. Lineage A −224°…−45°).
         */
        @JvmStatic
        fun unwrapIntoRange(principalRad: Double, minRad: Double, maxRad: Double): Double {
            val mid = 0.5 * (minRad + maxRad)
            var best = principalRad
            var bestDist = abs(principalRad - mid)
            for (k in -1..1) {
                if (k == 0) continue
                val cand = principalRad + k * 2.0 * Math.PI
                val dist = abs(cand - mid)
                if (dist < bestDist) {
                    bestDist = dist
                    best = cand
                }
            }
            return max(minRad, min(maxRad, best))
        }

        private fun endPoint(px: Int, py: Int, angleRad: Double, len: Double): DoubleArray {
            val ex = px + len * cos(angleRad)
            val ey = py - len * sin(angleRad) // invert for screen coords
            return doubleArrayOf(ex, ey)
        }
    }
}
