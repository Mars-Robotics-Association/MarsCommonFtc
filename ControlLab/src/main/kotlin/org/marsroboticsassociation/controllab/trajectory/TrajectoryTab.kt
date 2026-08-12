package org.marsroboticsassociation.controllab.trajectory

import java.awt.BasicStroke
import java.awt.BorderLayout
import java.awt.Color
import java.awt.Dimension
import java.awt.Font
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.geom.Rectangle2D
import java.io.File
import java.nio.file.Path
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.LinkedHashMap
import javax.swing.BorderFactory
import javax.swing.Box
import javax.swing.BoxLayout
import javax.swing.JButton
import javax.swing.JComboBox
import javax.swing.JFileChooser
import javax.swing.JLabel
import javax.swing.JLayeredPane
import javax.swing.JOptionPane
import javax.swing.JPanel
import javax.swing.JSlider
import javax.swing.Timer
import javax.swing.filechooser.FileNameExtensionFilter
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round
import org.knowm.xchart.XChartPanel
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.internal.series.AxesChartSeries
import org.knowm.xchart.style.Styler
import org.knowm.xchart.style.markers.SeriesMarkers
import org.marsroboticsassociation.controllab.flywheel.EditableParamField
import org.marsroboticsassociation.controllib.motion.SCurveVelocity
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment

class TrajectoryTab : JPanel() {

    private var engine: TrajectoryEngine
    private val buffer = RollingBuffer(WINDOW_SECS, BUFFER_POINTS, 5)
    private var elapsedSec = 0.0
    private val exactHistorySegments = LinkedHashMap<String, MutableList<TrajectoryCurveSegment>>()
    private var activeExactPlan: TrajectorySvgModel? = null
    private var activeExactPlanStartSec = Double.NaN
    private var backEmfViolation = false
    private var violationRanges: MutableList<DoubleArray> = ArrayList()

    private lateinit var chart: XYChart
    private lateinit var chartPanel: XChartPanel<XYChart>
    private lateinit var layeredPane: JLayeredPane
    private lateinit var violationBandPanel: ViolationBandPanel

    private lateinit var typeCombo: JComboBox<TrajectoryType>
    private lateinit var limitPanel: JPanel

    // SCurvePosition sliders
    private var slVMax: JSlider? = null
    private var slAAccel: JSlider? = null
    private var slADecel: JSlider? = null
    private var slJMax: JSlider? = null
    private var lbVMax: JLabel? = null
    private var lbAAccel: JLabel? = null
    private var lbADecel: JLabel? = null
    private var lbJMax: JLabel? = null

    // SCurveVelocity sliders
    private var slAMax: JSlider? = null
    private var slJInc: JSlider? = null
    private var slJDec: JSlider? = null
    private var lbAMax: JLabel? = null
    private var lbJInc: JLabel? = null
    private var lbJDec: JLabel? = null

    // Back-EMF motor characterization (for SCurveVelocity)
    private var efKs: EditableParamField? = null
    private var efKv: EditableParamField? = null
    private var efKa: EditableParamField? = null
    private lateinit var bemfPanel: JPanel

    // Target sliders
    private lateinit var slTargetA: JSlider
    private lateinit var slTargetB: JSlider
    private lateinit var lbTargetA: JLabel
    private lateinit var lbTargetB: JLabel
    private lateinit var btnGoA: JButton
    private lateinit var btnGoB: JButton
    private lateinit var btnExportSvg: JButton
    private lateinit var btnReset: JButton
    private lateinit var btnAutoTune: JButton
    private lateinit var simTimer: Timer

    init {
        layout = BorderLayout()
        engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        buildChart()
        buildSidebar()
        buildTimer()
        onTypeChanged()
    }

    private fun buildChart() {
        chart =
            XYChartBuilder()
                .width(900)
                .height(600)
                .title("Trajectory")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Value")
                .build()
        chart.styler.markerSize = 0
        chart.styler.legendPosition = Styler.LegendPosition.InsideNW
        chart
            .addSeries("Position (units)", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
        chart
            .addSeries("Velocity (units/s)", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
        chart
            .addSeries("Acceleration (units/s\u00b2)", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
        chart
            .addSeries("Max Motor Accel (units/s\u00b2)", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
            .setLineStyle(
                BasicStroke(
                    1.5f,
                    BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,
                    10.0f,
                    floatArrayOf(4.0f, 4.0f),
                    0.0f,
                )
            )
        chart
            .addSeries("Target", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
            .setLineStyle(
                BasicStroke(
                    1.5f,
                    BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,
                    10.0f,
                    floatArrayOf(6.0f, 4.0f),
                    0.0f,
                )
            )
        chartPanel = XChartPanel(chart)

        layeredPane =
            object : JLayeredPane() {
                override fun getPreferredSize(): Dimension = Dimension(900, 600)

                override fun doLayout() {
                    var w = width
                    var h = height
                    if (w <= 0 || h <= 0) {
                        w = 900
                        h = 600
                    }
                    violationBandPanel.setBounds(0, 0, w, h)
                    chartPanel.setBounds(0, 0, w, h)
                    super.doLayout()
                }
            }

        violationBandPanel = ViolationBandPanel()
        violationBandPanel.isOpaque = false

        layeredPane.add(chartPanel, JLayeredPane.DEFAULT_LAYER)
        layeredPane.add(violationBandPanel, JLayeredPane.PALETTE_LAYER)

        add(layeredPane, BorderLayout.CENTER)
    }

    private inner class ViolationBandPanel : JPanel() {
        override fun paintComponent(g: Graphics) {
            super.paintComponent(g)
            if (violationRanges.isEmpty() || width <= 0 || height <= 0) return

            // Derive the plot area bounds from two known screen-coordinate mappings.
            // getScreenYFromChart(dataY) is linear: screenY = slope * dataY + intercept.
            // Two points let us solve for slope and intercept, then find the plot edges
            // (where dataY equals the axis min/max), but we don't know axis min/max either.
            // Instead, use reflection to access the package-private getPlot().getBounds().
            val plotBounds: Rectangle2D
            try {
                val m = chart.javaClass.superclass.getDeclaredMethod("getPlot")
                m.isAccessible = true
                val plot = m.invoke(chart)
                plotBounds = plot.javaClass.getMethod("getBounds").invoke(plot) as Rectangle2D
            } catch (_: Exception) {
                return // can't determine plot area
            }
            if (plotBounds.width <= 0) return

            val plotLeft = plotBounds.x.toInt()
            val plotRight = (plotBounds.x + plotBounds.width).toInt()
            val plotTop = plotBounds.y.toInt()
            val plotH = plotBounds.height.toInt()

            val g2 = g as Graphics2D
            g2.color = Color(180, 180, 180, 120)

            for (range in violationRanges) {
                var x1 = chart.getScreenXFromChart(range[0]).toInt()
                var x2 = chart.getScreenXFromChart(range[1]).toInt()

                x1 = max(plotLeft, x1)
                x2 = min(plotRight, x2)

                if (x2 > x1) {
                    g2.fillRect(x1, plotTop, x2 - x1, plotH)
                }
            }
        }
    }

    private fun buildSidebar() {
        val sidebar = JPanel()
        sidebar.layout = BoxLayout(sidebar, BoxLayout.Y_AXIS)
        sidebar.preferredSize = Dimension(SIDEBAR_WIDTH, 0)
        sidebar.border = BorderFactory.createEmptyBorder(8, 8, 8, 8)

        typeCombo = JComboBox(TrajectoryType.values())
        typeCombo.maximumSize = Dimension(Integer.MAX_VALUE, 28)
        sidebar.add(typeCombo)
        sidebar.add(Box.createVerticalStrut(12))

        sidebar.add(boldLabel("Limits"))
        sidebar.add(Box.createVerticalStrut(4))

        limitPanel = JPanel()
        limitPanel.layout = BoxLayout(limitPanel, BoxLayout.Y_AXIS)
        sidebar.add(limitPanel)
        sidebar.add(Box.createVerticalStrut(12))

        sidebar.add(boldLabel("Motor (back-EMF)"))
        sidebar.add(Box.createVerticalStrut(4))

        bemfPanel = JPanel()
        bemfPanel.layout = BoxLayout(bemfPanel, BoxLayout.Y_AXIS)
        buildBackEmfPanelIfNeeded()
        bemfPanel.isVisible = false
        sidebar.add(bemfPanel)

        sidebar.add(boldLabel("Targets"))
        sidebar.add(Box.createVerticalStrut(4))

        slTargetA = JSlider(0, 1000, l2s(-100.0, -200.0, 200.0))
        lbTargetA = JLabel()
        slTargetB = JSlider(0, 1000, l2s(100.0, -200.0, 200.0))
        lbTargetB = JLabel()

        slTargetA.putClientProperty("min", -200.0)
        slTargetA.putClientProperty("max", 200.0)
        slTargetB.putClientProperty("min", -200.0)
        slTargetB.putClientProperty("max", 200.0)

        slTargetA.addChangeListener { updateTargetLabel(lbTargetA, slTargetA, "A") }
        slTargetB.addChangeListener { updateTargetLabel(lbTargetB, slTargetB, "B") }
        updateTargetLabel(lbTargetA, slTargetA, "A")
        updateTargetLabel(lbTargetB, slTargetB, "B")

        sidebar.add(lbTargetA)
        sidebar.add(slTargetA)
        sidebar.add(Box.createVerticalStrut(4))
        sidebar.add(lbTargetB)
        sidebar.add(slTargetB)
        sidebar.add(Box.createVerticalStrut(12))

        btnGoA = JButton("\u2192 Go to A")
        btnGoB = JButton("\u2192 Go to B")
        btnExportSvg = JButton("Export SVG")
        btnGoA.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnGoB.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnExportSvg.maximumSize = Dimension(Integer.MAX_VALUE, 36)

        sidebar.add(btnGoA)
        sidebar.add(Box.createVerticalStrut(6))
        sidebar.add(btnGoB)
        sidebar.add(Box.createVerticalStrut(12))
        sidebar.add(btnExportSvg)

        btnReset = JButton("\u21bb Reset to 0")
        btnReset.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnReset.isVisible = false
        sidebar.add(Box.createVerticalStrut(8))
        sidebar.add(btnReset)

        btnAutoTune = JButton("Auto-Tune")
        btnAutoTune.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnAutoTune.isVisible = false
        btnAutoTune.toolTipText =
            "Find optimal jDec that maximizes speed without violating back-EMF"
        sidebar.add(Box.createVerticalStrut(8))
        sidebar.add(btnAutoTune)

        typeCombo.addActionListener { onTypeChanged() }
        btnGoA.addActionListener { onGoTo(slTargetA) }
        btnGoB.addActionListener { onGoTo(slTargetB) }
        btnExportSvg.addActionListener { onExportSvg() }
        btnReset.addActionListener { onReset() }
        btnAutoTune.addActionListener { onAutoTune() }

        add(sidebar, BorderLayout.WEST)
    }

    private fun buildPosSlidersIfNeeded() {
        if (slVMax != null) return
        slVMax = JSlider(0, 1000, l2s(72.0, 0.1, 500.0))
        lbVMax = JLabel()
        slAAccel = JSlider(0, 1000, l2s(52.0, 0.1, 500.0))
        lbAAccel = JLabel()
        slADecel = JSlider(0, 1000, l2s(52.0, 0.1, 500.0))
        lbADecel = JLabel()
        slJMax = JSlider(0, 1000, l2s(100.0, 1.0, 5000.0))
        lbJMax = JLabel()

        slVMax!!.addChangeListener {
            updateLabel(lbVMax!!, slVMax!!, "vMax", 0.1, 500.0)
            stagePosParams()
        }
        slAAccel!!.addChangeListener {
            updateLabel(lbAAccel!!, slAAccel!!, "aAccel", 0.1, 500.0)
            stagePosParams()
        }
        slADecel!!.addChangeListener {
            updateLabel(lbADecel!!, slADecel!!, "aDecel", 0.1, 500.0)
            stagePosParams()
        }
        slJMax!!.addChangeListener {
            updateLabel(lbJMax!!, slJMax!!, "jMax", 1.0, 5000.0)
            stagePosParams()
        }
        updateLabel(lbVMax!!, slVMax!!, "vMax", 0.1, 500.0)
        updateLabel(lbAAccel!!, slAAccel!!, "aAccel", 0.1, 500.0)
        updateLabel(lbADecel!!, slADecel!!, "aDecel", 0.1, 500.0)
        updateLabel(lbJMax!!, slJMax!!, "jMax", 1.0, 5000.0)
        stagePosParams()
    }

    private fun buildVelSlidersIfNeeded() {
        if (slAMax != null) return
        slAMax = JSlider(0, 1000, l2s(1197.0, 10.0, 5000.0))
        lbAMax = JLabel()
        slJInc = JSlider(0, 1000, l2s(2669.0, 10.0, 10000.0))
        lbJInc = JLabel()
        slJDec = JSlider(0, 1000, l2s(800.0, 10.0, 5000.0))
        lbJDec = JLabel()

        slAMax!!.addChangeListener {
            updateLabel(lbAMax!!, slAMax!!, "aMax", 10.0, 5000.0)
            stageVelParams()
            checkBackEmfViolation()
        }
        slJInc!!.addChangeListener {
            updateLabel(lbJInc!!, slJInc!!, "jInc", 10.0, 10000.0)
            stageVelParams()
            checkBackEmfViolation()
        }
        slJDec!!.addChangeListener {
            updateLabel(lbJDec!!, slJDec!!, "jDec", 10.0, 5000.0)
            stageVelParams()
            checkBackEmfViolation()
        }
        updateLabel(lbAMax!!, slAMax!!, "aMax", 10.0, 5000.0)
        updateLabel(lbJInc!!, slJInc!!, "jInc", 10.0, 10000.0)
        updateLabel(lbJDec!!, slJDec!!, "jDec", 10.0, 5000.0)
        stageVelParams()
    }

    private fun buildBackEmfPanelIfNeeded() {
        if (efKs != null) return
        efKs = EditableParamField("kS", 0.893, "%.3f", 0.0, 12.0) { checkBackEmfViolation() }
        efKv = EditableParamField("kV", 0.00475, "%.6f", 0.0, 1.0) { checkBackEmfViolation() }
        efKa = EditableParamField("kA", 0.00599, "%.6f", 1e-9, 1.0) { checkBackEmfViolation() }
        bemfPanel.add(efKs)
        bemfPanel.add(efKv)
        bemfPanel.add(efKa)
    }

    private fun stagePosParams() {
        engine.setPositionParams(
            s2l(slVMax!!.value, 0.1, 500.0),
            s2l(slAAccel!!.value, 0.1, 500.0),
            s2l(slADecel!!.value, 0.1, 500.0),
            s2l(slJMax!!.value, 1.0, 5000.0),
        )
    }

    private fun stageVelParams() {
        engine.setVelocityParams(
            s2l(slAMax!!.value, 10.0, 5000.0),
            s2l(slJInc!!.value, 10.0, 10000.0),
            s2l(slJDec!!.value, 10.0, 5000.0),
        )
    }

    private fun onTypeChanged() {
        val sel = typeCombo.selectedItem as TrajectoryType

        engine.switchType(sel)
        buffer.clear()
        elapsedSec = 0.0
        resetExactExportHistory()

        limitPanel.removeAll()
        when (sel) {
            TrajectoryType.SCURVE_POSITION,
            TrajectoryType.SIN_CURVE_POSITION -> {
                buildPosSlidersIfNeeded()
                addSliderRow(limitPanel, lbVMax!!, slVMax!!)
                addSliderRow(limitPanel, lbAAccel!!, slAAccel!!)
                addSliderRow(limitPanel, lbADecel!!, slADecel!!)
                addSliderRow(limitPanel, lbJMax!!, slJMax!!)
                bemfPanel.isVisible = false
                btnReset.isVisible = false
                btnAutoTune.isVisible = false
                updateTargetRange(-200.0, 200.0, -100.0, 100.0)
            }
            TrajectoryType.SCURVE_VELOCITY -> {
                buildVelSlidersIfNeeded()
                buildBackEmfPanelIfNeeded()
                addSliderRow(limitPanel, lbAMax!!, slAMax!!)
                addSliderRow(limitPanel, lbJInc!!, slJInc!!)
                addSliderRow(limitPanel, lbJDec!!, slJDec!!)
                bemfPanel.isVisible = true
                btnReset.isVisible = true
                btnAutoTune.isVisible = true
                val ks = efKs!!.value
                val kv = efKv!!.value
                val maxV = if (kv > 0) (MOTOR_VOLTAGE - ks) / kv else 6000.0
                updateTargetRange(0.0, maxV, min(2000.0, maxV), 0.0)
                checkBackEmfViolation()
            }
        }
        limitPanel.revalidate()
        limitPanel.repaint()

        chart.seriesMap["Position (units)"]!!.setEnabled(engine.hasPosition())
        refreshExportButtonState()
        chartPanel.repaint()
    }

    private fun updateTargetRange(min: Double, max: Double, defA: Double, defB: Double) {
        slTargetA.putClientProperty("min", min)
        slTargetA.putClientProperty("max", max)
        slTargetB.putClientProperty("min", min)
        slTargetB.putClientProperty("max", max)
        slTargetA.value = l2s(defA, min, max)
        slTargetB.value = l2s(defB, min, max)
        updateTargetLabel(lbTargetA, slTargetA, "A")
        updateTargetLabel(lbTargetB, slTargetB, "B")
    }

    private fun targetSliderValue(sl: JSlider): Double {
        val minObj = sl.getClientProperty("min")
        val maxObj = sl.getClientProperty("max")
        val min = if (minObj != null) minObj as Double else -200.0
        val max = if (maxObj != null) maxObj as Double else 200.0
        return s2l(sl.value, min, max)
    }

    private fun onGoTo(targetSlider: JSlider) {
        val targetValue = targetSliderValue(targetSlider)
        commitActiveExactPlanThrough(elapsedSec)
        engine.applyParamsAndGoTo(targetValue)
        startActiveExactPlan()
        recordCurrentSample()
        refreshExportButtonState()
        checkBackEmfViolation()
    }

    private fun onReset() {
        elapsedSec = 0.0
        buffer.clear()
        engine.reset()
        resetExactExportHistory()
        recordCurrentSample()
        refreshExportButtonState()
    }

    private fun onAutoTune() {
        val jInc = s2l(slJInc!!.value, 10.0, 10000.0)
        val targetV = engine.getTarget()

        if (targetV <= 0) {
            JOptionPane.showMessageDialog(
                this,
                "Set a target velocity first",
                "Auto-Tune",
                JOptionPane.WARNING_MESSAGE,
            )
            return
        }

        val ks = efKs!!.value
        val kv = efKv!!.value
        val ka = efKa!!.value

        if (ka <= 0) {
            JOptionPane.showMessageDialog(
                this,
                "kA must be positive for back-EMF calculation",
                "Auto-Tune",
                JOptionPane.WARNING_MESSAGE,
            )
            return
        }

        val aMax = SCurveVelocity.findMaxAMax(0.0, targetV, jInc, MOTOR_VOLTAGE, ks, kv, ka)
        val jDec =
            SCurveVelocity.findMaxJDec(0.0, targetV, 0.0, aMax, jInc, MOTOR_VOLTAGE, ks, kv, ka)

        slAMax!!.value = l2s(aMax, 10.0, 5000.0)
        updateLabel(lbAMax!!, slAMax!!, "aMax", 10.0, 5000.0)
        slJDec!!.value = l2s(jDec, 10.0, 5000.0)
        updateLabel(lbJDec!!, slJDec!!, "jDec", 10.0, 5000.0)
        stageVelParams()

        checkBackEmfViolation()
    }

    private fun buildTimer() {
        simTimer = Timer(TIMER_MS) { onTick() }
        simTimer.start()
    }

    private fun onTick() {
        if (!engine.isMoving()) return
        engine.tick()
        elapsedSec += TrajectoryEngine.CYCLE_S
        if (!engine.isMoving()) {
            commitActiveExactPlanThrough(elapsedSec)
        }
        recordCurrentSample()
        refreshExportButtonState()
    }

    private fun recordCurrentSample() {
        val velocity = engine.getVelocity()
        val maxMotorAccel = maxMotorAcceleration(velocity)
        buffer.add(
            elapsedSec,
            engine.getPosition(),
            engine.getVelocity(),
            engine.getAcceleration(),
            engine.getTarget(),
            maxMotorAccel,
        )
        val times = buffer.getTimes()
        chart.updateXYSeries("Position (units)", times, buffer.getPositions(), null)
        chart.updateXYSeries("Velocity (units/s)", times, buffer.getVelocities(), null)
        chart.updateXYSeries(
            "Acceleration (units/s\u00b2)",
            times,
            buffer.getAccelerations(),
            null,
        )
        val showMaxMotorAccel = engine.getType() == TrajectoryType.SCURVE_VELOCITY
        chart.seriesMap["Max Motor Accel (units/s\u00b2)"]!!.setEnabled(showMaxMotorAccel)
        if (showMaxMotorAccel) {
            chart.updateXYSeries(
                "Max Motor Accel (units/s\u00b2)",
                times,
                buffer.getMaxMotorAccels(),
                null,
            )
        }
        chart.updateXYSeries("Target", times, buffer.getTargets(), null)
        chartPanel.repaint()
        if (times.size >= 2) {
            checkBackEmfViolation()
        }
    }

    private fun refreshExportButtonState() {
        val visible = engine.supportsExactSvgExport()
        btnExportSvg.isVisible = visible
        btnExportSvg.isEnabled = visible && !engine.isMoving()
        btnExportSvg.toolTipText =
            if (visible) {
                if (engine.isMoving()) {
                    "Export becomes available once the graph settles"
                } else {
                    "Export an exact SVG of the current trajectory"
                }
            } else {
                null
            }
        revalidate()
        repaint()
    }

    private fun onExportSvg() {
        if (engine.isMoving()) {
            JOptionPane.showMessageDialog(
                this,
                "Wait for the trajectory to settle before exporting.",
                "Trajectory Still Running",
                JOptionPane.INFORMATION_MESSAGE,
            )
            return
        }

        val model = styledSvgModel()
        if (model == null) {
            JOptionPane.showMessageDialog(
                this,
                "SVG export is only available for SCurvePosition and SCurveVelocity.",
                "Export Unavailable",
                JOptionPane.WARNING_MESSAGE,
            )
            return
        }

        val chooser = JFileChooser(defaultDownloadsDir())
        chooser.dialogTitle = "Save trajectory SVG"
        chooser.fileFilter = FileNameExtensionFilter("SVG Files (*.svg)", "svg")
        chooser.selectedFile = File(defaultDownloadsDir(), defaultSvgFilename())
        val ret = chooser.showSaveDialog(this)
        if (ret != JFileChooser.APPROVE_OPTION) return

        var out = chooser.selectedFile
        if (!out.name.lowercase().endsWith(".svg")) {
            out = File(out.parentFile, out.name + ".svg")
        }
        if (out.exists()) {
            val choice =
                JOptionPane.showConfirmDialog(
                    this,
                    "Replace existing file?\n" + out.absolutePath,
                    "Confirm Replace",
                    JOptionPane.YES_NO_OPTION,
                    JOptionPane.WARNING_MESSAGE,
                )
            if (choice != JOptionPane.YES_OPTION) return
        }

        try {
            TrajectorySvgExporter.export(Path.of(out.absolutePath), model)
            JOptionPane.showMessageDialog(this, "Exported to: " + out.absolutePath)
        } catch (ex: Exception) {
            JOptionPane.showMessageDialog(
                this,
                "Export failed: " + ex.message,
                "Export Failed",
                JOptionPane.ERROR_MESSAGE,
            )
        }
    }

    private fun styledSvgModel(): TrajectorySvgModel? {
        val base = visibleExactSvgModel() ?: return null

        val palette = chart.styler.seriesColors

        val styledSeries =
            (0 until base.series.size).map { index ->
                val series = base.series[index]
                val chartSeries = chart.seriesMap[series.label] as? AxesChartSeries
                if (chartSeries == null) return@map series
                val stroke =
                    if (chartSeries.lineStyle != null) chartSeries.lineStyle else series.stroke
                val strokeWidth =
                    if (chartSeries.lineWidth > 0) chartSeries.lineWidth else series.strokeWidth
                val color =
                    if (chartSeries.lineColor != null) {
                        chartSeries.lineColor
                    } else {
                        paletteColor(palette, index, series.color)
                    }
                TrajectorySvgSeries(series.label, series.segments, color, strokeWidth, stroke)
            }

        return TrajectorySvgModel(base.xMin, base.xMax, base.minY, base.maxY, styledSeries)
    }

    private fun resetExactExportHistory() {
        exactHistorySegments.clear()
        activeExactPlan = null
        activeExactPlanStartSec = Double.NaN
    }

    private fun startActiveExactPlan() {
        if (!engine.supportsExactSvgExport()) {
            activeExactPlan = null
            activeExactPlanStartSec = Double.NaN
            return
        }
        activeExactPlan = engine.buildExactSvgModel()
        activeExactPlanStartSec = elapsedSec
    }

    private fun commitActiveExactPlanThrough(absoluteEndSec: Double) {
        val plan = activeExactPlan
        if (plan == null || activeExactPlanStartSec.isNaN()) return
        val duration = min(plan.xMax - plan.xMin, absoluteEndSec - activeExactPlanStartSec)
        if (duration > 0) {
            appendPlanSegments(exactHistorySegments, plan, activeExactPlanStartSec, 0.0, duration)
        }
        activeExactPlan = null
        activeExactPlanStartSec = Double.NaN
    }

    private fun visibleExactSvgModel(): TrajectorySvgModel? {
        if (!engine.supportsExactSvgExport()) return null

        val combined = LinkedHashMap<String, MutableList<TrajectoryCurveSegment>>()
        for ((key, value) in exactHistorySegments) {
            combined[key] = ArrayList(value)
        }

        val plan = activeExactPlan
        if (plan != null && !activeExactPlanStartSec.isNaN()) {
            val duration =
                min(
                    plan.xMax - plan.xMin,
                    max(0.0, elapsedSec - activeExactPlanStartSec),
                )
            appendPlanSegments(combined, plan, activeExactPlanStartSec, 0.0, duration)
        }

        val times = buffer.getTimes()
        if (times.isEmpty()) return null
        val xMin = times[0]
        var xMax = times[times.size - 1]
        if (xMax <= xMin) xMax = xMin + 1e-9

        val series = ArrayList<TrajectorySvgSeries>()
        for ((key, value) in combined) {
            val clipped = ArrayList<TrajectoryCurveSegment>()
            for (segment in value) {
                val clippedSegment = segment.clippedTo(xMin, xMax)
                if (clippedSegment != null) {
                    clipped.add(clippedSegment)
                }
            }
            if (clipped.isNotEmpty()) {
                series.add(TrajectorySvgSeries(key, clipped, Color.BLACK, 2.0f, null))
            }
        }

        if (series.isEmpty()) return null
        return TrajectorySvgModel(xMin, xMax, minY(series), maxY(series), series)
    }

    private fun maxMotorAcceleration(velocity: Double): Double {
        val ks = efKs!!.value
        val kv = efKv!!.value
        val ka = efKa!!.value
        if (ka <= 0) return Double.POSITIVE_INFINITY
        val availableVoltage = MOTOR_VOLTAGE - ks - kv * abs(velocity)
        if (availableVoltage <= 0) return 0.0
        return availableVoltage / ka
    }

    private fun checkBackEmfViolation() {
        if (engine.getType() != TrajectoryType.SCURVE_VELOCITY) {
            backEmfViolation = false
            violationRanges.clear()
            updateViolationBandPanel()
            return
        }

        val ka = efKa!!.value
        val targetV = engine.getTarget()

        if (targetV <= 0) {
            backEmfViolation = false
            violationRanges.clear()
            updateViolationBandPanel()
            return
        }

        if (ka <= 0) {
            backEmfViolation = false
            violationRanges.clear()
            updateViolationBandPanel()
            return
        }

        val times = buffer.getTimes()
        val velocities = buffer.getVelocities()
        val accelerations = buffer.getAccelerations()

        violationRanges.clear()

        if (times.size < 2) {
            updateViolationBandPanel()
            return
        }

        var inViolation = false
        var rangeStart = 0.0

        val tolerance = 0.0
        for (i in times.indices) {
            val t = times[i]
            val v = velocities[i]
            val a = accelerations[i]

            val motorAMax = maxMotorAcceleration(v)
            val isViolating = a > motorAMax + tolerance

            if (isViolating && !inViolation) {
                rangeStart = t
                inViolation = true
            } else if (!isViolating && inViolation) {
                violationRanges.add(doubleArrayOf(rangeStart, t))
                inViolation = false
            }
        }

        if (inViolation && times.isNotEmpty()) {
            violationRanges.add(doubleArrayOf(rangeStart, times[times.size - 1]))
        }

        backEmfViolation = violationRanges.isNotEmpty()

        updateViolationBandPanel()
    }

    private fun updateViolationBandPanel() {
        violationBandPanel.repaint()
    }

    /** Release native resources held by the engine. Call on application shutdown. */
    fun disposeEngine() {
        simTimer.stop()
        engine.dispose()
    }

    private fun defaultSvgFilename(): String {
        val timestamp = LocalDateTime.now().format(EXPORT_TIMESTAMP)
        return when (engine.getType()) {
            TrajectoryType.SCURVE_POSITION -> "scurve-position-trajectory-$timestamp.svg"
            TrajectoryType.SCURVE_VELOCITY -> "scurve-velocity-trajectory-$timestamp.svg"
            else -> "trajectory-$timestamp.svg"
        }
    }

    companion object {
        private const val SIDEBAR_WIDTH = 300
        private const val TIMER_MS = 20
        private const val BUFFER_POINTS = 500
        private const val WINDOW_SECS = 10.0
        private val EXPORT_TIMESTAMP = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss")
        private const val MOTOR_VOLTAGE = 12.0

        @JvmStatic
        fun s2l(v: Int, min: Double, max: Double): Double {
            return min + (max - min) * v / 1000.0
        }

        @JvmStatic
        fun l2s(v: Double, min: Double, max: Double): Int {
            return round((v - min) / (max - min) * 1000.0).toInt()
        }

        private fun paletteColor(palette: Array<Color>?, index: Int, fallback: Color): Color {
            if (palette == null || palette.isEmpty()) return fallback
            val color = palette[Math.floorMod(index, palette.size)]
            return color ?: fallback
        }

        private fun appendPlanSegments(
            target: MutableMap<String, MutableList<TrajectoryCurveSegment>>,
            plan: TrajectorySvgModel,
            absoluteStartSec: Double,
            relativeStartSec: Double,
            relativeEndSec: Double,
        ) {
            if (relativeEndSec <= relativeStartSec) return
            for (series in plan.series) {
                val dst = target.computeIfAbsent(series.label) { ArrayList() }
                for (segment in series.segments) {
                    val clipped = segment.clippedTo(relativeStartSec, relativeEndSec)
                    if (clipped != null) {
                        dst.add(clipped.shiftedBy(absoluteStartSec))
                    }
                }
            }
        }

        private fun minY(seriesList: List<TrajectorySvgSeries>): Double {
            var min = Double.POSITIVE_INFINITY
            for (series in seriesList) {
                for (segment in series.segments) {
                    min = minOf(min, segment.minValue())
                }
            }
            return if (min.isFinite()) min else -1.0
        }

        private fun maxY(seriesList: List<TrajectorySvgSeries>): Double {
            var max = Double.NEGATIVE_INFINITY
            for (series in seriesList) {
                for (segment in series.segments) {
                    max = maxOf(max, segment.maxValue())
                }
            }
            return if (max.isFinite()) max else 1.0
        }

        private fun defaultDownloadsDir(): File {
            return File(System.getProperty("user.home"), "Downloads")
        }

        private fun addSliderRow(panel: JPanel, label: JLabel, slider: JSlider) {
            panel.add(label)
            panel.add(slider)
            panel.add(Box.createVerticalStrut(4))
        }

        private fun boldLabel(text: String): JLabel {
            val l = JLabel(text)
            l.font = l.font.deriveFont(Font.BOLD)
            return l
        }

        private fun updateLabel(lbl: JLabel, sl: JSlider, name: String, min: Double, max: Double) {
            lbl.text = String.format("%s: %.2f", name, s2l(sl.value, min, max))
        }

        private fun updateTargetLabel(lbl: JLabel, sl: JSlider, name: String) {
            val minObj = sl.getClientProperty("min")
            val maxObj = sl.getClientProperty("max")
            val min = if (minObj != null) minObj as Double else -200.0
            val max = if (maxObj != null) maxObj as Double else 200.0
            lbl.text = String.format("Target %s: %.1f", name, s2l(sl.value, min, max))
        }
    }
}
