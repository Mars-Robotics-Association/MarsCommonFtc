package org.marsroboticsassociation.controllab.flywheel

import java.awt.BasicStroke
import java.awt.BorderLayout
import java.awt.Dimension
import java.awt.Font
import javax.swing.BorderFactory
import javax.swing.Box
import javax.swing.BoxLayout
import javax.swing.JButton
import javax.swing.JComboBox
import javax.swing.JLabel
import javax.swing.JOptionPane
import javax.swing.JPanel
import javax.swing.Timer
import org.knowm.xchart.XChartPanel
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.Styler
import org.knowm.xchart.style.markers.SeriesMarkers
import org.marsroboticsassociation.controllab.trajectory.RollingBuffer
import org.marsroboticsassociation.controllib.control.FlywheelSimple

class FlywheelTab : JPanel() {

    private val engine: FlywheelEngine
    private val buffer = RollingBuffer(WINDOW_SECS, BUFFER_POINTS, 5)

    private lateinit var chart: XYChart
    private lateinit var chartPanel: XChartPanel<XYChart>

    private lateinit var typeCombo: JComboBox<FlywheelControllerType>
    private lateinit var paramPanel: JPanel
    private lateinit var plantPanel: JPanel
    private lateinit var simplePanel: JPanel
    private lateinit var pfPanel: JPanel
    private lateinit var ssPanel: JPanel

    // Editable fields
    private lateinit var efKS: EditableParamField
    private lateinit var efKV: EditableParamField
    private lateinit var efKA: EditableParamField
    private lateinit var efKP: EditableParamField
    private lateinit var efCutoff: EditableParamField
    private lateinit var efPlantKS: EditableParamField
    private lateinit var efPlantKV: EditableParamField
    private lateinit var efPlantKA: EditableParamField
    private lateinit var efTargetA: EditableParamField
    private lateinit var efTargetB: EditableParamField
    private lateinit var efSimpleMaxAccel: EditableParamField
    private lateinit var efPFAccelMax: EditableParamField
    private lateinit var efPFRisingJerk: EditableParamField
    private lateinit var efPFFallingJerk: EditableParamField
    private lateinit var efSSModelStdDev: EditableParamField
    private lateinit var efSSMeasurementStdDev: EditableParamField

    private var targetA = 1000.0
    private var targetB = 2000.0

    private lateinit var btnGoA: JButton
    private lateinit var btnGoB: JButton
    private lateinit var btnCoast: JButton
    private lateinit var btnNewChallenge: JButton
    private lateinit var btnRevealPlant: JButton
    private lateinit var simTimer: Timer

    init {
        layout = BorderLayout()
        engine = FlywheelEngine(FlywheelControllerType.FLYWHEEL_SIMPLE)
        buildChart()
        buildSidebar()
        buildTimer()
    }

    private fun buildChart() {
        chart =
            XYChartBuilder()
                .width(900)
                .height(600)
                .title("Flywheel Simulation")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Velocity (TPS)")
                .build()
        chart.styler.markerSize = 0
        chart.styler.legendPosition = Styler.LegendPosition.InsideNW

        chart
            .addSeries("True Velocity", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
            .setEnabled(false)
        chart
            .addSeries("Measured Velocity", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
        chart
            .addSeries("Filtered Velocity", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
        chart
            .addSeries("Profiled Velocity", doubleArrayOf(0.0), doubleArrayOf(0.0))
            .setMarker(SeriesMarkers.NONE)
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
        add(chartPanel, BorderLayout.CENTER)
    }

    private fun buildSidebar() {
        val sidebar = JPanel()
        sidebar.layout = BoxLayout(sidebar, BoxLayout.Y_AXIS)
        sidebar.preferredSize = Dimension(SIDEBAR_WIDTH, 0)
        sidebar.border = BorderFactory.createEmptyBorder(8, 8, 8, 8)

        typeCombo = JComboBox(FlywheelControllerType.values())
        typeCombo.maximumSize = Dimension(Integer.MAX_VALUE, 28)
        typeCombo.addActionListener {
            engine.setType(typeCombo.selectedItem as FlywheelControllerType)
            updateControllerPanel()
            buffer.clear()
        }
        sidebar.add(typeCombo)
        sidebar.add(Box.createVerticalStrut(12))

        btnNewChallenge = JButton("New Challenge")
        btnNewChallenge.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnNewChallenge.addActionListener { onNewChallenge() }
        sidebar.add(btnNewChallenge)
        sidebar.add(Box.createVerticalStrut(12))

        sidebar.add(boldLabel("Tuning"))
        sidebar.add(Box.createVerticalStrut(4))

        paramPanel = JPanel()
        paramPanel.layout = BoxLayout(paramPanel, BoxLayout.Y_AXIS)

        efKS =
            EditableParamField("kS", engine.getKS(), "%.3f", 0.0, Double.POSITIVE_INFINITY) {
                updateParams()
            }
        efKV =
            EditableParamField("kV", engine.getKV(), "%.6f", 0.0, Double.POSITIVE_INFINITY) {
                updateParams()
            }
        efKA =
            EditableParamField("kA", engine.getKA(), "%.6f", 0.0, Double.POSITIVE_INFINITY) {
                updateParams()
            }
        efKP =
            EditableParamField("kP", engine.getKP(), "%.4f", 0.0, Double.POSITIVE_INFINITY) {
                updateParams()
            }
        efCutoff =
            EditableParamField("Cutoff (Hz)", engine.getVelLpfCutoffHz(), "%.1f", 0.1, 100.0) {
                updateParams()
            }

        paramPanel.add(efKS)
        paramPanel.add(efKV)
        paramPanel.add(efKA)
        paramPanel.add(efKP)
        paramPanel.add(efCutoff)

        sidebar.add(paramPanel)
        sidebar.add(Box.createVerticalStrut(12))

        sidebar.add(boldLabel("Controller"))
        sidebar.add(Box.createVerticalStrut(4))

        buildControllerPanels()

        sidebar.add(simplePanel)
        sidebar.add(pfPanel)
        sidebar.add(ssPanel)
        updateControllerPanel()
        sidebar.add(Box.createVerticalStrut(12))

        btnRevealPlant = JButton("Show Simulator Plant")
        btnRevealPlant.maximumSize = Dimension(Integer.MAX_VALUE, 24)
        sidebar.add(btnRevealPlant)
        sidebar.add(Box.createVerticalStrut(4))

        plantPanel = JPanel()
        plantPanel.layout = BoxLayout(plantPanel, BoxLayout.Y_AXIS)
        plantPanel.isVisible = false

        efPlantKS =
            EditableParamField("Plant kS", engine.getPlantKS(), "%.3f", 0.0, 12.0) {
                updatePlantParams()
            }
        efPlantKV =
            EditableParamField("Plant kV", engine.getPlantKV(), "%.6f", 0.0, 1.0) {
                updatePlantParams()
            }
        efPlantKA =
            EditableParamField("Plant kA", engine.getPlantKA(), "%.6f", 1e-9, 1.0) {
                updatePlantParams()
            }
        plantPanel.add(efPlantKS)
        plantPanel.add(efPlantKV)
        plantPanel.add(efPlantKA)

        sidebar.add(plantPanel)

        btnRevealPlant.addActionListener {
            val currentlyVisible = plantPanel.isVisible
            if (!currentlyVisible) {
                // Revealing: confirm first
                val result =
                    JOptionPane.showConfirmDialog(
                        this,
                        "Are you sure you want to reveal the plant values?",
                        "Show Plant",
                        JOptionPane.YES_NO_OPTION,
                    )
                if (result != JOptionPane.YES_OPTION) return@addActionListener
            }
            val visible = !currentlyVisible
            plantPanel.isVisible = visible
            chart.seriesMap["True Velocity"]!!.setEnabled(visible)
            btnRevealPlant.text = if (visible) "Hide Simulator Plant" else "Show Simulator Plant"
            sidebar.revalidate()
        }

        sidebar.add(Box.createVerticalStrut(12))

        sidebar.add(boldLabel("Targets"))
        sidebar.add(Box.createVerticalStrut(4))

        efTargetA =
            EditableParamField("Target A", targetA, "%.0f", 0.0, 5000.0) { v -> targetA = v }
        efTargetB =
            EditableParamField("Target B", targetB, "%.0f", 0.0, 5000.0) { v -> targetB = v }

        sidebar.add(efTargetA)
        sidebar.add(efTargetB)
        sidebar.add(Box.createVerticalStrut(12))

        btnGoA = JButton("\u2192 Go to A")
        btnGoB = JButton("\u2192 Go to B")
        btnCoast = JButton("\u2741 Coast")
        btnGoA.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnGoB.maximumSize = Dimension(Integer.MAX_VALUE, 36)
        btnCoast.maximumSize = Dimension(Integer.MAX_VALUE, 36)

        btnGoA.addActionListener { engine.setTarget(targetA) }
        btnGoB.addActionListener { engine.setTarget(targetB) }
        btnCoast.addActionListener { engine.setTarget(0.0) }

        sidebar.add(btnGoA)
        sidebar.add(Box.createVerticalStrut(6))
        sidebar.add(btnGoB)
        sidebar.add(Box.createVerticalStrut(6))
        sidebar.add(btnCoast)

        add(sidebar, BorderLayout.WEST)
    }

    private fun onNewChallenge() {
        engine.newChallenge()
        buffer.clear()

        // Update tuning param fields to show zeroed values
        efKS.setValue(0.0, "%.3f")
        efKV.setValue(0.0, "%.6f")
        efKA.setValue(0.0, "%.6f")
        efKP.setValue(0.0, "%.4f")

        // Update plant fields to show new (hidden) values
        efPlantKS.setValue(engine.getPlantKS(), "%.3f")
        efPlantKV.setValue(engine.getPlantKV(), "%.6f")
        efPlantKA.setValue(engine.getPlantKA(), "%.6f")

        // Update profile param fields
        efPFAccelMax.setValue(engine.getPFBAccelMax(), "%.0f")
        efPFFallingJerk.setValue(engine.getPFJerkDecreasing(), "%.0f")

        // Hide plant panel so new values aren't immediately visible
        plantPanel.isVisible = false
        chart.seriesMap["True Velocity"]!!.setEnabled(false)
        btnRevealPlant.text = "Show Simulator Plant"
    }

    private fun updateParams() {
        engine.setParams(
            efKV.value,
            efKA.value,
            efKS.value,
            efKP.value,
            efCutoff.value,
        )
    }

    private fun updatePlantParams() {
        engine.setPlantParams(efPlantKV.value, efPlantKA.value, efPlantKS.value)
    }

    private fun buildTimer() {
        simTimer = Timer(TIMER_MS) { onTick() }
        simTimer.start()
    }

    private fun buildControllerPanels() {
        val initialMaxAccel = FlywheelSimple.PARAMS.maxAccel

        efSimpleMaxAccel =
            EditableParamField(
                "Max Accel (TPS\u00B2)",
                initialMaxAccel,
                "%.0f",
                0.0,
                initialMaxAccel,
            ) { v ->
                engine.setSimpleParams(v)
            }

        simplePanel = JPanel()
        simplePanel.layout = BoxLayout(simplePanel, BoxLayout.Y_AXIS)
        simplePanel.add(efSimpleMaxAccel)

        efPFAccelMax =
            EditableParamField(
                "Accel Max (TPS\u00B2)",
                engine.getPFBAccelMax(),
                "%.0f",
                0.0,
                5000.0,
            ) { v ->
                engine.setPFParams(v, engine.getPFJerkIncreasing(), engine.getPFJerkDecreasing())
            }
        efPFRisingJerk =
            EditableParamField(
                "Rising Jerk (TPS\u00B3)",
                engine.getPFJerkIncreasing(),
                "%.0f",
                0.0,
                10000.0,
            ) { v ->
                engine.setPFParams(engine.getPFBAccelMax(), v, engine.getPFJerkDecreasing())
            }
        efPFFallingJerk =
            EditableParamField(
                "Falling Jerk (TPS\u00B3)",
                engine.getPFJerkDecreasing(),
                "%.0f",
                0.0,
                10000.0,
            ) { v ->
                engine.setPFParams(engine.getPFBAccelMax(), engine.getPFJerkIncreasing(), v)
            }

        pfPanel = JPanel()
        pfPanel.layout = BoxLayout(pfPanel, BoxLayout.Y_AXIS)
        pfPanel.add(efPFAccelMax)
        pfPanel.add(efPFRisingJerk)
        pfPanel.add(efPFFallingJerk)

        efSSModelStdDev =
            EditableParamField(
                "Model StdDev (rad/s)",
                engine.getSSModelStdDev(),
                "%.4f",
                0.001,
                100.0,
            ) { v ->
                engine.setSSParams(v, engine.getSSMeasurementStdDev())
            }
        efSSMeasurementStdDev =
            EditableParamField(
                "Meas StdDev (rad/s)",
                engine.getSSMeasurementStdDev(),
                "%.4f",
                0.001,
                100.0,
            ) { v ->
                engine.setSSParams(engine.getSSModelStdDev(), v)
            }

        ssPanel = JPanel()
        ssPanel.layout = BoxLayout(ssPanel, BoxLayout.Y_AXIS)
        ssPanel.add(efSSModelStdDev)
        ssPanel.add(efSSMeasurementStdDev)
    }

    private fun updateControllerPanel() {
        val type = typeCombo.selectedItem as FlywheelControllerType
        simplePanel.isVisible = type == FlywheelControllerType.FLYWHEEL_SIMPLE
        pfPanel.isVisible = type == FlywheelControllerType.VELOCITY_MOTOR_PF
        ssPanel.isVisible = type == FlywheelControllerType.FLYWHEEL_STATE_SPACE
        btnNewChallenge.isVisible = type == FlywheelControllerType.VELOCITY_MOTOR_PF
        revalidate()
    }

    private fun onTick() {
        engine.tick()
        recordCurrentSample()
    }

    private fun recordCurrentSample() {
        buffer.add(
            engine.elapsedSec,
            engine.getTrueVelocity(),
            engine.getMeasuredVelocity(),
            engine.getFilteredVelocity(),
            engine.getProfiledVelocity(),
            engine.target,
        )

        val times = buffer.getTimes()
        if (times.size < 2) return

        chart.updateXYSeries("True Velocity", times, buffer.getData(0), null)
        chart.updateXYSeries("Measured Velocity", times, buffer.getData(1), null)
        chart.updateXYSeries("Filtered Velocity", times, buffer.getData(2), null)
        chart.updateXYSeries("Profiled Velocity", times, buffer.getData(3), null)
        chart.updateXYSeries("Target", times, buffer.getData(4), null)
        chartPanel.repaint()
    }

    fun dispose() {
        simTimer.stop()
    }

    companion object {
        private const val SIDEBAR_WIDTH = 260
        private const val TIMER_MS = 20
        private const val BUFFER_POINTS = 500
        private const val WINDOW_SECS = 5.0

        private fun boldLabel(text: String): JLabel {
            val l = JLabel(text)
            l.font = l.font.deriveFont(Font.BOLD)
            return l
        }
    }
}
