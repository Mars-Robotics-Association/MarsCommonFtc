package org.marsroboticsassociation.controllab.arm

import java.awt.BasicStroke
import java.awt.BorderLayout
import java.awt.Component
import java.awt.Dimension
import java.awt.Font
import java.nio.file.Paths
import java.util.concurrent.ExecutionException
import javax.swing.BorderFactory
import javax.swing.Box
import javax.swing.BoxLayout
import javax.swing.JButton
import javax.swing.JComboBox
import javax.swing.JComponent
import javax.swing.JLabel
import javax.swing.JOptionPane
import javax.swing.JPanel
import javax.swing.JScrollPane
import javax.swing.JSlider
import javax.swing.SwingConstants
import javax.swing.SwingWorker
import javax.swing.Timer
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.roundToInt
import org.knowm.xchart.XChartPanel
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.Styler
import org.knowm.xchart.style.markers.SeriesMarkers
import org.marsroboticsassociation.controllab.flywheel.EditableParamField
import org.marsroboticsassociation.controllab.trajectory.RollingBuffer
import org.marsroboticsassociation.controllib.mechanism.FeedbackGainSynthesis

/**
 * The Arm tab: an animated [ArmCanvas], a flywheel-style XChart time series, a param sidebar of
 * [EditableParamField]s, and a backlash-focused metrics readout, all ticked by a Swing timer at
 * roughly real time. Mirrors the structure of `FlywheelTab`.
 */
class ArmTab : JPanel(BorderLayout()) {
    private val engine = ArmEngine(ArmControllerType.ARM_PD)
    private val buffer = RollingBuffer(WINDOW_SECS, BUFFER_POINTS, NUM_STREAMS)

    private lateinit var chart: XYChart
    private lateinit var chartPanel: XChartPanel<XYChart>
    private lateinit var canvas: ArmCanvas
    private lateinit var metricsLabel: JLabel

    private lateinit var typeCombo: JComboBox<ArmControllerType>
    private lateinit var plantCombo: JComboBox<ArmEngine.PlantKind>
    private lateinit var targetSlider: JSlider
    private lateinit var targetLabel: JLabel
    private var updatingSlider = false

    // Controller panels
    private lateinit var pdPanel: JPanel
    private lateinit var lqrPanel: JPanel
    private lateinit var mechPanel: JPanel
    private lateinit var ffPanel: JPanel
    private lateinit var efKP: EditableParamField
    private lateinit var efKD: EditableParamField
    private lateinit var efQPos: EditableParamField
    private lateinit var efQVel: EditableParamField
    private lateinit var efR: EditableParamField
    private lateinit var efMkP: EditableParamField
    private lateinit var efMkI: EditableParamField
    private lateinit var efMkD: EditableParamField
    private lateinit var efMkS: EditableParamField
    private lateinit var efMkV: EditableParamField
    private lateinit var efMkA: EditableParamField
    private lateinit var efMkCos: EditableParamField
    private lateinit var efMkSin: EditableParamField
    private lateinit var efMvMax: EditableParamField
    private lateinit var efMaMax: EditableParamField
    private lateinit var efMdMax: EditableParamField
    private lateinit var efMjMax: EditableParamField
    private lateinit var efOmegaN: EditableParamField
    private lateinit var efZeta: EditableParamField
    private lateinit var efFfKs: EditableParamField
    private lateinit var efFfKg: EditableParamField
    private lateinit var efFfKv: EditableParamField
    private lateinit var efFfKa: EditableParamField

    // Plant panel
    private lateinit var efPlantKs: EditableParamField
    private lateinit var efPlantKg: EditableParamField
    private lateinit var efPlantKv: EditableParamField
    private lateinit var efPlantKa: EditableParamField
    private lateinit var efBacklash: EditableParamField
    private lateinit var efContactK: EditableParamField
    private lateinit var efContactC: EditableParamField
    private lateinit var efLoadVisc: EditableParamField
    private lateinit var efLoadStat: EditableParamField
    private lateinit var efDisturb: EditableParamField
    private lateinit var efFlexHz: EditableParamField
    private lateinit var efFlexZeta: EditableParamField
    private lateinit var encoderCombo: JComboBox<ArmPlantConfig.EncoderKind>

    private var sysIdBtn: JButton? = null
    private var suggestPdBtn: JButton? = null
    private var simTimer: Timer? = null

    init {
        // Flight recorder: every GUI session logs per-tick state + session events to CSV, so an
        // anomaly observed live can be reconstructed offline afterward.
        val log = engine.startFlightRecorder(Paths.get("controllab-logs"))
        if (log != null) {
            println("Arm tab flight recorder: ${log.toAbsolutePath()}")
        }

        buildChart()
        buildCenter()
        buildSidebar()
        buildTimer()
        syncTargetUi(engine.getTargetRad())
        updateControllerPanel()
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Chart + canvas
    // ─────────────────────────────────────────────────────────────────────────────

    private fun buildChart() {
        chart =
            XYChartBuilder()
                .width(700)
                .height(600)
                .title("Arm Simulation")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Angle (deg)")
                .build()
        chart.styler.markerSize = 0
        chart.styler.legendPosition = Styler.LegendPosition.InsideNW
        chart.setYAxisGroupTitle(1, "Velocity (deg/s)")

        addLine("Target", true)
        addLine("Profile pos", false)
        addLine("Measured enc", false)
        addLine("True load", false)
        addLine("Motor side", false)

        addVelLine("Load vel")
        addVelLine("Profile vel")
        addVelLine("Est vel")

        chartPanel = XChartPanel(chart)
    }

    private fun addLine(name: String, dashed: Boolean) {
        val s = chart.addSeries(name, doubleArrayOf(0.0), doubleArrayOf(0.0))
        s.marker = SeriesMarkers.NONE
        if (dashed) {
            s.lineStyle =
                BasicStroke(
                    1.5f,
                    BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,
                    10f,
                    floatArrayOf(6f, 4f),
                    0f,
                )
        }
    }

    /** A velocity-axis (group 1) line series. */
    private fun addVelLine(name: String) {
        val s = chart.addSeries(name, doubleArrayOf(0.0), doubleArrayOf(0.0))
        s.marker = SeriesMarkers.NONE
        s.yAxisGroup = 1
    }

    private fun buildCenter() {
        canvas = ArmCanvas(engine) { rad -> applyTargetRad(rad, false) }

        metricsLabel = JLabel()
        metricsLabel.border = BorderFactory.createEmptyBorder(6, 10, 6, 10)
        metricsLabel.verticalAlignment = SwingConstants.TOP

        val center = JPanel(BorderLayout())
        center.add(canvas, BorderLayout.WEST)
        center.add(chartPanel, BorderLayout.CENTER)
        center.add(metricsLabel, BorderLayout.SOUTH)
        add(center, BorderLayout.CENTER)
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Sidebar
    // ─────────────────────────────────────────────────────────────────────────────

    private fun buildSidebar() {
        val sidebar = JPanel()
        sidebar.layout = BoxLayout(sidebar, BoxLayout.Y_AXIS)
        sidebar.border = BorderFactory.createEmptyBorder(8, 8, 8, 8)

        typeCombo = JComboBox(ArmControllerType.entries.toTypedArray())
        typeCombo.maximumSize = Dimension(Int.MAX_VALUE, 28)
        typeCombo.addActionListener {
            engine.setControllerType(typeCombo.selectedItem as ArmControllerType)
            updateControllerPanel()
            buffer.clear()
        }
        sidebar.add(typeCombo)
        sidebar.add(Box.createVerticalStrut(8))

        sidebar.add(boldLabel("Plant model"))
        plantCombo = JComboBox(ArmEngine.PlantKind.entries.toTypedArray())
        plantCombo.selectedItem = engine.getPlantKind()
        plantCombo.maximumSize = Dimension(Int.MAX_VALUE, 28)
        plantCombo.addActionListener {
            engine.setPlantKind(plantCombo.selectedItem as ArmEngine.PlantKind)
        }
        sidebar.add(plantCombo)
        sidebar.add(Box.createVerticalStrut(8))

        // Target slider (degrees).
        sidebar.add(boldLabel("Target"))
        targetLabel = JLabel()
        sidebar.add(targetLabel)
        val minDeg = floor(Math.toDegrees(engine.getMinAngleRad())).toInt()
        val maxDeg = ceil(Math.toDegrees(engine.getMaxAngleRad())).toInt()
        targetSlider =
            JSlider(
                minDeg,
                maxDeg,
                Math.toDegrees(engine.getTargetRad()).roundToInt(),
            )
        targetSlider.maximumSize = Dimension(Int.MAX_VALUE, 40)
        targetSlider.addChangeListener {
            if (updatingSlider) return@addChangeListener
            applyTargetRad(Math.toRadians(targetSlider.value.toDouble()), true)
        }
        sidebar.add(targetSlider)
        sidebar.add(Box.createVerticalStrut(8))

        val resetBtn = JButton("Reset (park)")
        resetBtn.maximumSize = Dimension(Int.MAX_VALUE, 30)
        resetBtn.addActionListener {
            engine.reset()
            buffer.clear()
            syncTargetUi(engine.getTargetRad())
        }
        sidebar.add(resetBtn)
        sidebar.add(Box.createVerticalStrut(6))

        sysIdBtn =
            JButton("Run SysID → model").also { btn ->
                btn.maximumSize = Dimension(Int.MAX_VALUE, 30)
                btn.addActionListener { onRunSysId() }
                sidebar.add(btn)
            }
        sidebar.add(Box.createVerticalStrut(4))

        suggestPdBtn =
            JButton("Suggest PD from model").also { btn ->
                btn.maximumSize = Dimension(Int.MAX_VALUE, 30)
                btn.toolTipText =
                    "Pole-place mechanism kP/kD from model kV,kA and ωₙ, ζ (after SysID)."
                btn.addActionListener { onSuggestPd() }
                sidebar.add(btn)
            }
        sidebar.add(Box.createVerticalStrut(10))

        sidebar.add(boldLabel("Controller gains"))
        buildControllerPanels()
        sidebar.add(pdPanel)
        sidebar.add(lqrPanel)
        sidebar.add(mechPanel)
        sidebar.add(Box.createVerticalStrut(6))
        sidebar.add(ffPanel)
        sidebar.add(Box.createVerticalStrut(10))

        sidebar.add(boldLabel("Plant (the real robot)"))
        buildPlantPanel(sidebar)

        val scroll =
            JScrollPane(
                sidebar,
                JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
                JScrollPane.HORIZONTAL_SCROLLBAR_NEVER,
            )
        scroll.preferredSize = Dimension(SIDEBAR_WIDTH, 0)
        scroll.verticalScrollBar.unitIncrement = 16
        add(scroll, BorderLayout.WEST)
    }

    private fun buildControllerPanels() {
        // PD
        efKP = EditableParamField("kP", engine.getKP(), "%.2f", 0.0, 500.0) { updatePd() }
        efKD = EditableParamField("kD", engine.getKD(), "%.3f", 0.0, 100.0) { updatePd() }
        pdPanel = vpanel(efKP, efKD)

        // LQR
        efQPos =
            EditableParamField("Q pos", engine.getLqrQPos(), "%.3f", 1e-3, 100.0) { updateLqr() }
        efQVel =
            EditableParamField("Q vel", engine.getLqrQVel(), "%.3f", 1e-3, 100.0) { updateLqr() }
        efR = EditableParamField("R volt", engine.getLqrR(), "%.2f", 1e-3, 100.0) { updateLqr() }
        lqrPanel = vpanel(efQPos, efQVel, efR)

        // Mechanism (Lineage B) — includes model FF (kS/kV/kA/kCos/kSin) so SysID results are
        // visible.
        val g = engine.getMechGains()
        efMkP = EditableParamField("kP", g.kP, "%.2f", 0.0, 500.0) { updateMech() }
        efMkI = EditableParamField("kI", g.kI, "%.2f", 0.0, 500.0) { updateMech() }
        efMkD = EditableParamField("kD", g.kD, "%.3f", 0.0, 100.0) { updateMech() }
        efMkS = EditableParamField("kS", g.kS, "%.3f", 0.0, 12.0) { updateMech() }
        efMkV = EditableParamField("kV", g.kV, "%.3f", 0.0, 12.0) { updateMech() }
        efMkA = EditableParamField("kA", g.kA, "%.4f", 1e-4, 12.0) { updateMech() }
        efMkCos = EditableParamField("kCos", g.kCos, "%.3f", 0.0, 20.0) { updateMech() }
        efMkSin = EditableParamField("kSin", g.kSin, "%.3f", -20.0, 20.0) { updateMech() }
        efMvMax = EditableParamField("vMax", g.maxVel, "%.2f", 0.1, 50.0) { updateMech() }
        efMaMax = EditableParamField("aMax", g.maxAccel, "%.2f", 0.1, 200.0) { updateMech() }
        efMdMax = EditableParamField("dMax", g.maxDecel, "%.2f", 0.1, 200.0) { updateMech() }
        efMjMax = EditableParamField("jMax", g.maxJerk, "%.1f", 1.0, 5000.0) { updateMech() }
        // Design specs for Suggest PD (not controller gains themselves).
        efOmegaN = EditableParamField("ωₙ (rad/s)", 4.0, "%.2f", 0.1, 30.0) {}
        efZeta = EditableParamField("ζ damp", 0.8, "%.2f", 0.1, 3.0) {}
        mechPanel =
            vpanel(
                efMkP,
                efMkI,
                efMkD,
                efMkS,
                efMkV,
                efMkA,
                efMkCos,
                efMkSin,
                efMvMax,
                efMaMax,
                efMdMax,
                efMjMax,
                boldLabel("PD design (Suggest PD)"),
                efOmegaN,
                efZeta,
            )

        // Shared feedforward (Lineage A only)
        efFfKs = EditableParamField("ff kS", engine.getFfKs(), "%.3f", 0.0, 12.0) { updateFf() }
        efFfKg = EditableParamField("ff kG", engine.getFfKg(), "%.3f", 0.0, 12.0) { updateFf() }
        efFfKv = EditableParamField("ff kV", engine.getFfKv(), "%.3f", 0.0, 12.0) { updateFf() }
        efFfKa = EditableParamField("ff kA", engine.getFfKa(), "%.4f", 1e-4, 12.0) { updateFf() }
        ffPanel = vpanel(boldLabel("Feedforward"), efFfKs, efFfKg, efFfKv, efFfKa)
    }

    private fun buildPlantPanel(sidebar: JPanel) {
        efPlantKs =
            EditableParamField("kS", engine.getPlantKs(), "%.3f", 0.0, 12.0) { updatePlantDyn() }
        efPlantKg =
            EditableParamField("kG", engine.getPlantKg(), "%.3f", 0.0, 12.0) { updatePlantDyn() }
        efPlantKv =
            EditableParamField("kV", engine.getPlantKv(), "%.3f", 0.0, 12.0) { updatePlantDyn() }
        efPlantKa =
            EditableParamField("kA", engine.getPlantKa(), "%.4f", 1e-4, 12.0) { updatePlantDyn() }
        efBacklash =
            EditableParamField(
                "Backlash (deg)",
                Math.toDegrees(engine.getBacklashRadCfg()),
                "%.2f",
                0.0,
                30.0,
            ) { v ->
                engine.setBacklashRad(Math.toRadians(v))
            }
        efContactK =
            EditableParamField("Contact k", engine.getContactStiffness(), "%.1f", 1.0, 5000.0) { v
                ->
                engine.setContact(v, engine.getContactDamping())
            }
        efContactC =
            EditableParamField("Contact c", engine.getContactDamping(), "%.2f", 0.0, 100.0) { v ->
                engine.setContact(engine.getContactStiffness(), v)
            }
        efLoadVisc =
            EditableParamField("Load kV", engine.getLoadViscous(), "%.3f", 0.0, 12.0) { v ->
                engine.setLoadFriction(v, engine.getLoadStatic())
            }
        efLoadStat =
            EditableParamField("Load kS", engine.getLoadStatic(), "%.3f", 0.0, 12.0) { v ->
                engine.setLoadFriction(engine.getLoadViscous(), v)
            }
        efDisturb =
            EditableParamField(
                "Disturbance (V)",
                engine.getDisturbanceVoltage(),
                "%.2f",
                -12.0,
                12.0,
            ) { v ->
                engine.setDisturbanceVoltage(v)
            }
        efFlexHz =
            EditableParamField("Flex ωₙ (Hz)", engine.getFlexHz(), "%.2f", 0.2, 30.0) { v ->
                engine.setFlexParams(v, engine.getFlexZeta())
            }
        efFlexZeta =
            EditableParamField("Flex ζ", engine.getFlexZeta(), "%.3f", 0.001, 1.0) { v ->
                engine.setFlexParams(engine.getFlexHz(), v)
            }

        encoderCombo = JComboBox(ArmPlantConfig.EncoderKind.entries.toTypedArray())
        encoderCombo.selectedItem = engine.getEncoderKind()
        encoderCombo.maximumSize = Dimension(Int.MAX_VALUE, 26)
        encoderCombo.addActionListener {
            engine.setEncoderKind(encoderCombo.selectedItem as ArmPlantConfig.EncoderKind)
        }

        sidebar.add(efPlantKs)
        sidebar.add(efPlantKg)
        sidebar.add(efPlantKv)
        sidebar.add(efPlantKa)
        sidebar.add(efBacklash)
        sidebar.add(efContactK)
        sidebar.add(efContactC)
        sidebar.add(efLoadVisc)
        sidebar.add(efLoadStat)
        sidebar.add(efFlexHz)
        sidebar.add(efFlexZeta)
        sidebar.add(efDisturb)
        sidebar.add(JLabel("Encoder:"))
        sidebar.add(encoderCombo)
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Sidebar callbacks
    // ─────────────────────────────────────────────────────────────────────────────

    private fun updatePd() {
        engine.setPdGains(efKP.value, efKD.value)
    }

    private fun updateLqr() {
        engine.setLqrWeights(efQPos.value, efQVel.value, efR.value)
    }

    private fun updateFf() {
        engine.setFeedforwardGains(efFfKs.value, efFfKg.value, efFfKv.value, efFfKa.value)
    }

    private fun updateMech() {
        engine.setMechanismGains(
            efMkP.value,
            efMkI.value,
            efMkD.value,
            efMkS.value,
            efMkV.value,
            efMkA.value,
            efMkCos.value,
            efMkSin.value,
            efMvMax.value,
            efMaMax.value,
            efMdMax.value,
            efMjMax.value,
        )
    }

    private fun updatePlantDyn() {
        engine.setPlantDynamics(
            efPlantKs.value,
            efPlantKg.value,
            efPlantKv.value,
            efPlantKa.value,
        )
    }

    /**
     * Run SysID off the EDT (the characterization is many simulated seconds of work) and present
     * the result on the EDT when done.
     */
    private fun onRunSysId() {
        sysIdBtn?.isEnabled = false
        sysIdBtn?.text = "Running SysID…"
        val plantKind = engine.getPlantKind()
        val plantKs = engine.getPlantKs()
        val plantKv = engine.getPlantKv()
        val plantKa = engine.getPlantKa()
        val plantKg = engine.getPlantKg()
        val plantHalfLash = engine.getBacklashRad() / 2.0

        val worker =
            object : SwingWorker<ArmSysId.Result, Void>() {
                override fun doInBackground(): ArmSysId.Result = engine.runSysId()

                override fun done() {
                    try {
                        val r = get()
                        presentSysIdResult(
                            r,
                            plantKind,
                            plantKs,
                            plantKv,
                            plantKa,
                            plantKg,
                            plantHalfLash,
                        )
                    } catch (ex: InterruptedException) {
                        Thread.currentThread().interrupt()
                    } catch (ex: ExecutionException) {
                        val cause = ex.cause ?: ex
                        JOptionPane.showMessageDialog(
                            this@ArmTab,
                            "SysID failed: ${cause.message}",
                            "SysID Error",
                            JOptionPane.ERROR_MESSAGE,
                        )
                    } finally {
                        sysIdBtn?.isEnabled = true
                        sysIdBtn?.text = "Run SysID → model"
                    }
                }
            }
        worker.execute()
    }

    private fun presentSysIdResult(
        r: ArmSysId.Result,
        plantKind: ArmEngine.PlantKind,
        plantKs: Double,
        plantKv: Double,
        plantKa: Double,
        plantKg: Double,
        plantHalfLash: Double,
    ) {
        val plant =
            if (plantKind == ArmEngine.PlantKind.RIGID) {
                "rigid plant"
            } else {
                plantKind.toString().lowercase() + " plant (motor-side encoder)"
            }
        val kvDetail =
            if (r.kVHold.isNaN()) {
                ""
            } else {
                String.format(
                    "<tr><td>kV hold / run</td><td>%.3f / %.3f</td><td></td></tr>",
                    r.kVHold,
                    r.kVRun,
                )
            }
        val lashDetail =
            if (r.halfLashRad.isNaN()) {
                ""
            } else {
                String.format(
                    "<tr><td>half-lash</td><td>%.2f°</td><td>%.2f°</td></tr>",
                    Math.toDegrees(r.halfLashRad),
                    Math.toDegrees(plantHalfLash),
                )
            }
        val warning =
            if (r.kVDisagreement() > 0.10) {
                "<br><b>Warning:</b> hold-side and run-side kV disagree by more than 10% — " +
                    "the moving runs look flex/lash-contaminated (the hold-side kV ships)."
            } else {
                ""
            }
        val msg =
            String.format(
                "<html>Identified through the %s (R² = %.4f, %d samples):<br><br>" +
                    "<table cellpadding=3>" +
                    "<tr><th></th><th>recovered</th><th>true plant</th></tr>" +
                    "<tr><td>kS</td><td>%.3f</td><td>%.3f</td></tr>" +
                    "<tr><td>kV</td><td>%.3f</td><td>%.3f</td></tr>" +
                    "<tr><td>kA</td><td>%.3f</td><td>%.3f</td></tr>" +
                    "<tr><td>kCos (kG)</td><td>%.3f</td><td>%.3f</td></tr>" +
                    "<tr><td>kSin</td><td>%.3f</td><td>%.3f</td></tr>" +
                    "%s%s" +
                    "</table>%s<br>Apply these to the controller model?</html>",
                plant,
                r.rSquared,
                r.samples,
                r.kS,
                plantKs,
                r.kV,
                plantKv,
                r.kA,
                plantKa,
                r.kCos,
                plantKg,
                r.kSin,
                0.0,
                kvDetail,
                lashDetail,
                warning,
            )
        val choice =
            JOptionPane.showConfirmDialog(
                this,
                msg,
                "SysID Result",
                JOptionPane.YES_NO_OPTION,
                JOptionPane.PLAIN_MESSAGE,
            )
        if (choice != JOptionPane.YES_OPTION) return

        engine.applyIdentifiedGains(r)
        // Reflect the identified values in both Lineage A FF fields and mechanism model fields.
        efFfKs.setValue(r.kS, "%.3f")
        efFfKg.setValue(r.kCos, "%.3f")
        efFfKv.setValue(r.kV, "%.3f")
        efFfKa.setValue(r.kA, "%.4f")
        efMkS.setValue(r.kS, "%.3f")
        efMkV.setValue(r.kV, "%.3f")
        efMkA.setValue(r.kA, "%.4f")
        efMkCos.setValue(r.kCos, "%.3f")
        efMkSin.setValue(r.kSin, "%.3f")
        buffer.clear()
    }

    private fun updateControllerPanel() {
        val t = typeCombo.selectedItem as ArmControllerType
        pdPanel.isVisible = t == ArmControllerType.ARM_PD
        lqrPanel.isVisible = t == ArmControllerType.ARM_LQR
        mechPanel.isVisible = t == ArmControllerType.MECHANISM_PIDF
        ffPanel.isVisible = t == ArmControllerType.ARM_PD || t == ArmControllerType.ARM_LQR
        suggestPdBtn?.isEnabled = t == ArmControllerType.MECHANISM_PIDF
        revalidate()
        repaint()
    }

    /**
     * Pole-place mechanism PD from the current model kV/kA (SysID or hand-entered) and the ωₙ, ζ
     * design fields. Leaves kI unchanged.
     */
    private fun onSuggestPd() {
        val kV = efMkV.value
        val kA = efMkA.value
        val omegaN = efOmegaN.value
        val zeta = efZeta.value
        val s: FeedbackGainSynthesis.PdSuggestion
        try {
            s = FeedbackGainSynthesis.suggestPd(kV, kA, omegaN, zeta)
        } catch (ex: IllegalArgumentException) {
            JOptionPane.showMessageDialog(
                this,
                ex.message,
                "Suggest PD",
                JOptionPane.ERROR_MESSAGE,
            )
            return
        }

        val clampNote =
            if (s.kDClampedToZero) {
                "<br><i>kD clamped to 0: plant kV already supplies more damping than 2ζωₙ kA " +
                    "at this ωₙ — raise ωₙ if you want more derivative authority.</i>"
            } else {
                ""
            }
        val msg =
            String.format(
                "<html>From model kV=%.3f, kA=%.4f and design ωₙ=%.2f rad/s, ζ=%.2f:<br><br>" +
                    "<code>kP = kA · ωₙ² = %.3f</code> V/rad<br>" +
                    "<code>kD = 2ζωₙ kA − kV = %.3f</code> V/(rad/s)<br>" +
                    "%s<br><br>" +
                    "Apply to mechanism feedback? (kI left unchanged)</html>",
                s.kV,
                s.kA,
                s.omegaN,
                s.zeta,
                s.kP,
                s.kD,
                clampNote,
            )
        val choice =
            JOptionPane.showConfirmDialog(
                this,
                msg,
                "Suggest PD from model",
                JOptionPane.YES_NO_OPTION,
                JOptionPane.PLAIN_MESSAGE,
            )
        if (choice != JOptionPane.YES_OPTION) return

        efMkP.setValue(s.kP, "%.2f")
        efMkD.setValue(s.kD, "%.3f")
        updateMech()
        buffer.clear()
    }

    private fun applyTargetRad(rad: Double, fromSlider: Boolean) {
        engine.setTargetRad(rad)
        val deg = Math.toDegrees(engine.getTargetRad())
        if (!fromSlider) {
            updatingSlider = true
            targetSlider.value = deg.roundToInt()
            updatingSlider = false
        }
        targetLabel.text = String.format("%.1f°", deg)
    }

    private fun syncTargetUi(rad: Double) {
        updatingSlider = true
        targetSlider.value = Math.toDegrees(rad).roundToInt()
        updatingSlider = false
        targetLabel.text = String.format("%.1f°", Math.toDegrees(rad))
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Timer loop
    // ─────────────────────────────────────────────────────────────────────────────

    private fun buildTimer() {
        simTimer = Timer(TIMER_MS) { onTick() }
        simTimer!!.start()
    }

    private fun onTick() {
        engine.tick()
        recordSample()
        canvas.repaint()
        updateMetricsLabel()
    }

    private fun recordSample() {
        buffer.add(
            engine.getElapsedSec(),
            Math.toDegrees(engine.getTargetRad()),
            Math.toDegrees(engine.getTrajPosRad()),
            Math.toDegrees(engine.getMeasuredEncoderRad()),
            Math.toDegrees(engine.getTrueLoadRad()),
            Math.toDegrees(engine.getMotorRad()),
            Math.toDegrees(engine.getTrueLoadVelRad()),
            Math.toDegrees(engine.getTrajVelRad()),
            Math.toDegrees(engine.getEstimatedVelRad()),
        )

        val times = buffer.getTimes()
        if (times.size < 2) return
        // xchart's List overloads are Java MutableList; ArrayList matches from Kotlin.
        val t = ArrayList(times)
        chart.updateXYSeries("Target", t, ArrayList(buffer.getData(0)), null)
        chart.updateXYSeries("Profile pos", t, ArrayList(buffer.getData(1)), null)
        chart.updateXYSeries("Measured enc", t, ArrayList(buffer.getData(2)), null)
        chart.updateXYSeries("True load", t, ArrayList(buffer.getData(3)), null)
        chart.updateXYSeries("Motor side", t, ArrayList(buffer.getData(4)), null)
        chart.updateXYSeries("Load vel", t, ArrayList(buffer.getData(5)), null)
        chart.updateXYSeries("Profile vel", t, ArrayList(buffer.getData(6)), null)
        chart.updateXYSeries("Est vel", t, ArrayList(buffer.getData(7)), null)
        chartPanel.repaint()
    }

    private fun updateMetricsLabel() {
        val m = engine.getMetrics()
        val settle =
            if (m.settleTimeSec().isNaN()) "—" else String.format("%.2f s", m.settleTimeSec())
        metricsLabel.text =
            String.format(
                "<html><b>%s</b> &nbsp; | &nbsp; Engaged: %.0f%%<br>" +
                    "Target %.1f° &nbsp; Load %.1f° &nbsp; Est %.1f°<br>" +
                    "SS err %.1f° &nbsp; Overshoot %.1f° &nbsp; Settle %s<br>" +
                    "Lash gap %.1f° &nbsp; Lost motion (peak) %.1f°</html>",
                engine.getModeLabel(),
                m.pctEngaged(),
                Math.toDegrees(engine.getTargetRad()),
                Math.toDegrees(engine.getTrueLoadRad()),
                Math.toDegrees(engine.getEstimatedPosRad()),
                m.steadyStateErrorDeg(),
                m.overshootDeg(),
                settle,
                m.lashGapDeg(),
                m.lostMotionDeg(),
            )
    }

    fun dispose() {
        simTimer?.stop()
    }

    companion object {
        private const val SIDEBAR_WIDTH = 300
        private const val TIMER_MS = 16
        private const val BUFFER_POINTS = 600
        private const val WINDOW_SECS = 6.0
        private const val NUM_STREAMS = 8

        private fun vpanel(vararg items: JComponent): JPanel {
            val p = JPanel()
            p.layout = BoxLayout(p, BoxLayout.Y_AXIS)
            p.alignmentX = Component.LEFT_ALIGNMENT
            for (c in items) {
                c.alignmentX = Component.LEFT_ALIGNMENT
                p.add(c)
            }
            return p
        }

        private fun boldLabel(text: String): JLabel {
            val l = JLabel(text)
            l.font = l.font.deriveFont(Font.BOLD)
            l.alignmentX = Component.LEFT_ALIGNMENT
            return l
        }
    }
}
