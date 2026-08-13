package org.marsroboticsassociation.controllab

import edu.wpi.first.math.MathUtil
import java.awt.BorderLayout
import java.awt.FlowLayout
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import java.io.File
import java.util.ArrayList
import java.util.Locale
import javax.swing.Action
import javax.swing.Box
import javax.swing.BoxLayout
import javax.swing.JButton
import javax.swing.JComboBox
import javax.swing.JFileChooser
import javax.swing.JFrame
import javax.swing.JLabel
import javax.swing.JOptionPane
import javax.swing.JPanel
import javax.swing.JSlider
import javax.swing.JTabbedPane
import javax.swing.JTextField
import javax.swing.SwingUtilities
import javax.swing.event.ChangeEvent
import javax.swing.filechooser.FileNameExtensionFilter
import kotlin.math.abs
import kotlin.math.log10
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.round
import kotlin.math.sqrt
import org.knowm.xchart.XChartPanel
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.SeriesMarkers
import org.marsroboticsassociation.controllab.arm.ArmTab
import org.marsroboticsassociation.controllab.flywheel.FlywheelTab
import org.marsroboticsassociation.controllab.trajectory.TrajectoryTab
import org.marsroboticsassociation.controllib.filter.Filter

class ControlLabApp {

    private lateinit var frame: JFrame
    private lateinit var timeColCombo: JComboBox<String>
    private lateinit var dataColCombo: JComboBox<String>
    private lateinit var startField: JTextField
    private lateinit var endField: JTextField
    private lateinit var filterTypeCombo: JComboBox<String>
    private lateinit var cutoffSlider: JSlider // map 0..1000 -> 0..500 Hz
    private lateinit var cutoffLabel: JLabel

    private lateinit var qSlider: JSlider
    private lateinit var qLabel: JLabel

    private lateinit var alphaSlider: JSlider
    private lateinit var betaSlider: JSlider
    private lateinit var gammaSlider: JSlider
    private lateinit var alphaLabel: JLabel
    private lateinit var betaLabel: JLabel
    private lateinit var gammaLabel: JLabel

    private lateinit var lagLabel: JLabel
    private lateinit var chart: XYChart
    private lateinit var chartPanel: XChartPanel<XYChart>
    private var loadedSignal: CsvSignal? = null
    private var lastTime: List<Double>? = null
    private var lastRaw: List<Double>? = null
    private var lastFiltered: List<Double>? = null

    // Container so we can show/hide ABG sliders
    private lateinit var abgContainer: JPanel

    private fun createAndShow() {
        frame = JFrame("Control Lab")
        frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
        frame.setSize(1200, 800)
        frame.layout = BorderLayout()

        // top controls
        val top = JPanel(FlowLayout(FlowLayout.LEFT))
        val openBtn = JButton("Open CSV")
        top.add(openBtn)

        top.add(JLabel("Time:"))
        timeColCombo = JComboBox()
        top.add(timeColCombo)

        top.add(JLabel("Signal:"))
        dataColCombo = JComboBox()
        top.add(dataColCombo)

        top.add(JLabel("Start:"))
        startField = JTextField(6)
        top.add(startField)

        top.add(JLabel("End:"))
        endField = JTextField(6)
        top.add(endField)

        filterTypeCombo =
            JComboBox(arrayOf("NONE", "LOWPASS", "BIQUAD", "BESSEL", "ALPHA-BETA-GAMMA"))
        top.add(JLabel("Filter:"))
        top.add(filterTypeCombo)

        val cutoffSliderVal = cutoffToSlider(4.0)
        cutoffSlider = JSlider(1, 1000, cutoffSliderVal) // map to 0.1..1000 Hz approximately
        cutoffLabel = JLabel(String.format("Cutoff: %.2f Hz", sliderToCutoff(cutoffSliderVal)))
        cutoffSlider.addChangeListener { _: ChangeEvent ->
            val hz = sliderToCutoff(cutoffSlider.value)
            cutoffLabel.text = String.format("Cutoff: %.2f Hz", hz)
        }
        top.add(cutoffLabel)
        top.add(cutoffSlider)

        // ------------------------------
        // Q slider for BIQUAD
        // ------------------------------
        val qSliderVal = qToSlider(1.0 / sqrt(2.0))
        qSlider = JSlider(1, 1000, qSliderVal) // default Q ≈ 0.707
        qLabel = JLabel(String.format("Q: %.2f", sliderToQ(qSliderVal)))
        qSlider.addChangeListener {
            val q = sliderToQ(qSlider.value)
            qLabel.text = String.format("Q: %.2f", q)
        }
        top.add(qLabel)
        top.add(qSlider)

        // ------------------------------------
        // ABG sliders (in a vertical container)
        // ------------------------------------
        alphaSlider = JSlider(0, 1000, 200)
        betaSlider = JSlider(0, 1000, 50)
        gammaSlider = JSlider(0, 1000, 0)

        alphaLabel = JLabel(String.format("α: %.3f", alphaSlider.value / 1000.0))
        betaLabel = JLabel(String.format("β: %.3f", betaSlider.value / 1000.0))
        gammaLabel = JLabel(String.format("γ: %.4f", gammaSlider.value / 1000.0))

        alphaSlider.addChangeListener {
            alphaLabel.text = String.format("α: %.3f", alphaSlider.value / 1000.0)
        }
        betaSlider.addChangeListener {
            betaLabel.text = String.format("β: %.3f", betaSlider.value / 1000.0)
        }
        gammaSlider.addChangeListener {
            gammaLabel.text = String.format("γ: %.3f", gammaSlider.value / 1000.0)
        }

        abgContainer = JPanel()
        abgContainer.layout = BoxLayout(abgContainer, BoxLayout.Y_AXIS)

        abgContainer.add(alphaLabel)
        abgContainer.add(alphaSlider)
        abgContainer.add(Box.createVerticalStrut(12))

        abgContainer.add(betaLabel)
        abgContainer.add(betaSlider)
        abgContainer.add(Box.createVerticalStrut(12))

        abgContainer.add(gammaLabel)
        abgContainer.add(gammaSlider)

        abgContainer.isVisible = false // only show when ABG filter active

        // ------------------------------------
        // Right controls
        // ------------------------------------
        val right = JPanel(BorderLayout())
        val buttons = JPanel(FlowLayout(FlowLayout.LEFT))
        val applyBtn = JButton("Apply Filter")
        val exportBtn = JButton("Export Filtered CSV")
        val estimateBtn = JButton("Estimate Lag")
        buttons.add(applyBtn)
        buttons.add(estimateBtn)
        buttons.add(exportBtn)
        right.add(buttons, BorderLayout.NORTH)

        lagLabel = JLabel("Lag: N/A")
        right.add(lagLabel, BorderLayout.SOUTH)

        // chart
        chart =
            XYChartBuilder()
                .width(1000)
                .height(600)
                .title("Control Lab")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Value")
                .build()
        chartPanel = XChartPanel(chart)

        val filterPanel = JPanel(BorderLayout())
        filterPanel.add(abgContainer, BorderLayout.WEST)
        filterPanel.add(top, BorderLayout.NORTH)
        filterPanel.add(chartPanel, BorderLayout.CENTER)
        filterPanel.add(right, BorderLayout.SOUTH)

        val trajectoryTab = TrajectoryTab()
        val flywheelTab = FlywheelTab()
        val armTab = ArmTab()
        val tabs = JTabbedPane()
        tabs.addTab("Filter", filterPanel)
        tabs.addTab("Trajectory", trajectoryTab)
        tabs.addTab("Flywheel", flywheelTab)
        tabs.addTab("Arm", armTab)

        frame.addWindowListener(
            object : WindowAdapter() {
                override fun windowClosing(e: WindowEvent) {
                    trajectoryTab.disposeEngine()
                    flywheelTab.dispose()
                    armTab.dispose()
                }
            }
        )

        frame.add(tabs, BorderLayout.CENTER)

        updateControlsForFilter()

        openBtn.addActionListener { onOpenCsv() }
        filterTypeCombo.addActionListener { updateControlsForFilter() }
        applyBtn.addActionListener { onApplyFilter() }
        estimateBtn.addActionListener { onEstimateLag() }
        exportBtn.addActionListener { onExportCsv() }

        frame.isVisible = true
    }

    private fun updateControlsForFilter() {
        val sel = filterTypeCombo.selectedItem as String?
        val isLP = "LOWPASS" == sel
        val isBiquad = "BIQUAD" == sel
        val isBessel = "BESSEL" == sel
        val isABG = "ALPHA-BETA-GAMMA" == sel

        val usesCutoff = isLP || isBiquad || isBessel

        cutoffSlider.isEnabled = usesCutoff
        cutoffLabel.isEnabled = usesCutoff

        qSlider.isEnabled = isBiquad
        qLabel.isEnabled = isBiquad

        abgContainer.isVisible = isABG

        frame.revalidate()
    }

    private fun onOpenCsv() {
        val downloads = getDefaultDownloadsDir()
        val chooser = JFileChooser(downloads)
        val csvFilter = FileNameExtensionFilter("CSV Files (*.csv)", "csv")
        chooser.fileFilter = csvFilter
        forceDetailsView(chooser)
        chooser.dialogTitle = "Open CSV"
        val ret = chooser.showOpenDialog(frame)
        if (ret != JFileChooser.APPROVE_OPTION) return
        val f = chooser.selectedFile
        try {
            val signal = CsvSignal.load(f.absolutePath)
            loadedSignal = signal
            timeColCombo.removeAllItems()
            dataColCombo.removeAllItems()
            for (h in signal.headers()) {
                timeColCombo.addItem(h)
                dataColCombo.addItem(h)
            }

            // Defaults
            if (signal.headers().contains("Run Time")) timeColCombo.selectedItem = "Run Time"

            if (signal.headers().contains("flywheel TPS measured"))
                dataColCombo.selectedItem = "flywheel TPS measured"

            val selectedTimeCol = timeColCombo.selectedItem as? String ?: return
            val selectedDataCol = dataColCombo.selectedItem as? String ?: return

            signal.select(selectedTimeCol, selectedDataCol)

            val min = signal.minTime() ?: 0.0
            val max = signal.maxTime() ?: 1.0
            startField.text = String.format("%.3f", min)
            endField.text = String.format("%.3f", max)
        } catch (ex: Exception) {
            JOptionPane.showMessageDialog(frame, "Failed to load CSV: " + ex.message)
            ex.printStackTrace()
        }
    }

    private fun onApplyFilter() {
        val signal =
            loadedSignal
                ?: run {
                    JOptionPane.showMessageDialog(frame, "Load a CSV first.")
                    return
                }

        val timeCol = timeColCombo.selectedItem as String?
        val dataCol = dataColCombo.selectedItem as String?
        if (timeCol == null || dataCol == null) {
            JOptionPane.showMessageDialog(frame, "Select time and signal columns.")
            return
        }

        val s = parseDoubleOrDefault(startField.text, Double.NEGATIVE_INFINITY)
        val e = parseDoubleOrDefault(endField.text, Double.POSITIVE_INFINITY)

        signal.select(timeCol, dataCol).window(s, e)
        val time = signal.time()
        val raw = signal.data()

        if (time.isEmpty() || raw.isEmpty()) {
            JOptionPane.showMessageDialog(frame, "No data in selected window.")
            return
        }

        // create filter from UI
        val sel = filterTypeCombo.selectedItem as String
        val type = FilterFactory.Type.valueOf(sel.replace('-', '_').uppercase(Locale.ROOT))
        val filter: Filter =
            when (type) {
                FilterFactory.Type.LOWPASS -> {
                    val cutoff = sliderToCutoff(cutoffSlider.value)
                    FilterFactory.create(FilterFactory.Type.LOWPASS, cutoff, Double.NaN, Double.NaN)
                }
                FilterFactory.Type.BIQUAD -> {
                    val cutoff = sliderToCutoff(cutoffSlider.value)
                    val q = sliderToQ(qSlider.value)
                    FilterFactory.create(FilterFactory.Type.BIQUAD, cutoff, q, Double.NaN)
                }
                else ->
                    FilterFactory.create(
                        FilterFactory.Type.NONE,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                    )
            }

        // apply variable-dt filtering
        val filtered = ArrayList<Double>(raw.size)
        val derivative = ArrayList<Double>(raw.size)
        filter.reset()
        var prevT = time[0]
        for (i in raw.indices) {
            val t = time[i]
            val dt = if (i == 0) 0.0 else t - prevT
            prevT = t
            filtered.add(filter.update(raw[i], dt))
            val deriv = filter.rate
            if (!deriv.isNaN()) derivative.add(deriv)
        }

        lastTime = time
        lastRaw = raw
        lastFiltered = filtered

        redrawChart(time, raw, filtered, derivative.takeIf { it.isNotEmpty() })
    }

    private fun onEstimateLag() {
        val time = lastTime
        val raw = lastRaw
        val filtered = lastFiltered
        if (time == null || raw == null || filtered == null) {
            JOptionPane.showMessageDialog(frame, "Run filter first.")
            return
        }
        val lagSeconds = Utils.estimateLagSeconds(raw, filtered, time)
        lagLabel.text = String.format("Lag ≈ %.4f s", lagSeconds)
    }

    private fun onExportCsv() {
        val time = lastTime
        val raw = lastRaw
        val filtered = lastFiltered
        if (time == null || raw == null || filtered == null) {
            JOptionPane.showMessageDialog(frame, "Run filter first.")
            return
        }
        val chooser = JFileChooser()
        chooser.dialogTitle = "Save filtered CSV"
        val ret = chooser.showSaveDialog(frame)
        if (ret != JFileChooser.APPROVE_OPTION) return
        val out = chooser.selectedFile
        try {
            Utils.exportToCsv(out.toPath(), time, raw, filtered)
            JOptionPane.showMessageDialog(frame, "Exported to: " + out.absolutePath)
        } catch (ex: Exception) {
            JOptionPane.showMessageDialog(frame, "Export failed: " + ex.message)
            ex.printStackTrace()
        }
    }

    private fun redrawChart(
        time: List<Double>,
        raw: List<Double>,
        filtered: List<Double>,
        derivative: List<Double>?,
    ) {
        chart.styler.markerSize = 2
        chart.seriesMap.clear()
        chart.addSeries("raw", time, raw).setMarker(SeriesMarkers.CIRCLE)
        chart.addSeries("filtered", time, filtered).setMarker(SeriesMarkers.NONE)
        if (derivative != null && derivative.isNotEmpty()) {
            chart.addSeries("derivative", time, derivative).setMarker(SeriesMarkers.NONE)
        }
        chartPanel.revalidate()
        chartPanel.repaint()
    }

    companion object {
        @JvmStatic
        fun main(args: Array<String>) {
            SwingUtilities.invokeLater {
                try {
                    ControlLabApp().createAndShow()
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }
        }

        private fun getDefaultDownloadsDir(): File {
            val home = System.getProperty("user.home")

            if (System.getProperty("os.name").lowercase().contains("win")) {
                return File(home, "Downloads")
            }
            if (System.getProperty("os.name").lowercase().contains("mac")) {
                return File("$home/Downloads")
            }

            val xdg = System.getenv("XDG_DOWNLOAD_DIR")
            if (xdg != null) return File(xdg)

            return File(home, "Downloads")
        }

        private fun forceDetailsView(chooser: JFileChooser) {
            SwingUtilities.invokeLater {
                val details: Action? = chooser.actionMap.get("viewTypeDetails")
                details?.actionPerformed(null)
            }
        }

        private fun sliderScaling(val_: Int, sliderMax: Int, rangeScaling: Int): Double {
            val norm = val_ / sliderMax.toDouble()
            val result = 10.0.pow(rangeScaling * norm - 1)
            return max(1e-6, result)
        }

        private fun inverseSliderScaling(valIn: Double, sliderMax: Int, rangeScaling: Int): Int {
            var v = valIn
            v = max(1e-6, v)
            val norm = (log10(v) + 1.0) / rangeScaling.toDouble()
            val result = round(norm * sliderMax.toDouble()).toInt()
            return MathUtil.clamp(result, 1, sliderMax)
        }

        // slider value mapping helper
        private fun sliderToCutoff(val_: Int): Double {
            return sliderScaling(val_, 1000, 3)
        }

        private fun cutoffToSlider(hz: Double): Int {
            return inverseSliderScaling(hz, 1000, 3)
        }

        private fun sliderToQ(val_: Int): Double {
            var val2 = sliderScaling(val_, 1000, 2)
            val butterworth = 1.0 / sqrt(2.0)
            if (abs(val2 - butterworth) < 0.02) {
                val2 = butterworth
            }
            return val2
        }

        private fun qToSlider(q: Double): Int {
            return inverseSliderScaling(q, 1000, 2)
        }

        private fun parseDoubleOrDefault(s: String?, def: Double): Double {
            if (s == null || s.isEmpty()) return def
            return try {
                s.trim().toDouble()
            } catch (_: NumberFormatException) {
                def
            }
        }
    }
}
