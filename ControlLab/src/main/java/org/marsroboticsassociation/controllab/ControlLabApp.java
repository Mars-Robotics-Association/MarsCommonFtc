package org.marsroboticsassociation.controllab;

import org.knowm.xchart.*;
import org.knowm.xchart.XChartPanel;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.filechooser.FileNameExtensionFilter;

import java.awt.*;
import java.io.File;
import java.util.List;
import java.util.*;

import org.knowm.xchart.style.markers.Marker;
import org.knowm.xchart.style.markers.SeriesMarkers;
import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.filter.Filter;

public class ControlLabApp {

    private JFrame frame;
    private JComboBox<String> timeColCombo;
    private JComboBox<String> dataColCombo;
    private JTextField startField;
    private JTextField endField;
    private JComboBox<String> filterTypeCombo;
    private JSlider cutoffSlider;        // map 0..1000 -> 0..500 Hz
    private JLabel cutoffLabel;

    private JSlider qSlider;
    private JLabel qLabel;

    private JSlider alphaSlider, betaSlider, gammaSlider;
    private JLabel alphaLabel, betaLabel, gammaLabel;

    private JLabel lagLabel;
    private XYChart chart;
    private XChartPanel<XYChart> chartPanel;
    private CsvSignal loadedSignal;
    private List<Double> lastTime, lastRaw, lastFiltered;

    // Container so we can show/hide ABG sliders
    private JPanel abgContainer;

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            try {
                new ControlLabApp().createAndShow();
            } catch (Exception e) {
                e.printStackTrace();
            }
        });
    }

    private void createAndShow() {
        frame = new JFrame("Control Lab");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1200, 800);
        frame.setLayout(new BorderLayout());

        // top controls
        JPanel top = new JPanel(new FlowLayout(FlowLayout.LEFT));
        JButton openBtn = new JButton("Open CSV");
        top.add(openBtn);

        top.add(new JLabel("Time:"));
        timeColCombo = new JComboBox<>();
        top.add(timeColCombo);

        top.add(new JLabel("Signal:"));
        dataColCombo = new JComboBox<>();
        top.add(dataColCombo);

        top.add(new JLabel("Start:"));
        startField = new JTextField(6);
        top.add(startField);

        top.add(new JLabel("End:"));
        endField = new JTextField(6);
        top.add(endField);

        filterTypeCombo = new JComboBox<>(new String[]{
                "NONE", "LOWPASS", "BIQUAD", "BESSEL", "ALPHA-BETA-GAMMA"
        });
        top.add(new JLabel("Filter:"));
        top.add(filterTypeCombo);

        int cutoffSliderVal = cutoffToSlider(4.0);
        cutoffSlider = new JSlider(1, 1000, cutoffSliderVal); // map to 0.1..1000 Hz approximately
        cutoffLabel = new JLabel(String.format("Cutoff: %.2f Hz", sliderToCutoff(cutoffSliderVal)));
        cutoffSlider.addChangeListener((ChangeEvent e) -> {
            double hz = sliderToCutoff(cutoffSlider.getValue());
            cutoffLabel.setText(String.format("Cutoff: %.2f Hz", hz));
        });
        top.add(cutoffLabel);
        top.add(cutoffSlider);

        // ------------------------------
        // Q slider for BIQUAD
        // ------------------------------
        int qSliderVal = qToSlider(1.0 / Math.sqrt(2.0));
        qSlider = new JSlider(1, 1000, qSliderVal);   // default Q ≈ 0.707
        qLabel = new JLabel(String.format("Q: %.2f", sliderToQ(qSliderVal)));
        qSlider.addChangeListener(e -> {
            double q = sliderToQ(qSlider.getValue());
            qLabel.setText(String.format("Q: %.2f", q));
        });
        top.add(qLabel);
        top.add(qSlider);

        //------------------------------------
        // ABG sliders (in a vertical container)
        //------------------------------------
        alphaSlider = new JSlider(0, 1000, 200);
        betaSlider = new JSlider(0, 1000, 50);
        gammaSlider = new JSlider(0, 1000, 0);

        alphaLabel = new JLabel(String.format("α: %.3f", alphaSlider.getValue() / 1000.0));
        betaLabel = new JLabel(String.format("β: %.3f", betaSlider.getValue() / 1000.0));
        gammaLabel = new JLabel(String.format("γ: %.4f", gammaSlider.getValue() / 1000.0));

        alphaSlider.addChangeListener(e ->
                alphaLabel.setText(String.format("α: %.3f", alphaSlider.getValue() / 1000.0)));
        betaSlider.addChangeListener(e ->
                betaLabel.setText(String.format("β: %.3f", betaSlider.getValue() / 1000.0)));
        gammaSlider.addChangeListener(e ->
                gammaLabel.setText(String.format("γ: %.3f", gammaSlider.getValue() / 1000.0)));

        abgContainer = new JPanel();
        abgContainer.setLayout(new BoxLayout(abgContainer, BoxLayout.Y_AXIS));

        abgContainer.add(alphaLabel);
        abgContainer.add(alphaSlider);
        abgContainer.add(Box.createVerticalStrut(12));

        abgContainer.add(betaLabel);
        abgContainer.add(betaSlider);
        abgContainer.add(Box.createVerticalStrut(12));

        abgContainer.add(gammaLabel);
        abgContainer.add(gammaSlider);

        abgContainer.setVisible(false);  // only show when ABG filter active

        //------------------------------------
        // Right controls
        //------------------------------------
        JPanel right = new JPanel(new BorderLayout());
        JPanel buttons = new JPanel(new FlowLayout(FlowLayout.LEFT));
        JButton applyBtn = new JButton("Apply Filter");
        JButton exportBtn = new JButton("Export Filtered CSV");
        JButton estimateBtn = new JButton("Estimate Lag");
        buttons.add(applyBtn);
        buttons.add(estimateBtn);
        buttons.add(exportBtn);
        right.add(buttons, BorderLayout.NORTH);

        lagLabel = new JLabel("Lag: N/A");
        right.add(lagLabel, BorderLayout.SOUTH);

        // chart
        chart = new XYChartBuilder().width(1000).height(600).title("Control Lab").xAxisTitle("Time (s)").yAxisTitle("Value").build();
        chartPanel = new XChartPanel<>(chart);

        JPanel filterPanel = new JPanel(new BorderLayout());
        filterPanel.add(abgContainer, BorderLayout.WEST);
        filterPanel.add(top,          BorderLayout.NORTH);
        filterPanel.add(chartPanel,   BorderLayout.CENTER);
        filterPanel.add(right,        BorderLayout.SOUTH);

        org.marsroboticsassociation.controllab.trajectory.TrajectoryTab trajectoryTab =
                new org.marsroboticsassociation.controllab.trajectory.TrajectoryTab();
        org.marsroboticsassociation.controllab.flywheel.FlywheelTab flywheelTab =
                new org.marsroboticsassociation.controllab.flywheel.FlywheelTab();
        JTabbedPane tabs = new JTabbedPane();
        tabs.addTab("Filter",     filterPanel);
        tabs.addTab("Trajectory", trajectoryTab);
        tabs.addTab("Flywheel",   flywheelTab);

        frame.addWindowListener(new java.awt.event.WindowAdapter() {
            @Override
            public void windowClosing(java.awt.event.WindowEvent e) {
                trajectoryTab.disposeEngine();
                flywheelTab.dispose();
            }
        });

        frame.add(tabs, BorderLayout.CENTER);

        updateControlsForFilter();

        openBtn.addActionListener(e -> onOpenCsv());
        filterTypeCombo.addActionListener(e -> updateControlsForFilter());
        applyBtn.addActionListener(e -> onApplyFilter());
        estimateBtn.addActionListener(e -> onEstimateLag());
        exportBtn.addActionListener(e -> onExportCsv());

        frame.setVisible(true);
    }

    private void updateControlsForFilter() {
        String sel = (String) filterTypeCombo.getSelectedItem();
        boolean isLP = "LOWPASS".equals(sel);
        boolean isBiquad = "BIQUAD".equals(sel);
        boolean isBessel = "BESSEL".equals(sel);
        boolean isABG = "ALPHA-BETA-GAMMA".equals(sel);

        boolean usesCutoff = (isLP || isBiquad || isBessel);

        cutoffSlider.setEnabled(usesCutoff);
        cutoffLabel.setEnabled(usesCutoff);

        qSlider.setEnabled(isBiquad);
        qLabel.setEnabled(isBiquad);

        abgContainer.setVisible(isABG);

        frame.revalidate();
    }

    private static File getDefaultDownloadsDir() {
        String home = System.getProperty("user.home");

        if (System.getProperty("os.name").toLowerCase().contains("win")) {
            return new File(home, "Downloads");
        }
        if (System.getProperty("os.name").toLowerCase().contains("mac")) {
            return new File(home + "/Downloads");
        }

        String xdg = System.getenv("XDG_DOWNLOAD_DIR");
        if (xdg != null) return new File(xdg);

        return new File(home, "Downloads");
    }

    private static void forceDetailsView(JFileChooser chooser) {
        SwingUtilities.invokeLater(() -> {
            Action details = chooser.getActionMap().get("viewTypeDetails");
            if (details != null) details.actionPerformed(null);
        });
    }

    private void onOpenCsv() {
        File downloads = getDefaultDownloadsDir();
        JFileChooser chooser = new JFileChooser(downloads);
        FileNameExtensionFilter csvFilter = new FileNameExtensionFilter(
                "CSV Files (*.csv)", "csv");
        chooser.setFileFilter(csvFilter);
        forceDetailsView(chooser);
        chooser.setDialogTitle("Open CSV");
        int ret = chooser.showOpenDialog(frame);
        if (ret != JFileChooser.APPROVE_OPTION) return;
        File f = chooser.getSelectedFile();
        try {
            loadedSignal = CsvSignal.load(f.getAbsolutePath());
            timeColCombo.removeAllItems();
            dataColCombo.removeAllItems();
            for (String h : loadedSignal.headers()) {
                timeColCombo.addItem(h);
                dataColCombo.addItem(h);
            }

            // Defaults
            if (loadedSignal.headers().contains("Run Time"))
                timeColCombo.setSelectedItem("Run Time");

            if (loadedSignal.headers().contains("flywheel TPS measured"))
                dataColCombo.setSelectedItem("flywheel TPS measured");

            String selectedTimeCol = (String) timeColCombo.getSelectedItem();
            String selectedDataCol = (String) dataColCombo.getSelectedItem();

            loadedSignal.select(selectedTimeCol, selectedDataCol);

            double min = loadedSignal.minTime().orElse(0.0);
            double max = loadedSignal.maxTime().orElse(1.0);
            startField.setText(String.format("%.3f", min));
            endField.setText(String.format("%.3f", max));
        } catch (Exception ex) {
            JOptionPane.showMessageDialog(frame, "Failed to load CSV: " + ex.getMessage());
            ex.printStackTrace();
        }
    }

    private void onApplyFilter() {
        if (loadedSignal == null) {
            JOptionPane.showMessageDialog(frame, "Load a CSV first.");
            return;
        }

        String timeCol = (String) timeColCombo.getSelectedItem();
        String dataCol = (String) dataColCombo.getSelectedItem();
        if (timeCol == null || dataCol == null) {
            JOptionPane.showMessageDialog(frame, "Select time and signal columns.");
            return;
        }

        double s = parseDoubleOrDefault(startField.getText(), Double.NEGATIVE_INFINITY);
        double e = parseDoubleOrDefault(endField.getText(), Double.POSITIVE_INFINITY);

        loadedSignal.select(timeCol, dataCol).window(s, e);
        List<Double> time = loadedSignal.time();
        List<Double> raw = loadedSignal.data();

        if (time.isEmpty() || raw.isEmpty()) {
            JOptionPane.showMessageDialog(frame, "No data in selected window.");
            return;
        }

        // create filter from UI
        String sel = (String) filterTypeCombo.getSelectedItem();
        FilterFactory.Type type = FilterFactory.Type.valueOf(sel.replace('-', '_').toUpperCase(Locale.ROOT));
        Filter filter;
        if (type == FilterFactory.Type.LOWPASS) {
            double cutoff = sliderToCutoff(cutoffSlider.getValue());
            filter = FilterFactory.create(FilterFactory.Type.LOWPASS, cutoff, Double.NaN, Double.NaN);

        } else if (type == FilterFactory.Type.BIQUAD) {
            double cutoff = sliderToCutoff(cutoffSlider.getValue());
            double q = sliderToQ(qSlider.getValue());
            filter = FilterFactory.create(FilterFactory.Type.BIQUAD, cutoff, q, Double.NaN);

        } else {
            filter = FilterFactory.create(FilterFactory.Type.NONE, Double.NaN, Double.NaN, Double.NaN);
        }

        // apply variable-dt filtering
        List<Double> filtered = new ArrayList<>(raw.size());
        List<Double> derivative = new ArrayList<>(1);
        filter.reset();
        double prevT = time.get(0);
        for (int i = 0; i < raw.size(); i++) {
            double t = time.get(i);
            double dt = i == 0 ? 0.0 : t - prevT;
            prevT = t;
            filtered.add(filter.update(raw.get(i), dt));
            double deriv = filter.getRate();
            if (!Double.isNaN(deriv))
                derivative.add(deriv);
        }

        if (derivative.isEmpty()) {
            derivative = null;
        }

        lastTime = time;
        lastRaw = raw;
        lastFiltered = filtered;

        redrawChart(time, raw, filtered, derivative);
    }

    private void onEstimateLag() {
        if (lastTime == null || lastRaw == null || lastFiltered == null) {
            JOptionPane.showMessageDialog(frame, "Run filter first.");
            return;
        }
        double lagSeconds = Utils.estimateLagSeconds(lastRaw, lastFiltered, lastTime);
        lagLabel.setText(String.format("Lag ≈ %.4f s", lagSeconds));
    }

    private void onExportCsv() {
        if (lastTime == null || lastRaw == null || lastFiltered == null) {
            JOptionPane.showMessageDialog(frame, "Run filter first.");
            return;
        }
        JFileChooser chooser = new JFileChooser();
        chooser.setDialogTitle("Save filtered CSV");
        int ret = chooser.showSaveDialog(frame);
        if (ret != JFileChooser.APPROVE_OPTION) return;
        File out = chooser.getSelectedFile();
        try {
            Utils.exportToCsv(out.toPath(), lastTime, lastRaw, lastFiltered);
            JOptionPane.showMessageDialog(frame, "Exported to: " + out.getAbsolutePath());
        } catch (Exception ex) {
            JOptionPane.showMessageDialog(frame, "Export failed: " + ex.getMessage());
            ex.printStackTrace();
        }
    }

    private void redrawChart(List<Double> time, List<Double> raw, List<Double> filtered, List<Double> derivative) {
        chart.getStyler().setMarkerSize(2);
        chart.getSeriesMap().clear();
        chart.addSeries("raw", time, raw).setMarker(SeriesMarkers.CIRCLE);
        chart.addSeries("filtered", time, filtered).setMarker(SeriesMarkers.NONE);
        if (derivative != null && !derivative.isEmpty()) {
            chart.addSeries("derivative", time, derivative).setMarker(SeriesMarkers.NONE);
        }
        chartPanel.revalidate();
        chartPanel.repaint();
    }

    private static double sliderScaling(int val, int sliderMax, int rangeScaling) {
        double norm = val / (double) sliderMax;
        double result = Math.pow(10, rangeScaling * norm - 1);
        return Math.max(1e-6, result);
    }

    private static int inverseSliderScaling(double val, int sliderMax, int rangeScaling) {
        val = Math.max(1e-6, val);
        double norm = (Math.log10(val) + 1.0) / (double) rangeScaling;
        int result = (int) Math.round(norm * (double) sliderMax);
        return MathUtil.clamp(result, 1, sliderMax);
    }

    // slider value mapping helper
    private static double sliderToCutoff(int val) {
        return sliderScaling(val, 1000, 3);
    }

    private static int cutoffToSlider(double hz) {
        return inverseSliderScaling(hz, 1000, 3);
    }

    private static double sliderToQ(int val) {
        double val2 = sliderScaling(val, 1000, 2);
        double butterworth = 1.0 / Math.sqrt(2.0);
        if (Math.abs(val2 - butterworth) < 0.02) {
            val2 = butterworth;
        }
        return val2;
    }

    private static int qToSlider(double q) {
        return inverseSliderScaling(q, 1000, 2);
    }

    private static double parseDoubleOrDefault(String s, double def) {
        if (s == null || s.isEmpty()) return def;
        try {
            return Double.parseDouble(s.trim());
        } catch (NumberFormatException ex) {
            return def;
        }
    }
}
