package org.marsroboticsassociation.controllab.trajectory;

import org.knowm.xchart.*;
import org.knowm.xchart.internal.series.AxesChartSeries;
import org.knowm.xchart.style.markers.SeriesMarkers;
import org.marsroboticsassociation.controllab.flywheel.EditableParamField;
import org.marsroboticsassociation.controllib.motion.SCurveVelocity;
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment;

import java.awt.*;
import java.io.File;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;

public class TrajectoryTab extends JPanel {

    private static final int SIDEBAR_WIDTH = 300;
    private static final int TIMER_MS = 20;
    private static final int BUFFER_POINTS = 500;
    private static final double WINDOW_SECS = 10.0;
    private static final DateTimeFormatter EXPORT_TIMESTAMP =
            DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");
    private static final double MOTOR_VOLTAGE = 12.0;

    private TrajectoryEngine engine;
    private final RollingBuffer buffer = new RollingBuffer(WINDOW_SECS, BUFFER_POINTS, 5);
    private double elapsedSec = 0.0;
    private final Map<String, List<TrajectoryCurveSegment>> exactHistorySegments =
            new LinkedHashMap<>();
    private TrajectorySvgModel activeExactPlan;
    private double activeExactPlanStartSec = Double.NaN;
    private boolean backEmfViolation = false;
    private List<double[]> violationRanges = new ArrayList<>();

    private XYChart chart;
    private XChartPanel<XYChart> chartPanel;
    private JLayeredPane layeredPane;
    private ViolationBandPanel violationBandPanel;

    private JComboBox<TrajectoryType> typeCombo;
    private JPanel limitPanel;

    // SCurvePosition sliders
    private JSlider slVMax, slAAccel, slADecel, slJMax;
    private JLabel lbVMax, lbAAccel, lbADecel, lbJMax;

    // SCurveVelocity sliders
    private JSlider slAMax, slJInc, slJDec;
    private JLabel lbAMax, lbJInc, lbJDec;

    // Ruckig sliders
    private JSlider slRVMax, slRAMax, slRJMax;
    private JLabel lbRVMax, lbRAMax, lbRJMax;

    // Back-EMF motor characterization (for SCurveVelocity)
    private EditableParamField efKs, efKv, efKa;
    private JPanel bemfPanel;

    // Target sliders
    private JSlider slTargetA, slTargetB;
    private JLabel lbTargetA, lbTargetB;
    private JButton btnGoA, btnGoB;
    private JButton btnExportSvg;
    private JButton btnReset;
    private JButton btnAutoTune;
    private Timer simTimer;

    static double s2l(int v, double min, double max) {
        return min + (max - min) * v / 1000.0;
    }

    static int l2s(double v, double min, double max) {
        return (int) Math.round((v - min) / (max - min) * 1000.0);
    }

    public TrajectoryTab() {
        setLayout(new BorderLayout());
        engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        buildChart();
        buildSidebar();
        buildTimer();
        onTypeChanged();
    }

    private void buildChart() {
        chart =
                new XYChartBuilder()
                        .width(900)
                        .height(600)
                        .title("Trajectory")
                        .xAxisTitle("Time (s)")
                        .yAxisTitle("Value")
                        .build();
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setLegendPosition(org.knowm.xchart.style.Styler.LegendPosition.InsideNW);
        chart.addSeries("Position (units)", new double[] {0}, new double[] {0})
                .setMarker(SeriesMarkers.NONE);
        chart.addSeries("Velocity (units/s)", new double[] {0}, new double[] {0})
                .setMarker(SeriesMarkers.NONE);
        chart.addSeries("Acceleration (units/s\u00b2)", new double[] {0}, new double[] {0})
                .setMarker(SeriesMarkers.NONE);
        chart.addSeries("Max Motor Accel (units/s\u00b2)", new double[] {0}, new double[] {0})
                .setMarker(SeriesMarkers.NONE)
                .setLineStyle(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, new float[] {4.0f, 4.0f}, 0.0f));
        chart.addSeries("Target", new double[] {0}, new double[] {0})
                .setMarker(SeriesMarkers.NONE)
                .setLineStyle(
                        new java.awt.BasicStroke(
                                1.5f,
                                java.awt.BasicStroke.CAP_BUTT,
                                java.awt.BasicStroke.JOIN_MITER,
                                10.0f,
                                new float[] {6.0f, 4.0f},
                                0.0f));
        chartPanel = new XChartPanel<>(chart);
        
        layeredPane = new JLayeredPane() {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(900, 600);
            }
            
            @Override
            public void doLayout() {
                int w = getWidth();
                int h = getHeight();
                if (w <= 0 || h <= 0) {
                    w = 900;
                    h = 600;
                }
                violationBandPanel.setBounds(0, 0, w, h);
                chartPanel.setBounds(0, 0, w, h);
                super.doLayout();
            }
        };
        
        violationBandPanel = new ViolationBandPanel();
        violationBandPanel.setOpaque(false);

        layeredPane.add(chartPanel, JLayeredPane.DEFAULT_LAYER);
        layeredPane.add(violationBandPanel, JLayeredPane.PALETTE_LAYER);
        
        add(layeredPane, BorderLayout.CENTER);
    }
    
    private class ViolationBandPanel extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            if (violationRanges.isEmpty() || getWidth() <= 0 || getHeight() <= 0) return;

            // Derive the plot area bounds from two known screen-coordinate mappings.
            // getScreenYFromChart(dataY) is linear: screenY = slope * dataY + intercept.
            // Two points let us solve for slope and intercept, then find the plot edges
            // (where dataY equals the axis min/max), but we don't know axis min/max either.
            // Instead, use reflection to access the package-private getPlot().getBounds().
            java.awt.geom.Rectangle2D plotBounds;
            try {
                var m = chart.getClass().getSuperclass().getDeclaredMethod("getPlot");
                m.setAccessible(true);
                Object plot = m.invoke(chart);
                plotBounds = (java.awt.geom.Rectangle2D) plot.getClass().getMethod("getBounds").invoke(plot);
            } catch (Exception e) {
                return; // can't determine plot area
            }
            if (plotBounds == null || plotBounds.getWidth() <= 0) return;

            int plotLeft = (int) plotBounds.getX();
            int plotRight = (int) (plotBounds.getX() + plotBounds.getWidth());
            int plotTop = (int) plotBounds.getY();
            int plotH = (int) plotBounds.getHeight();

            Graphics2D g2 = (Graphics2D) g;
            g2.setColor(new Color(180, 180, 180, 120));

            for (double[] range : violationRanges) {
                int x1 = (int) chart.getScreenXFromChart(range[0]);
                int x2 = (int) chart.getScreenXFromChart(range[1]);

                x1 = Math.max(plotLeft, x1);
                x2 = Math.min(plotRight, x2);

                if (x2 > x1) {
                    g2.fillRect(x1, plotTop, x2 - x1, plotH);
                }
            }
        }
    }

    private void buildSidebar() {
        JPanel sidebar = new JPanel();
        sidebar.setLayout(new BoxLayout(sidebar, BoxLayout.Y_AXIS));
        sidebar.setPreferredSize(new Dimension(SIDEBAR_WIDTH, 0));
        sidebar.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

        typeCombo = new JComboBox<>(TrajectoryType.values());
        typeCombo.setMaximumSize(new Dimension(Integer.MAX_VALUE, 28));
        sidebar.add(typeCombo);
        sidebar.add(Box.createVerticalStrut(12));

        if (!TrajectoryEngine.isRuckigAvailable()) {
            typeCombo.setRenderer(
                    new DefaultListCellRenderer() {
                        @Override
                        public Component getListCellRendererComponent(
                                JList<?> list,
                                Object value,
                                int index,
                                boolean sel,
                                boolean focus) {
                            JLabel lbl =
                                    (JLabel)
                                            super.getListCellRendererComponent(
                                                    list, value, index, sel, focus);
                            if (value == TrajectoryType.RUCKIG) {
                                lbl.setEnabled(false);
                                lbl.setToolTipText(
                                        "Ruckig JNI library not built. Run: gradlew"
                                                + " :ControlLab:buildRuckigDesktop");
                            }
                            return lbl;
                        }
                    });
        }

        sidebar.add(boldLabel("Limits"));
        sidebar.add(Box.createVerticalStrut(4));

        limitPanel = new JPanel();
        limitPanel.setLayout(new BoxLayout(limitPanel, BoxLayout.Y_AXIS));
        sidebar.add(limitPanel);
        sidebar.add(Box.createVerticalStrut(12));

        sidebar.add(boldLabel("Motor (back-EMF)"));
        sidebar.add(Box.createVerticalStrut(4));

        bemfPanel = new JPanel();
        bemfPanel.setLayout(new BoxLayout(bemfPanel, BoxLayout.Y_AXIS));
        buildBackEmfPanelIfNeeded();
        bemfPanel.setVisible(false);
        sidebar.add(bemfPanel);

        sidebar.add(boldLabel("Targets"));
        sidebar.add(Box.createVerticalStrut(4));

        slTargetA = new JSlider(0, 1000, l2s(-100, -200, 200));
        lbTargetA = new JLabel();
        slTargetB = new JSlider(0, 1000, l2s(100, -200, 200));
        lbTargetB = new JLabel();

        slTargetA.putClientProperty("min", -200.0);
        slTargetA.putClientProperty("max", 200.0);
        slTargetB.putClientProperty("min", -200.0);
        slTargetB.putClientProperty("max", 200.0);

        slTargetA.addChangeListener(e -> updateTargetLabel(lbTargetA, slTargetA, "A"));
        slTargetB.addChangeListener(e -> updateTargetLabel(lbTargetB, slTargetB, "B"));
        updateTargetLabel(lbTargetA, slTargetA, "A");
        updateTargetLabel(lbTargetB, slTargetB, "B");

        sidebar.add(lbTargetA);
        sidebar.add(slTargetA);
        sidebar.add(Box.createVerticalStrut(4));
        sidebar.add(lbTargetB);
        sidebar.add(slTargetB);
        sidebar.add(Box.createVerticalStrut(12));

        btnGoA = new JButton("\u2192 Go to A");
        btnGoB = new JButton("\u2192 Go to B");
        btnExportSvg = new JButton("Export SVG");
        btnGoA.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnGoB.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnExportSvg.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));

        sidebar.add(btnGoA);
        sidebar.add(Box.createVerticalStrut(6));
        sidebar.add(btnGoB);
        sidebar.add(Box.createVerticalStrut(12));
        sidebar.add(btnExportSvg);
        
        btnReset = new JButton("\u21bb Reset to 0");
        btnReset.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnReset.setVisible(false);
        sidebar.add(Box.createVerticalStrut(8));
        sidebar.add(btnReset);
        
        btnAutoTune = new JButton("Auto-Tune");
        btnAutoTune.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnAutoTune.setVisible(false);
        btnAutoTune.setToolTipText("Find optimal jDec that maximizes speed without violating back-EMF");
        sidebar.add(Box.createVerticalStrut(8));
        sidebar.add(btnAutoTune);
        
        
        typeCombo.addActionListener(e -> onTypeChanged());
        btnGoA.addActionListener(e -> onGoTo(slTargetA));
        btnGoB.addActionListener(e -> onGoTo(slTargetB));
        btnExportSvg.addActionListener(e -> onExportSvg());
        btnReset.addActionListener(e -> onReset());
        btnAutoTune.addActionListener(e -> onAutoTune());

        add(sidebar, BorderLayout.WEST);
    }

    private void buildPosSlidersIfNeeded() {
        if (slVMax != null) return;
        slVMax = new JSlider(0, 1000, l2s(72, 0.1, 500));
        lbVMax = new JLabel();
        slAAccel = new JSlider(0, 1000, l2s(52, 0.1, 500));
        lbAAccel = new JLabel();
        slADecel = new JSlider(0, 1000, l2s(52, 0.1, 500));
        lbADecel = new JLabel();
        slJMax = new JSlider(0, 1000, l2s(100, 1, 5000));
        lbJMax = new JLabel();

        slVMax.addChangeListener(
                e -> {
                    updateLabel(lbVMax, slVMax, "vMax", 0.1, 500);
                    stagePosParams();
                });
        slAAccel.addChangeListener(
                e -> {
                    updateLabel(lbAAccel, slAAccel, "aAccel", 0.1, 500);
                    stagePosParams();
                });
        slADecel.addChangeListener(
                e -> {
                    updateLabel(lbADecel, slADecel, "aDecel", 0.1, 500);
                    stagePosParams();
                });
        slJMax.addChangeListener(
                e -> {
                    updateLabel(lbJMax, slJMax, "jMax", 1, 5000);
                    stagePosParams();
                });
        updateLabel(lbVMax, slVMax, "vMax", 0.1, 500);
        updateLabel(lbAAccel, slAAccel, "aAccel", 0.1, 500);
        updateLabel(lbADecel, slADecel, "aDecel", 0.1, 500);
        updateLabel(lbJMax, slJMax, "jMax", 1, 5000);
        stagePosParams();
    }

    private void buildVelSlidersIfNeeded() {
        if (slAMax != null) return;
        slAMax = new JSlider(0, 1000, l2s(1197, 10, 5000));
        lbAMax = new JLabel();
        slJInc = new JSlider(0, 1000, l2s(2669, 10, 10000));
        lbJInc = new JLabel();
        slJDec = new JSlider(0, 1000, l2s(800, 10, 5000));
        lbJDec = new JLabel();

        slAMax.addChangeListener(
                e -> {
                    updateLabel(lbAMax, slAMax, "aMax", 10, 5000);
                    stageVelParams();
                    checkBackEmfViolation();
                });
        slJInc.addChangeListener(
                e -> {
                    updateLabel(lbJInc, slJInc, "jInc", 10, 10000);
                    stageVelParams();
                    checkBackEmfViolation();
                });
        slJDec.addChangeListener(
                e -> {
                    updateLabel(lbJDec, slJDec, "jDec", 10, 5000);
                    stageVelParams();
                    checkBackEmfViolation();
                });
        updateLabel(lbAMax, slAMax, "aMax", 10, 5000);
        updateLabel(lbJInc, slJInc, "jInc", 10, 10000);
        updateLabel(lbJDec, slJDec, "jDec", 10, 5000);
        stageVelParams();
    }

    private void buildRuckigSlidersIfNeeded() {
        if (slRVMax != null) return;
        slRVMax = new JSlider(0, 1000, l2s(10, 0.1, 500));
        lbRVMax = new JLabel();
        slRAMax = new JSlider(0, 1000, l2s(5, 0.1, 500));
        lbRAMax = new JLabel();
        slRJMax = new JSlider(0, 1000, l2s(50, 1, 5000));
        lbRJMax = new JLabel();

        slRVMax.addChangeListener(
                e -> {
                    updateLabel(lbRVMax, slRVMax, "vMax", 0.1, 500);
                    stageRuckigParams();
                    checkBackEmfViolation();
                });
        slRAMax.addChangeListener(
                e -> {
                    updateLabel(lbRAMax, slRAMax, "aMax", 0.1, 500);
                    stageRuckigParams();
                    checkBackEmfViolation();
                });
        slRJMax.addChangeListener(
                e -> {
                    updateLabel(lbRJMax, slRJMax, "jMax", 1, 5000);
                    stageRuckigParams();
                    checkBackEmfViolation();
                });
        updateLabel(lbRVMax, slRVMax, "vMax", 0.1, 500);
        updateLabel(lbRAMax, slRAMax, "aMax", 0.1, 500);
        updateLabel(lbRJMax, slRJMax, "jMax", 1, 5000);
        stageRuckigParams();
    }

    private void buildBackEmfPanelIfNeeded() {
        if (efKs != null) return;
        efKs = new EditableParamField("kS", 0.893, "%.3f", 0, 12.0, v -> checkBackEmfViolation());
        efKv = new EditableParamField("kV", 0.00475, "%.6f", 0, 1.0, v -> checkBackEmfViolation());
        efKa = new EditableParamField("kA", 0.00599, "%.6f", 1e-9, 1.0, v -> checkBackEmfViolation());
        bemfPanel.add(efKs);
        bemfPanel.add(efKv);
        bemfPanel.add(efKa);
    }

    private void stagePosParams() {
        engine.setPositionParams(
                s2l(slVMax.getValue(), 0.1, 500),
                s2l(slAAccel.getValue(), 0.1, 500),
                s2l(slADecel.getValue(), 0.1, 500),
                s2l(slJMax.getValue(), 1, 5000));
    }

    private void stageVelParams() {
        engine.setVelocityParams(
                s2l(slAMax.getValue(), 10, 5000),
                s2l(slJInc.getValue(), 10, 10000),
                s2l(slJDec.getValue(), 10, 5000));
    }

    private void stageRuckigParams() {
        engine.setRuckigParams(
                s2l(slRVMax.getValue(), 0.1, 500),
                s2l(slRAMax.getValue(), 0.1, 500),
                s2l(slRJMax.getValue(), 1, 5000));
    }

    private void onTypeChanged() {
        TrajectoryType sel = (TrajectoryType) typeCombo.getSelectedItem();

        if (sel == TrajectoryType.RUCKIG && !TrajectoryEngine.isRuckigAvailable()) {
            typeCombo.setSelectedItem(engine.getType());
            JOptionPane.showMessageDialog(
                    this,
                    "Ruckig JNI library not built.\nRun: gradlew :ControlLab:buildRuckigDesktop",
                    "Ruckig Unavailable",
                    JOptionPane.WARNING_MESSAGE);
            return;
        }

        engine.switchType(sel);
        buffer.clear();
        elapsedSec = 0.0;
        resetExactExportHistory();

        limitPanel.removeAll();
        switch (sel) {
            case SCURVE_POSITION:
            case SIN_CURVE_POSITION:
                buildPosSlidersIfNeeded();
                addSliderRow(limitPanel, lbVMax, slVMax);
                addSliderRow(limitPanel, lbAAccel, slAAccel);
                addSliderRow(limitPanel, lbADecel, slADecel);
                addSliderRow(limitPanel, lbJMax, slJMax);
                bemfPanel.setVisible(false);
                btnReset.setVisible(false);
                btnAutoTune.setVisible(false);
                updateTargetRange(-200, 200, -100, 100);
                break;
            case SCURVE_VELOCITY:
                buildVelSlidersIfNeeded();
                buildBackEmfPanelIfNeeded();
                addSliderRow(limitPanel, lbAMax, slAMax);
                addSliderRow(limitPanel, lbJInc, slJInc);
                addSliderRow(limitPanel, lbJDec, slJDec);
                bemfPanel.setVisible(true);
                btnReset.setVisible(true);
                btnAutoTune.setVisible(true);
                double ks = efKs.getValue();
                double kv = efKv.getValue();
                double maxV = kv > 0 ? (MOTOR_VOLTAGE - ks) / kv : 6000;
                updateTargetRange(0, maxV, Math.min(2000, maxV), 0);
                checkBackEmfViolation();
                break;
            case RUCKIG:
                buildRuckigSlidersIfNeeded();
                addSliderRow(limitPanel, lbRVMax, slRVMax);
                addSliderRow(limitPanel, lbRAMax, slRAMax);
                addSliderRow(limitPanel, lbRJMax, slRJMax);
                bemfPanel.setVisible(false);
                btnReset.setVisible(false);
                updateTargetRange(-200, 200, -100, 100);
                break;
        }
        limitPanel.revalidate();
        limitPanel.repaint();

        chart.getSeriesMap().get("Position (units)").setEnabled(engine.hasPosition());
        refreshExportButtonState();
        chartPanel.repaint();
    }

    private void updateTargetRange(double min, double max, double defA, double defB) {
        slTargetA.putClientProperty("min", min);
        slTargetA.putClientProperty("max", max);
        slTargetB.putClientProperty("min", min);
        slTargetB.putClientProperty("max", max);
        slTargetA.setValue(l2s(defA, min, max));
        slTargetB.setValue(l2s(defB, min, max));
        updateTargetLabel(lbTargetA, slTargetA, "A");
        updateTargetLabel(lbTargetB, slTargetB, "B");
    }

    private double targetSliderValue(JSlider sl) {
        Object minObj = sl.getClientProperty("min");
        Object maxObj = sl.getClientProperty("max");
        double min = (minObj != null) ? (double) minObj : -200.0;
        double max = (maxObj != null) ? (double) maxObj : 200.0;
        return s2l(sl.getValue(), min, max);
    }

    private void onGoTo(JSlider targetSlider) {
        double targetValue = targetSliderValue(targetSlider);
        commitActiveExactPlanThrough(elapsedSec);
        engine.applyParamsAndGoTo(targetValue);
        startActiveExactPlan();
        recordCurrentSample();
        refreshExportButtonState();
        checkBackEmfViolation();
    }

    private void onReset() {
        elapsedSec = 0.0;
        buffer.clear();
        engine.reset();
        resetExactExportHistory();
        recordCurrentSample();
        refreshExportButtonState();
    }

    private void onAutoTune() {
        double jInc = s2l(slJInc.getValue(), 10, 10000);
        double targetV = engine.getTarget();
        
        if (targetV <= 0) {
            JOptionPane.showMessageDialog(this, "Set a target velocity first", "Auto-Tune", JOptionPane.WARNING_MESSAGE);
            return;
        }
        
        double ks = efKs.getValue();
        double kv = efKv.getValue();
        double ka = efKa.getValue();
        
        if (ka <= 0) {
            JOptionPane.showMessageDialog(this, "kA must be positive for back-EMF calculation", "Auto-Tune", JOptionPane.WARNING_MESSAGE);
            return;
        }
        
        double aMax = SCurveVelocity.findMaxAMax(0, targetV, jInc, MOTOR_VOLTAGE, ks, kv, ka);
        double jDec = SCurveVelocity.findMaxJDec(0, targetV, 0, aMax, jInc, MOTOR_VOLTAGE, ks, kv, ka);
        
        slAMax.setValue(l2s(aMax, 10, 5000));
        updateLabel(lbAMax, slAMax, "aMax", 10, 5000);
        slJDec.setValue(l2s(jDec, 10, 5000));
        updateLabel(lbJDec, slJDec, "jDec", 10, 5000);
        stageVelParams();
        
        checkBackEmfViolation();
    }

    private void buildTimer() {
        simTimer = new Timer(TIMER_MS, e -> onTick());
        simTimer.start();
    }

    private void onTick() {
        if (!engine.isMoving()) return;
        engine.tick();
        elapsedSec += TrajectoryEngine.CYCLE_S;
        if (!engine.isMoving()) {
            commitActiveExactPlanThrough(elapsedSec);
        }
        recordCurrentSample();
        refreshExportButtonState();
    }

    private void recordCurrentSample() {
        double velocity = engine.getVelocity();
        double maxMotorAccel = maxMotorAcceleration(velocity);
        buffer.add(
                elapsedSec,
                engine.getPosition(),
                engine.getVelocity(),
                engine.getAcceleration(),
                engine.getTarget(),
                maxMotorAccel);
        List<Double> times = buffer.getTimes();
        chart.updateXYSeries("Position (units)", times, buffer.getPositions(), null);
        chart.updateXYSeries("Velocity (units/s)", times, buffer.getVelocities(), null);
        chart.updateXYSeries(
                "Acceleration (units/s\u00b2)", times, buffer.getAccelerations(), null);
        boolean showMaxMotorAccel = engine.getType() == TrajectoryType.SCURVE_VELOCITY;
        chart.getSeriesMap().get("Max Motor Accel (units/s\u00b2)").setEnabled(showMaxMotorAccel);
        if (showMaxMotorAccel) {
            chart.updateXYSeries("Max Motor Accel (units/s\u00b2)", times, buffer.getMaxMotorAccels(), null);
        }
        chart.updateXYSeries("Target", times, buffer.getTargets(), null);
        chartPanel.repaint();
        if (times.size() >= 2) {
            checkBackEmfViolation();
        }
    }

    private void refreshExportButtonState() {
        boolean visible = engine.supportsExactSvgExport();
        btnExportSvg.setVisible(visible);
        btnExportSvg.setEnabled(visible && !engine.isMoving());
        btnExportSvg.setToolTipText(
                visible
                        ? (engine.isMoving()
                                ? "Export becomes available once the graph settles"
                                : "Export an exact SVG of the current trajectory")
                        : null);
        revalidate();
        repaint();
    }

    private void onExportSvg() {
        if (engine.isMoving()) {
            JOptionPane.showMessageDialog(
                    this,
                    "Wait for the trajectory to settle before exporting.",
                    "Trajectory Still Running",
                    JOptionPane.INFORMATION_MESSAGE);
            return;
        }

        TrajectorySvgModel model = styledSvgModel();
        if (model == null) {
            JOptionPane.showMessageDialog(
                    this,
                    "SVG export is only available for SCurvePosition and SCurveVelocity.",
                    "Export Unavailable",
                    JOptionPane.WARNING_MESSAGE);
            return;
        }

        JFileChooser chooser = new JFileChooser(defaultDownloadsDir());
        chooser.setDialogTitle("Save trajectory SVG");
        chooser.setFileFilter(new FileNameExtensionFilter("SVG Files (*.svg)", "svg"));
        chooser.setSelectedFile(new File(defaultDownloadsDir(), defaultSvgFilename()));
        int ret = chooser.showSaveDialog(this);
        if (ret != JFileChooser.APPROVE_OPTION) return;

        File out = chooser.getSelectedFile();
        if (!out.getName().toLowerCase().endsWith(".svg")) {
            out = new File(out.getParentFile(), out.getName() + ".svg");
        }
        if (out.exists()) {
            int choice =
                    JOptionPane.showConfirmDialog(
                            this,
                            "Replace existing file?\n" + out.getAbsolutePath(),
                            "Confirm Replace",
                            JOptionPane.YES_NO_OPTION,
                            JOptionPane.WARNING_MESSAGE);
            if (choice != JOptionPane.YES_OPTION) return;
        }

        try {
            TrajectorySvgExporter.export(Path.of(out.getAbsolutePath()), model);
            JOptionPane.showMessageDialog(this, "Exported to: " + out.getAbsolutePath());
        } catch (Exception ex) {
            JOptionPane.showMessageDialog(
                    this,
                    "Export failed: " + ex.getMessage(),
                    "Export Failed",
                    JOptionPane.ERROR_MESSAGE);
        }
    }

    private TrajectorySvgModel styledSvgModel() {
        TrajectorySvgModel base = visibleExactSvgModel();
        if (base == null) return null;

        java.awt.Color[] palette = chart.getStyler().getSeriesColors();

        List<TrajectorySvgSeries> styledSeries =
                java.util.stream.IntStream.range(0, base.series().size())
                        .mapToObj(
                                index -> {
                                    TrajectorySvgSeries series = base.series().get(index);
                                    AxesChartSeries chartSeries =
                                            (AxesChartSeries) chart.getSeriesMap().get(series.label());
                                    if (chartSeries == null) return series;
                                    java.awt.BasicStroke stroke =
                                            chartSeries.getLineStyle() != null
                                                    ? chartSeries.getLineStyle()
                                                    : series.stroke();
                                    float strokeWidth =
                                            chartSeries.getLineWidth() > 0
                                                    ? chartSeries.getLineWidth()
                                                    : series.strokeWidth();
                                    java.awt.Color color =
                                            chartSeries.getLineColor() != null
                                                    ? chartSeries.getLineColor()
                                                    : paletteColor(palette, index, series.color());
                                    return new TrajectorySvgSeries(
                                            series.label(),
                                            series.segments(),
                                            color,
                                            strokeWidth,
                                            stroke);
                                })
                        .toList();

        return new TrajectorySvgModel(base.xMin(), base.xMax(), base.minY(), base.maxY(), styledSeries);
    }

    private static java.awt.Color paletteColor(
            java.awt.Color[] palette, int index, java.awt.Color fallback) {
        if (palette == null || palette.length == 0) return fallback;
        java.awt.Color color = palette[Math.floorMod(index, palette.length)];
        return color != null ? color : fallback;
    }

    private void resetExactExportHistory() {
        exactHistorySegments.clear();
        activeExactPlan = null;
        activeExactPlanStartSec = Double.NaN;
    }

    private void startActiveExactPlan() {
        if (!engine.supportsExactSvgExport()) {
            activeExactPlan = null;
            activeExactPlanStartSec = Double.NaN;
            return;
        }
        activeExactPlan = engine.buildExactSvgModel();
        activeExactPlanStartSec = elapsedSec;
    }

    private void commitActiveExactPlanThrough(double absoluteEndSec) {
        if (activeExactPlan == null || Double.isNaN(activeExactPlanStartSec)) return;
        double duration = Math.min(activeExactPlan.xMax() - activeExactPlan.xMin(), absoluteEndSec - activeExactPlanStartSec);
        if (duration > 0) {
            appendPlanSegments(exactHistorySegments, activeExactPlan, activeExactPlanStartSec, 0.0, duration);
        }
        activeExactPlan = null;
        activeExactPlanStartSec = Double.NaN;
    }

    private TrajectorySvgModel visibleExactSvgModel() {
        if (!engine.supportsExactSvgExport()) return null;

        Map<String, List<TrajectoryCurveSegment>> combined = new LinkedHashMap<>();
        for (Map.Entry<String, List<TrajectoryCurveSegment>> entry :
                exactHistorySegments.entrySet()) {
            combined.put(entry.getKey(), new ArrayList<>(entry.getValue()));
        }

        if (activeExactPlan != null && !Double.isNaN(activeExactPlanStartSec)) {
            double duration =
                    Math.min(
                            activeExactPlan.xMax() - activeExactPlan.xMin(),
                            Math.max(0.0, elapsedSec - activeExactPlanStartSec));
            appendPlanSegments(combined, activeExactPlan, activeExactPlanStartSec, 0.0, duration);
        }

        List<Double> times = buffer.getTimes();
        if (times.isEmpty()) return null;
        double xMin = times.get(0);
        double xMax = times.get(times.size() - 1);
        if (xMax <= xMin) xMax = xMin + 1e-9;

        List<TrajectorySvgSeries> series = new ArrayList<>();
        for (Map.Entry<String, List<TrajectoryCurveSegment>> entry :
                combined.entrySet()) {
            List<TrajectoryCurveSegment> clipped = new ArrayList<>();
            for (TrajectoryCurveSegment segment : entry.getValue()) {
                TrajectoryCurveSegment clippedSegment = segment.clippedTo(xMin, xMax);
                if (clippedSegment != null) {
                    clipped.add(clippedSegment);
                }
            }
            if (!clipped.isEmpty()) {
                series.add(new TrajectorySvgSeries(entry.getKey(), clipped, Color.BLACK, 2.0f, null));
            }
        }

        if (series.isEmpty()) return null;
        return new TrajectorySvgModel(xMin, xMax, minY(series), maxY(series), series);
    }

    private static void appendPlanSegments(
            Map<String, List<TrajectoryCurveSegment>> target,
            TrajectorySvgModel plan,
            double absoluteStartSec,
            double relativeStartSec,
            double relativeEndSec) {
        if (relativeEndSec <= relativeStartSec) return;
        for (TrajectorySvgSeries series : plan.series()) {
            List<TrajectoryCurveSegment> dst =
                    target.computeIfAbsent(series.label(), key -> new ArrayList<>());
            for (TrajectoryCurveSegment segment : series.segments()) {
                TrajectoryCurveSegment clipped = segment.clippedTo(relativeStartSec, relativeEndSec);
                if (clipped != null) {
                    dst.add(clipped.shiftedBy(absoluteStartSec));
                }
            }
        }
    }

    private static double minY(List<TrajectorySvgSeries> seriesList) {
        double min = Double.POSITIVE_INFINITY;
        for (TrajectorySvgSeries series : seriesList) {
            for (TrajectoryCurveSegment segment : series.segments()) {
                min = Math.min(min, segment.minValue());
            }
        }
        return Double.isFinite(min) ? min : -1.0;
    }

    private static double maxY(List<TrajectorySvgSeries> seriesList) {
        double max = Double.NEGATIVE_INFINITY;
        for (TrajectorySvgSeries series : seriesList) {
            for (TrajectoryCurveSegment segment : series.segments()) {
                max = Math.max(max, segment.maxValue());
            }
        }
        return Double.isFinite(max) ? max : 1.0;
    }

    private String defaultSvgFilename() {
        String timestamp = LocalDateTime.now().format(EXPORT_TIMESTAMP);
        return switch (engine.getType()) {
            case SCURVE_POSITION -> "scurve-position-trajectory-" + timestamp + ".svg";
            case SCURVE_VELOCITY -> "scurve-velocity-trajectory-" + timestamp + ".svg";
            default -> "trajectory-" + timestamp + ".svg";
        };
    }

    private static File defaultDownloadsDir() {
        return new File(System.getProperty("user.home"), "Downloads");
    }

    private static void addSliderRow(JPanel panel, JLabel label, JSlider slider) {
        panel.add(label);
        panel.add(slider);
        panel.add(Box.createVerticalStrut(4));
    }

    private static JLabel boldLabel(String text) {
        JLabel l = new JLabel(text);
        l.setFont(l.getFont().deriveFont(Font.BOLD));
        return l;
    }

    private static void updateLabel(JLabel lbl, JSlider sl, String name, double min, double max) {
        lbl.setText(String.format("%s: %.2f", name, s2l(sl.getValue(), min, max)));
    }

    private static void updateTargetLabel(JLabel lbl, JSlider sl, String name) {
        Object minObj = sl.getClientProperty("min");
        Object maxObj = sl.getClientProperty("max");
        double min = (minObj != null) ? (double) minObj : -200.0;
        double max = (maxObj != null) ? (double) maxObj : 200.0;
        lbl.setText(String.format("Target %s: %.1f", name, s2l(sl.getValue(), min, max)));
    }

    private double maxMotorAcceleration(double velocity) {
        double ks = efKs.getValue();
        double kv = efKv.getValue();
        double ka = efKa.getValue();
        if (ka <= 0) return Double.POSITIVE_INFINITY;
        double availableVoltage = MOTOR_VOLTAGE - ks - kv * Math.abs(velocity);
        if (availableVoltage <= 0) return 0;
        return availableVoltage / ka;
    }

    private void checkBackEmfViolation() {
        if (engine == null || engine.getType() != TrajectoryType.SCURVE_VELOCITY) {
            backEmfViolation = false;
            violationRanges.clear();
            updateViolationBandPanel();
            return;
        }

        double ka = efKa.getValue();
        double targetV = engine.getTarget();

        if (targetV <= 0) {
            backEmfViolation = false;
            violationRanges.clear();
            updateViolationBandPanel();
            return;
        }
        
        if (ka <= 0) {
            backEmfViolation = false;
            violationRanges.clear();
            updateViolationBandPanel();
            return;
        }
        
        List<Double> times = buffer.getTimes();
        List<Double> velocities = buffer.getVelocities();
        List<Double> accelerations = buffer.getAccelerations();
        
        violationRanges.clear();
        
        if (times.size() < 2) {
            updateViolationBandPanel();
            return;
        }
        
        boolean inViolation = false;
        double rangeStart = 0;
        
        double tolerance = 0.0;
        for (int i = 0; i < times.size(); i++) {
            double t = times.get(i);
            double v = velocities.get(i);
            double a = accelerations.get(i);
            
            double motorAMax = maxMotorAcceleration(v);
            boolean isViolating = a > motorAMax + tolerance;
            
            if (isViolating && !inViolation) {
                rangeStart = t;
                inViolation = true;
            } else if (!isViolating && inViolation) {
                violationRanges.add(new double[]{rangeStart, t});
                inViolation = false;
            }
        }
        
        if (inViolation && times.size() > 0) {
            violationRanges.add(new double[]{rangeStart, times.get(times.size() - 1)});
        }
        
        backEmfViolation = !violationRanges.isEmpty();

        updateViolationBandPanel();
    }

    private void updateViolationBandPanel() {
        if (violationBandPanel != null) {
            violationBandPanel.repaint();
        }
    }

    /** Release native resources held by the engine. Call on application shutdown. */
    public void disposeEngine() {
        simTimer.stop();
        engine.dispose();
    }
}
