package org.marsroboticsassociation.controllab.trajectory;

import org.knowm.xchart.*;
import org.knowm.xchart.style.markers.SeriesMarkers;

import javax.swing.*;
import java.awt.*;
import java.util.List;

public class TrajectoryTab extends JPanel {

    private static final int    SIDEBAR_WIDTH  = 260;
    private static final int    TIMER_MS       = 20;
    private static final int    BUFFER_POINTS  = 500;
    private static final double WINDOW_SECS    = 10.0;

    private TrajectoryEngine engine;
    private final RollingBuffer buffer = new RollingBuffer(WINDOW_SECS, BUFFER_POINTS);
    private double elapsedSec = 0.0;

    private XYChart chart;
    private XChartPanel<XYChart> chartPanel;

    private JComboBox<TrajectoryType> typeCombo;
    private JPanel limitPanel;

    // SCurvePosition sliders
    private JSlider slVMax, slAAccel, slADecel, slJMax;
    private JLabel  lbVMax, lbAAccel, lbADecel, lbJMax;

    // SCurveVelocity sliders
    private JSlider slAMax, slJInc, slJDec;
    private JLabel  lbAMax, lbJInc, lbJDec;

    // Ruckig sliders
    private JSlider slRVMax, slRAMax, slRJMax;
    private JLabel  lbRVMax, lbRAMax, lbRJMax;

    // Target sliders
    private JSlider slTargetA, slTargetB;
    private JLabel  lbTargetA, lbTargetB;

    private JButton btnGoA, btnGoB;
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
        chart = new XYChartBuilder()
                .width(900).height(600)
                .title("Trajectory")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Value")
                .build();
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setLegendPosition(
                org.knowm.xchart.style.Styler.LegendPosition.InsideNW);
        chart.addSeries("Position (units)",      new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE);
        chart.addSeries("Velocity (units/s)",    new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE);
        chart.addSeries("Acceleration (units/s\u00b2)", new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE);
        chartPanel = new XChartPanel<>(chart);
        add(chartPanel, BorderLayout.CENTER);
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
            typeCombo.setRenderer(new DefaultListCellRenderer() {
                @Override
                public Component getListCellRendererComponent(
                        JList<?> list, Object value, int index, boolean sel, boolean focus) {
                    JLabel lbl = (JLabel) super.getListCellRendererComponent(list, value, index, sel, focus);
                    if (value == TrajectoryType.RUCKIG) {
                        lbl.setEnabled(false);
                        lbl.setToolTipText("Ruckig JNI library not built. Run: gradlew :ControlLab:buildRuckigDesktop");
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

        sidebar.add(boldLabel("Targets"));
        sidebar.add(Box.createVerticalStrut(4));

        slTargetA = new JSlider(0, 1000, l2s(-100, -200, 200));
        lbTargetA = new JLabel();
        slTargetB = new JSlider(0, 1000, l2s(100,  -200, 200));
        lbTargetB = new JLabel();

        slTargetA.putClientProperty("min", -200.0);
        slTargetA.putClientProperty("max",  200.0);
        slTargetB.putClientProperty("min", -200.0);
        slTargetB.putClientProperty("max",  200.0);

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
        btnGoA.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnGoB.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));

        sidebar.add(btnGoA);
        sidebar.add(Box.createVerticalStrut(6));
        sidebar.add(btnGoB);

        typeCombo.addActionListener(e -> onTypeChanged());
        btnGoA.addActionListener(e -> onGoTo(slTargetA));
        btnGoB.addActionListener(e -> onGoTo(slTargetB));

        add(sidebar, BorderLayout.WEST);
    }

    private void buildPosSlidersIfNeeded() {
        if (slVMax != null) return;
        slVMax   = new JSlider(0, 1000, l2s(10,  0.1,  50));  lbVMax   = new JLabel();
        slAAccel = new JSlider(0, 1000, l2s(5,   0.1,  50));  lbAAccel = new JLabel();
        slADecel = new JSlider(0, 1000, l2s(5,   0.1,  50));  lbADecel = new JLabel();
        slJMax   = new JSlider(0, 1000, l2s(50,  1,   500));  lbJMax   = new JLabel();

        slVMax  .addChangeListener(e -> { updateLabel(lbVMax,   slVMax,   "vMax",   0.1,  50); stagePosParams(); });
        slAAccel.addChangeListener(e -> { updateLabel(lbAAccel, slAAccel, "aAccel", 0.1,  50); stagePosParams(); });
        slADecel.addChangeListener(e -> { updateLabel(lbADecel, slADecel, "aDecel", 0.1,  50); stagePosParams(); });
        slJMax  .addChangeListener(e -> { updateLabel(lbJMax,   slJMax,   "jMax",   1,   500); stagePosParams(); });
        updateLabel(lbVMax,   slVMax,   "vMax",   0.1,  50);
        updateLabel(lbAAccel, slAAccel, "aAccel", 0.1,  50);
        updateLabel(lbADecel, slADecel, "aDecel", 0.1,  50);
        updateLabel(lbJMax,   slJMax,   "jMax",   1,   500);
    }

    private void buildVelSlidersIfNeeded() {
        if (slAMax != null) return;
        slAMax = new JSlider(0, 1000, l2s(1197, 10,  5000));  lbAMax = new JLabel();
        slJInc = new JSlider(0, 1000, l2s(2669, 10, 10000));  lbJInc = new JLabel();
        slJDec = new JSlider(0, 1000, l2s(800,  10,  5000));  lbJDec = new JLabel();

        slAMax.addChangeListener(e -> { updateLabel(lbAMax, slAMax, "aMax", 10,  5000); stageVelParams(); });
        slJInc.addChangeListener(e -> { updateLabel(lbJInc, slJInc, "jInc", 10, 10000); stageVelParams(); });
        slJDec.addChangeListener(e -> { updateLabel(lbJDec, slJDec, "jDec", 10,  5000); stageVelParams(); });
        updateLabel(lbAMax, slAMax, "aMax", 10,  5000);
        updateLabel(lbJInc, slJInc, "jInc", 10, 10000);
        updateLabel(lbJDec, slJDec, "jDec", 10,  5000);
    }

    private void buildRuckigSlidersIfNeeded() {
        if (slRVMax != null) return;
        slRVMax = new JSlider(0, 1000, l2s(10,  0.1,  50));  lbRVMax = new JLabel();
        slRAMax = new JSlider(0, 1000, l2s(5,   0.1,  50));  lbRAMax = new JLabel();
        slRJMax = new JSlider(0, 1000, l2s(50,  1,   500));  lbRJMax = new JLabel();

        slRVMax.addChangeListener(e -> { updateLabel(lbRVMax, slRVMax, "vMax", 0.1,  50); stageRuckigParams(); });
        slRAMax.addChangeListener(e -> { updateLabel(lbRAMax, slRAMax, "aMax", 0.1,  50); stageRuckigParams(); });
        slRJMax.addChangeListener(e -> { updateLabel(lbRJMax, slRJMax, "jMax", 1,   500); stageRuckigParams(); });
        updateLabel(lbRVMax, slRVMax, "vMax", 0.1,  50);
        updateLabel(lbRAMax, slRAMax, "aMax", 0.1,  50);
        updateLabel(lbRJMax, slRJMax, "jMax", 1,   500);
    }

    private void stagePosParams() {
        engine.setPositionParams(
                s2l(slVMax.getValue(),   0.1,  50),
                s2l(slAAccel.getValue(), 0.1,  50),
                s2l(slADecel.getValue(), 0.1,  50),
                s2l(slJMax.getValue(),   1,   500));
    }

    private void stageVelParams() {
        engine.setVelocityParams(
                s2l(slAMax.getValue(), 10,  5000),
                s2l(slJInc.getValue(), 10, 10000),
                s2l(slJDec.getValue(), 10,  5000));
    }

    private void stageRuckigParams() {
        engine.setRuckigParams(
                s2l(slRVMax.getValue(), 0.1,  50),
                s2l(slRAMax.getValue(), 0.1,  50),
                s2l(slRJMax.getValue(), 1,   500));
    }

    private void onTypeChanged() {
        TrajectoryType sel = (TrajectoryType) typeCombo.getSelectedItem();

        if (sel == TrajectoryType.RUCKIG && !TrajectoryEngine.isRuckigAvailable()) {
            typeCombo.setSelectedItem(engine.getType());
            JOptionPane.showMessageDialog(this,
                    "Ruckig JNI library not built.\nRun: gradlew :ControlLab:buildRuckigDesktop",
                    "Ruckig Unavailable", JOptionPane.WARNING_MESSAGE);
            return;
        }

        engine.switchType(sel);
        buffer.clear();
        elapsedSec = 0.0;

        limitPanel.removeAll();
        switch (sel) {
            case SCURVE_POSITION:
                buildPosSlidersIfNeeded();
                addSliderRow(limitPanel, lbVMax,   slVMax);
                addSliderRow(limitPanel, lbAAccel, slAAccel);
                addSliderRow(limitPanel, lbADecel, slADecel);
                addSliderRow(limitPanel, lbJMax,   slJMax);
                updateTargetRange(-200, 200, -100, 100);
                break;
            case SCURVE_VELOCITY:
                buildVelSlidersIfNeeded();
                addSliderRow(limitPanel, lbAMax, slAMax);
                addSliderRow(limitPanel, lbJInc, slJInc);
                addSliderRow(limitPanel, lbJDec, slJDec);
                updateTargetRange(0, 6000, 1000, 3000);
                break;
            case RUCKIG:
                buildRuckigSlidersIfNeeded();
                addSliderRow(limitPanel, lbRVMax, slRVMax);
                addSliderRow(limitPanel, lbRAMax, slRAMax);
                addSliderRow(limitPanel, lbRJMax, slRJMax);
                updateTargetRange(-200, 200, -100, 100);
                break;
        }
        limitPanel.revalidate();
        limitPanel.repaint();

        chart.getSeriesMap().get("Position (units)").setEnabled(engine.hasPosition());
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
        double max = (maxObj != null) ? (double) maxObj :  200.0;
        return s2l(sl.getValue(), min, max);
    }

    private void onGoTo(JSlider targetSlider) {
        engine.applyParamsAndGoTo(targetSliderValue(targetSlider));
    }

    private void buildTimer() {
        simTimer = new Timer(TIMER_MS, e -> onTick());
        simTimer.start();
    }

    private void onTick() {
        if (!engine.isMoving()) return;
        engine.tick();
        elapsedSec += TrajectoryEngine.CYCLE_S;
        buffer.add(elapsedSec, engine.getPosition(), engine.getVelocity(), engine.getAcceleration());
        List<Double> times = buffer.getTimes();
        if (times.size() < 2) return;
        chart.updateXYSeries("Position (units)",        times, buffer.getPositions(),     null);
        chart.updateXYSeries("Velocity (units/s)",      times, buffer.getVelocities(),    null);
        chart.updateXYSeries("Acceleration (units/s\u00b2)", times, buffer.getAccelerations(), null);
        chartPanel.repaint();
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
        double max = (maxObj != null) ? (double) maxObj :  200.0;
        lbl.setText(String.format("Target %s: %.1f", name, s2l(sl.getValue(), min, max)));
    }

    /** Release native resources held by the engine. Call on application shutdown. */
    public void disposeEngine() {
        simTimer.stop();
        engine.dispose();
    }
}
