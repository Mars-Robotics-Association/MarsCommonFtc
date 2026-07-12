package org.marsroboticsassociation.controllab.arm;

import org.knowm.xchart.XChartPanel;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.Styler;
import org.knowm.xchart.style.markers.SeriesMarkers;
import org.marsroboticsassociation.controllab.flywheel.EditableParamField;
import org.marsroboticsassociation.controllab.trajectory.RollingBuffer;

import javax.swing.*;
import java.awt.*;
import java.util.List;
import java.util.concurrent.ExecutionException;

/**
 * The Arm tab: an animated {@link ArmCanvas}, a flywheel-style XChart time series, a param sidebar of
 * {@link EditableParamField}s, and a backlash-focused metrics readout, all ticked by a Swing timer at
 * roughly real time. Mirrors the structure of {@code FlywheelTab}.
 */
public class ArmTab extends JPanel {

    private static final int SIDEBAR_WIDTH = 300;
    private static final int TIMER_MS = 16;
    private static final int BUFFER_POINTS = 600;
    private static final double WINDOW_SECS = 6.0;
    private static final int NUM_STREAMS = 8;

    private final ArmEngine engine = new ArmEngine(ArmControllerType.ARM_PD);
    private final RollingBuffer buffer = new RollingBuffer(WINDOW_SECS, BUFFER_POINTS, NUM_STREAMS);

    private XYChart chart;
    private XChartPanel<XYChart> chartPanel;
    private ArmCanvas canvas;
    private JLabel metricsLabel;

    private JComboBox<ArmControllerType> typeCombo;
    private JCheckBox backlashCheck;
    private JSlider targetSlider;
    private JLabel targetLabel;
    private boolean updatingSlider = false;

    // Controller panels
    private JPanel pdPanel, lqrPanel, mechPanel, ffPanel;
    private EditableParamField efKP, efKD;
    private EditableParamField efQPos, efQVel, efR;
    private EditableParamField efMkP, efMkI, efMkD, efMkS, efMkV, efMkA, efMkCos, efMkSin;
    private EditableParamField efMvMax, efMaMax, efMjMax;
    private EditableParamField efFfKs, efFfKg, efFfKv, efFfKa;

    // Plant panel
    private EditableParamField efPlantKs, efPlantKg, efPlantKv, efPlantKa;
    private EditableParamField efBacklash, efContactK, efContactC, efLoadVisc, efLoadStat, efDisturb;
    private JComboBox<ArmPlantConfig.EncoderKind> encoderCombo;

    private JButton sysIdBtn;
    private Timer simTimer;

    public ArmTab() {
        setLayout(new BorderLayout());
        buildChart();
        buildCenter();
        buildSidebar();
        buildTimer();
        syncTargetUi(engine.getTargetRad());
        updateControllerPanel();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Chart + canvas
    // ─────────────────────────────────────────────────────────────────────────────

    private void buildChart() {
        chart = new XYChartBuilder()
                .width(700).height(600)
                .title("Arm Simulation")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Angle (deg)")
                .build();
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setLegendPosition(Styler.LegendPosition.InsideNW);
        chart.setYAxisGroupTitle(1, "Velocity (deg/s)");

        addLine("Target", true);
        addLine("Profile pos", false);
        addLine("Measured enc", false);
        addLine("True load", false);
        addLine("Motor side", false);

        addVelLine("Load vel");
        addVelLine("Profile vel");
        addVelLine("Est vel");

        chartPanel = new XChartPanel<>(chart);
    }

    private void addLine(String name, boolean dashed) {
        XYSeries s = chart.addSeries(name, new double[]{0}, new double[]{0});
        s.setMarker(SeriesMarkers.NONE);
        if (dashed) {
            s.setLineStyle(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                    10f, new float[]{6f, 4f}, 0f));
        }
    }

    /** A velocity-axis (group 1) line series. */
    private void addVelLine(String name) {
        XYSeries s = chart.addSeries(name, new double[]{0}, new double[]{0});
        s.setMarker(SeriesMarkers.NONE);
        s.setYAxisGroup(1);
    }

    private void buildCenter() {
        canvas = new ArmCanvas(engine, rad -> applyTargetRad(rad, false));

        metricsLabel = new JLabel();
        metricsLabel.setBorder(BorderFactory.createEmptyBorder(6, 10, 6, 10));
        metricsLabel.setVerticalAlignment(SwingConstants.TOP);

        JPanel center = new JPanel(new BorderLayout());
        center.add(canvas, BorderLayout.WEST);
        center.add(chartPanel, BorderLayout.CENTER);
        center.add(metricsLabel, BorderLayout.SOUTH);
        add(center, BorderLayout.CENTER);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Sidebar
    // ─────────────────────────────────────────────────────────────────────────────

    private void buildSidebar() {
        JPanel sidebar = new JPanel();
        sidebar.setLayout(new BoxLayout(sidebar, BoxLayout.Y_AXIS));
        sidebar.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

        typeCombo = new JComboBox<>(ArmControllerType.values());
        typeCombo.setMaximumSize(new Dimension(Integer.MAX_VALUE, 28));
        typeCombo.addActionListener(e -> {
            engine.setControllerType((ArmControllerType) typeCombo.getSelectedItem());
            updateControllerPanel();
            buffer.clear();
        });
        sidebar.add(typeCombo);
        sidebar.add(Box.createVerticalStrut(8));

        backlashCheck = new JCheckBox("Backlash plant", engine.isBacklashEnabled());
        backlashCheck.setAlignmentX(LEFT_ALIGNMENT);
        backlashCheck.addActionListener(e -> engine.setBacklashEnabled(backlashCheck.isSelected()));
        sidebar.add(backlashCheck);
        sidebar.add(Box.createVerticalStrut(8));

        // Target slider (degrees).
        sidebar.add(boldLabel("Target"));
        targetLabel = new JLabel();
        sidebar.add(targetLabel);
        int minDeg = (int) Math.floor(Math.toDegrees(engine.getMinAngleRad()));
        int maxDeg = (int) Math.ceil(Math.toDegrees(engine.getMaxAngleRad()));
        targetSlider = new JSlider(minDeg, maxDeg, (int) Math.round(Math.toDegrees(engine.getTargetRad())));
        targetSlider.setMaximumSize(new Dimension(Integer.MAX_VALUE, 40));
        targetSlider.addChangeListener(e -> {
            if (updatingSlider) return;
            applyTargetRad(Math.toRadians(targetSlider.getValue()), true);
        });
        sidebar.add(targetSlider);
        sidebar.add(Box.createVerticalStrut(8));

        JButton resetBtn = new JButton("Reset (park)");
        resetBtn.setMaximumSize(new Dimension(Integer.MAX_VALUE, 30));
        resetBtn.addActionListener(e -> {
            engine.reset();
            buffer.clear();
            syncTargetUi(engine.getTargetRad());
        });
        sidebar.add(resetBtn);
        sidebar.add(Box.createVerticalStrut(6));

        sysIdBtn = new JButton("Run SysID → model");
        sysIdBtn.setMaximumSize(new Dimension(Integer.MAX_VALUE, 30));
        sysIdBtn.addActionListener(e -> onRunSysId());
        sidebar.add(sysIdBtn);
        sidebar.add(Box.createVerticalStrut(10));

        sidebar.add(boldLabel("Controller gains"));
        buildControllerPanels();
        sidebar.add(pdPanel);
        sidebar.add(lqrPanel);
        sidebar.add(mechPanel);
        sidebar.add(Box.createVerticalStrut(6));
        sidebar.add(ffPanel);
        sidebar.add(Box.createVerticalStrut(10));

        sidebar.add(boldLabel("Plant (the real robot)"));
        buildPlantPanel(sidebar);

        JScrollPane scroll = new JScrollPane(sidebar,
                JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        scroll.setPreferredSize(new Dimension(SIDEBAR_WIDTH, 0));
        scroll.getVerticalScrollBar().setUnitIncrement(16);
        add(scroll, BorderLayout.WEST);
    }

    private void buildControllerPanels() {
        // PD
        efKP = new EditableParamField("kP", engine.getKP(), "%.2f", 0, 500, v -> updatePd());
        efKD = new EditableParamField("kD", engine.getKD(), "%.3f", 0, 100, v -> updatePd());
        pdPanel = vpanel(efKP, efKD);

        // LQR
        efQPos = new EditableParamField("Q pos", engine.getLqrQPos(), "%.3f", 1e-3, 100, v -> updateLqr());
        efQVel = new EditableParamField("Q vel", engine.getLqrQVel(), "%.3f", 1e-3, 100, v -> updateLqr());
        efR = new EditableParamField("R volt", engine.getLqrR(), "%.2f", 1e-3, 100, v -> updateLqr());
        lqrPanel = vpanel(efQPos, efQVel, efR);

        // Mechanism (Lineage B) — includes model FF (kS/kV/kA/kCos/kSin) so SysID results are visible.
        MechanismArmAdapter.Gains g = engine.getMechGains();
        efMkP = new EditableParamField("kP", g.kP, "%.2f", 0, 500, v -> updateMech());
        efMkI = new EditableParamField("kI", g.kI, "%.2f", 0, 500, v -> updateMech());
        efMkD = new EditableParamField("kD", g.kD, "%.3f", 0, 100, v -> updateMech());
        efMkS = new EditableParamField("kS", g.kS, "%.3f", 0, 12, v -> updateMech());
        efMkV = new EditableParamField("kV", g.kV, "%.3f", 0, 12, v -> updateMech());
        efMkA = new EditableParamField("kA", g.kA, "%.4f", 1e-4, 12, v -> updateMech());
        efMkCos = new EditableParamField("kCos", g.kCos, "%.3f", 0, 20, v -> updateMech());
        efMkSin = new EditableParamField("kSin", g.kSin, "%.3f", -20, 20, v -> updateMech());
        efMvMax = new EditableParamField("vMax", g.maxVel, "%.2f", 0.1, 50, v -> updateMech());
        efMaMax = new EditableParamField("aMax", g.maxAccel, "%.2f", 0.1, 200, v -> updateMech());
        efMjMax = new EditableParamField("jMax", g.maxJerk, "%.1f", 1, 5000, v -> updateMech());
        mechPanel = vpanel(efMkP, efMkI, efMkD, efMkS, efMkV, efMkA, efMkCos, efMkSin,
                efMvMax, efMaMax, efMjMax);

        // Shared feedforward (Lineage A only)
        efFfKs = new EditableParamField("ff kS", engine.getFfKs(), "%.3f", 0, 12, v -> updateFf());
        efFfKg = new EditableParamField("ff kG", engine.getFfKg(), "%.3f", 0, 12, v -> updateFf());
        efFfKv = new EditableParamField("ff kV", engine.getFfKv(), "%.3f", 0, 12, v -> updateFf());
        efFfKa = new EditableParamField("ff kA", engine.getFfKa(), "%.4f", 1e-4, 12, v -> updateFf());
        ffPanel = vpanel(boldLabel("Feedforward"), efFfKs, efFfKg, efFfKv, efFfKa);
    }

    private void buildPlantPanel(JPanel sidebar) {
        efPlantKs = new EditableParamField("kS", engine.getPlantKs(), "%.3f", 0, 12, v -> updatePlantDyn());
        efPlantKg = new EditableParamField("kG", engine.getPlantKg(), "%.3f", 0, 12, v -> updatePlantDyn());
        efPlantKv = new EditableParamField("kV", engine.getPlantKv(), "%.3f", 0, 12, v -> updatePlantDyn());
        efPlantKa = new EditableParamField("kA", engine.getPlantKa(), "%.4f", 1e-4, 12, v -> updatePlantDyn());
        efBacklash = new EditableParamField("Backlash (deg)", Math.toDegrees(engine.getBacklashRadCfg()),
                "%.2f", 0, 30, v -> engine.setBacklashRad(Math.toRadians(v)));
        efContactK = new EditableParamField("Contact k", engine.getContactStiffness(), "%.1f", 1, 5000,
                v -> engine.setContact(v, engine.getContactDamping()));
        efContactC = new EditableParamField("Contact c", engine.getContactDamping(), "%.2f", 0, 100,
                v -> engine.setContact(engine.getContactStiffness(), v));
        efLoadVisc = new EditableParamField("Load kV", engine.getLoadViscous(), "%.3f", 0, 12,
                v -> engine.setLoadFriction(v, engine.getLoadStatic()));
        efLoadStat = new EditableParamField("Load kS", engine.getLoadStatic(), "%.3f", 0, 12,
                v -> engine.setLoadFriction(engine.getLoadViscous(), v));
        efDisturb = new EditableParamField("Disturbance (V)", engine.getDisturbanceVoltage(), "%.2f", -12, 12,
                v -> engine.setDisturbanceVoltage(v));

        encoderCombo = new JComboBox<>(ArmPlantConfig.EncoderKind.values());
        encoderCombo.setSelectedItem(engine.getEncoderKind());
        encoderCombo.setMaximumSize(new Dimension(Integer.MAX_VALUE, 26));
        encoderCombo.addActionListener(e ->
                engine.setEncoderKind((ArmPlantConfig.EncoderKind) encoderCombo.getSelectedItem()));

        sidebar.add(efPlantKs);
        sidebar.add(efPlantKg);
        sidebar.add(efPlantKv);
        sidebar.add(efPlantKa);
        sidebar.add(efBacklash);
        sidebar.add(efContactK);
        sidebar.add(efContactC);
        sidebar.add(efLoadVisc);
        sidebar.add(efLoadStat);
        sidebar.add(efDisturb);
        sidebar.add(new JLabel("Encoder:"));
        sidebar.add(encoderCombo);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Sidebar callbacks
    // ─────────────────────────────────────────────────────────────────────────────

    private void updatePd()  { engine.setPdGains(efKP.getValue(), efKD.getValue()); }
    private void updateLqr() { engine.setLqrWeights(efQPos.getValue(), efQVel.getValue(), efR.getValue()); }
    private void updateFf()  { engine.setFeedforwardGains(efFfKs.getValue(), efFfKg.getValue(),
                                                          efFfKv.getValue(), efFfKa.getValue()); }
    private void updateMech() {
        engine.setMechanismGains(efMkP.getValue(), efMkI.getValue(), efMkD.getValue(),
                efMkS.getValue(), efMkV.getValue(), efMkA.getValue(),
                efMkCos.getValue(), efMkSin.getValue(),
                efMvMax.getValue(), efMaMax.getValue(), efMjMax.getValue());
    }
    private void updatePlantDyn() {
        engine.setPlantDynamics(efPlantKs.getValue(), efPlantKg.getValue(),
                efPlantKv.getValue(), efPlantKa.getValue());
    }

    /**
     * Run SysID off the EDT (the characterization is many simulated seconds of work) and present
     * the result on the EDT when done.
     */
    private void onRunSysId() {
        if (sysIdBtn != null) {
            sysIdBtn.setEnabled(false);
            sysIdBtn.setText("Running SysID…");
        }
        final boolean throughBacklash = engine.isBacklashEnabled();
        final double plantKs = engine.getPlantKs();
        final double plantKv = engine.getPlantKv();
        final double plantKa = engine.getPlantKa();
        final double plantKg = engine.getPlantKg();

        SwingWorker<ArmSysId.Result, Void> worker = new SwingWorker<ArmSysId.Result, Void>() {
            @Override
            protected ArmSysId.Result doInBackground() {
                return engine.runSysId();
            }

            @Override
            protected void done() {
                try {
                    ArmSysId.Result r = get();
                    presentSysIdResult(r, throughBacklash, plantKs, plantKv, plantKa, plantKg);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                } catch (ExecutionException ex) {
                    Throwable cause = ex.getCause() != null ? ex.getCause() : ex;
                    JOptionPane.showMessageDialog(ArmTab.this,
                            "SysID failed: " + cause.getMessage(),
                            "SysID Error", JOptionPane.ERROR_MESSAGE);
                } finally {
                    if (sysIdBtn != null) {
                        sysIdBtn.setEnabled(true);
                        sysIdBtn.setText("Run SysID → model");
                    }
                }
            }
        };
        worker.execute();
    }

    private void presentSysIdResult(ArmSysId.Result r, boolean throughBacklash,
                                    double plantKs, double plantKv, double plantKa, double plantKg) {
        String plant = throughBacklash
                ? "backlash plant (motor-side encoder)" : "rigid plant";
        String msg = String.format(
                "<html>Identified through the %s (R² = %.4f, %d samples):<br><br>"
                        + "<table cellpadding=3>"
                        + "<tr><th></th><th>recovered</th><th>true plant</th></tr>"
                        + "<tr><td>kS</td><td>%.3f</td><td>%.3f</td></tr>"
                        + "<tr><td>kV</td><td>%.3f</td><td>%.3f</td></tr>"
                        + "<tr><td>kA</td><td>%.3f</td><td>%.3f</td></tr>"
                        + "<tr><td>kCos (kG)</td><td>%.3f</td><td>%.3f</td></tr>"
                        + "<tr><td>kSin</td><td>%.3f</td><td>%.3f</td></tr>"
                        + "</table><br>Apply these to the controller model?</html>",
                plant, r.rSquared, r.samples,
                r.kS, plantKs,
                r.kV, plantKv,
                r.kA, plantKa,
                r.kCos, plantKg,
                r.kSin, 0.0);
        int choice = JOptionPane.showConfirmDialog(this, msg, "SysID Result",
                JOptionPane.YES_NO_OPTION, JOptionPane.PLAIN_MESSAGE);
        if (choice != JOptionPane.YES_OPTION) return;

        engine.applyIdentifiedGains(r);
        // Reflect the identified values in both Lineage A FF fields and mechanism model fields.
        efFfKs.setValue(r.kS, "%.3f");
        efFfKg.setValue(r.kCos, "%.3f");
        efFfKv.setValue(r.kV, "%.3f");
        efFfKa.setValue(r.kA, "%.4f");
        efMkS.setValue(r.kS, "%.3f");
        efMkV.setValue(r.kV, "%.3f");
        efMkA.setValue(r.kA, "%.4f");
        efMkCos.setValue(r.kCos, "%.3f");
        efMkSin.setValue(r.kSin, "%.3f");
        buffer.clear();
    }

    private void updateControllerPanel() {
        ArmControllerType t = (ArmControllerType) typeCombo.getSelectedItem();
        pdPanel.setVisible(t == ArmControllerType.ARM_PD);
        lqrPanel.setVisible(t == ArmControllerType.ARM_LQR);
        mechPanel.setVisible(t == ArmControllerType.MECHANISM_PIDF);
        ffPanel.setVisible(t == ArmControllerType.ARM_PD || t == ArmControllerType.ARM_LQR);
        revalidate();
        repaint();
    }

    private void applyTargetRad(double rad, boolean fromSlider) {
        engine.setTargetRad(rad);
        double deg = Math.toDegrees(engine.getTargetRad());
        if (!fromSlider) {
            updatingSlider = true;
            targetSlider.setValue((int) Math.round(deg));
            updatingSlider = false;
        }
        targetLabel.setText(String.format("%.1f°", deg));
    }

    private void syncTargetUi(double rad) {
        updatingSlider = true;
        targetSlider.setValue((int) Math.round(Math.toDegrees(rad)));
        updatingSlider = false;
        targetLabel.setText(String.format("%.1f°", Math.toDegrees(rad)));
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Timer loop
    // ─────────────────────────────────────────────────────────────────────────────

    private void buildTimer() {
        simTimer = new Timer(TIMER_MS, e -> onTick());
        simTimer.start();
    }

    private void onTick() {
        engine.tick();
        recordSample();
        canvas.repaint();
        updateMetricsLabel();
    }

    private void recordSample() {
        buffer.add(engine.getElapsedSec(),
                Math.toDegrees(engine.getTargetRad()),
                Math.toDegrees(engine.getTrajPosRad()),
                Math.toDegrees(engine.getMeasuredEncoderRad()),
                Math.toDegrees(engine.getTrueLoadRad()),
                Math.toDegrees(engine.getMotorRad()),
                Math.toDegrees(engine.getTrueLoadVelRad()),
                Math.toDegrees(engine.getTrajVelRad()),
                Math.toDegrees(engine.getEstimatedVelRad()));

        List<Double> times = buffer.getTimes();
        if (times.size() < 2) return;
        chart.updateXYSeries("Target", times, buffer.getData(0), null);
        chart.updateXYSeries("Profile pos", times, buffer.getData(1), null);
        chart.updateXYSeries("Measured enc", times, buffer.getData(2), null);
        chart.updateXYSeries("True load", times, buffer.getData(3), null);
        chart.updateXYSeries("Motor side", times, buffer.getData(4), null);
        chart.updateXYSeries("Load vel", times, buffer.getData(5), null);
        chart.updateXYSeries("Profile vel", times, buffer.getData(6), null);
        chart.updateXYSeries("Est vel", times, buffer.getData(7), null);
        chartPanel.repaint();
    }

    private void updateMetricsLabel() {
        ArmMetrics m = engine.getMetrics();
        String settle = Double.isNaN(m.settleTimeSec()) ? "—" : String.format("%.2f s", m.settleTimeSec());
        metricsLabel.setText(String.format(
                "<html><b>%s</b> &nbsp; | &nbsp; Engaged: %.0f%%<br>"
                        + "Target %.1f° &nbsp; Load %.1f° &nbsp; Est %.1f°<br>"
                        + "SS err %.1f° &nbsp; Overshoot %.1f° &nbsp; Settle %s<br>"
                        + "Lash gap %.1f° &nbsp; Lost motion (peak) %.1f°</html>",
                engine.getModeLabel(), m.pctEngaged(),
                Math.toDegrees(engine.getTargetRad()), Math.toDegrees(engine.getTrueLoadRad()),
                Math.toDegrees(engine.getEstimatedPosRad()),
                m.steadyStateErrorDeg(), m.overshootDeg(), settle,
                m.lashGapDeg(), m.lostMotionDeg()));
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────────────

    private static JPanel vpanel(JComponent... items) {
        JPanel p = new JPanel();
        p.setLayout(new BoxLayout(p, BoxLayout.Y_AXIS));
        p.setAlignmentX(LEFT_ALIGNMENT);
        for (JComponent c : items) {
            c.setAlignmentX(LEFT_ALIGNMENT);
            p.add(c);
        }
        return p;
    }

    private static JLabel boldLabel(String text) {
        JLabel l = new JLabel(text);
        l.setFont(l.getFont().deriveFont(Font.BOLD));
        l.setAlignmentX(LEFT_ALIGNMENT);
        return l;
    }

    public void dispose() {
        if (simTimer != null) simTimer.stop();
    }
}
