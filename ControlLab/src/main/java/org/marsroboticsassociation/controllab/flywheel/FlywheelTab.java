package org.marsroboticsassociation.controllab.flywheel;

import org.knowm.xchart.*;
import org.knowm.xchart.style.markers.SeriesMarkers;
import org.marsroboticsassociation.controllab.trajectory.RollingBuffer;
import org.marsroboticsassociation.controllib.control.FlywheelSimple;

import javax.swing.*;
import java.awt.*;
import java.util.List;

public class FlywheelTab extends JPanel {

    private static final int SIDEBAR_WIDTH = 260;
    private static final int TIMER_MS = 20;
    private static final int BUFFER_POINTS = 500;
    private static final double WINDOW_SECS = 5.0;

    private FlywheelEngine engine;
    private final RollingBuffer buffer = new RollingBuffer(WINDOW_SECS, BUFFER_POINTS, 5);

    private XYChart chart;
    private XChartPanel<XYChart> chartPanel;

    private JComboBox<FlywheelControllerType> typeCombo;
    private JPanel paramPanel, plantPanel;
    private JPanel simplePanel, pfPanel, ssPanel;

    // Editable fields
    private EditableParamField efKS, efKV, efKA, efKP, efCutoff;
    private EditableParamField efPlantKS, efPlantKV, efPlantKA;
    private EditableParamField efTargetA, efTargetB;
    private EditableParamField efSimpleMaxAccel;
    private EditableParamField efPFAccelMax, efPFRisingJerk, efPFFallingJerk;
    private EditableParamField efSSModelStdDev, efSSMeasurementStdDev;

    private double targetA = 1000;
    private double targetB = 2000;

    private JButton btnGoA, btnGoB, btnCoast;
    private Timer simTimer;

    public FlywheelTab() {
        setLayout(new BorderLayout());
        engine = new FlywheelEngine(FlywheelControllerType.FLYWHEEL_SIMPLE);
        buildChart();
        buildSidebar();
        buildTimer();
    }

    private void buildChart() {
        chart = new XYChartBuilder()
                .width(900)
                .height(600)
                .title("Flywheel Simulation")
                .xAxisTitle("Time (s)")
                .yAxisTitle("Velocity (TPS)")
                .build();
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setLegendPosition(org.knowm.xchart.style.Styler.LegendPosition.InsideNW);
        
        chart.addSeries("True Velocity", new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE).setEnabled(false);
        chart.addSeries("Measured Velocity", new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE);
        chart.addSeries("Filtered Velocity", new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE);
        chart.addSeries("Profiled Velocity", new double[]{0}, new double[]{0}).setMarker(SeriesMarkers.NONE);
        chart.addSeries("Target", new double[]{0}, new double[]{0})
                .setMarker(SeriesMarkers.NONE)
                .setLineStyle(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, new float[]{6.0f, 4.0f}, 0.0f));

        chartPanel = new XChartPanel<>(chart);
        add(chartPanel, BorderLayout.CENTER);
    }

    private void buildSidebar() {
        JPanel sidebar = new JPanel();
        sidebar.setLayout(new BoxLayout(sidebar, BoxLayout.Y_AXIS));
        sidebar.setPreferredSize(new Dimension(SIDEBAR_WIDTH, 0));
        sidebar.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

        typeCombo = new JComboBox<>(FlywheelControllerType.values());
        typeCombo.setMaximumSize(new Dimension(Integer.MAX_VALUE, 28));
        typeCombo.addActionListener(e -> {
            engine.setType((FlywheelControllerType) typeCombo.getSelectedItem());
            updateControllerPanel();
            buffer.clear();
        });
        sidebar.add(typeCombo);
        sidebar.add(Box.createVerticalStrut(12));

        sidebar.add(boldLabel("Tuning"));
        sidebar.add(Box.createVerticalStrut(4));

        paramPanel = new JPanel();
        paramPanel.setLayout(new BoxLayout(paramPanel, BoxLayout.Y_AXIS));
        
        efKS = new EditableParamField("kS", engine.getKS(), "%.3f", 0, Double.POSITIVE_INFINITY, v -> updateParams());
        efKV = new EditableParamField("kV", engine.getKV(), "%.6f", 0, Double.POSITIVE_INFINITY, v -> updateParams());
        efKA = new EditableParamField("kA", engine.getKA(), "%.6f", 0, Double.POSITIVE_INFINITY, v -> updateParams());
        efKP = new EditableParamField("kP", engine.getKP(), "%.4f", 0, Double.POSITIVE_INFINITY, v -> updateParams());
        efCutoff = new EditableParamField("Cutoff (Hz)", engine.getVelLpfCutoffHz(), "%.1f", 0.1, 100.0, v -> updateParams());

        paramPanel.add(efKS);
        paramPanel.add(efKV);
        paramPanel.add(efKA);
        paramPanel.add(efKP);
        paramPanel.add(efCutoff);

        sidebar.add(paramPanel);
        sidebar.add(Box.createVerticalStrut(12));

        sidebar.add(boldLabel("Controller"));
        sidebar.add(Box.createVerticalStrut(4));

        buildControllerPanels();

        sidebar.add(simplePanel);
        sidebar.add(pfPanel);
        sidebar.add(ssPanel);
        updateControllerPanel();
        sidebar.add(Box.createVerticalStrut(12));

        JButton btnRevealPlant = new JButton("Show Simulator Plant");
        btnRevealPlant.setMaximumSize(new Dimension(Integer.MAX_VALUE, 24));
        sidebar.add(btnRevealPlant);
        sidebar.add(Box.createVerticalStrut(4));

        plantPanel = new JPanel();
        plantPanel.setLayout(new BoxLayout(plantPanel, BoxLayout.Y_AXIS));
        plantPanel.setVisible(false);

        efPlantKS = new EditableParamField("Plant kS", engine.getPlantKS(), "%.3f", 0, 12.0, v -> updatePlantParams());
        efPlantKV = new EditableParamField("Plant kV", engine.getPlantKV(), "%.6f", 0, 1.0, v -> updatePlantParams());
        efPlantKA = new EditableParamField("Plant kA", engine.getPlantKA(), "%.6f", 1e-9, 1.0, v -> updatePlantParams());
        plantPanel.add(efPlantKS);
        plantPanel.add(efPlantKV);
        plantPanel.add(efPlantKA);
        
        sidebar.add(plantPanel);

        btnRevealPlant.addActionListener(e -> {
            boolean visible = !plantPanel.isVisible();
            plantPanel.setVisible(visible);
            chart.getSeriesMap().get("True Velocity").setEnabled(visible);
            btnRevealPlant.setText(visible ? "Hide Simulator Plant" : "Show Simulator Plant");
            sidebar.revalidate();
        });

        sidebar.add(Box.createVerticalStrut(12));

        sidebar.add(boldLabel("Targets"));
        sidebar.add(Box.createVerticalStrut(4));

        efTargetA = new EditableParamField("Target A", targetA, "%.0f", 0, 5000, v -> targetA = v);
        efTargetB = new EditableParamField("Target B", targetB, "%.0f", 0, 5000, v -> targetB = v);

        sidebar.add(efTargetA);
        sidebar.add(efTargetB);
        sidebar.add(Box.createVerticalStrut(12));

        btnGoA = new JButton("\u2192 Go to A");
        btnGoB = new JButton("\u2192 Go to B");
        btnCoast = new JButton("\u2741 Coast");
        btnGoA.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnGoB.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
        btnCoast.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));

        btnGoA.addActionListener(e -> engine.setTarget(targetA));
        btnGoB.addActionListener(e -> engine.setTarget(targetB));
        btnCoast.addActionListener(e -> engine.setTarget(0));

        sidebar.add(btnGoA);
        sidebar.add(Box.createVerticalStrut(6));
        sidebar.add(btnGoB);
        sidebar.add(Box.createVerticalStrut(6));
        sidebar.add(btnCoast);

        add(sidebar, BorderLayout.WEST);
    }

    private void updateParams() {
        engine.setParams(
                efKV.getValue(),
                efKA.getValue(),
                efKS.getValue(),
                efKP.getValue(),
                efCutoff.getValue()
        );
    }

    private void updatePlantParams() {
        engine.setPlantParams(
                efPlantKV.getValue(),
                efPlantKA.getValue(),
                efPlantKS.getValue()
        );
    }

    private void buildTimer() {
        simTimer = new Timer(TIMER_MS, e -> onTick());
        simTimer.start();
    }

    private void buildControllerPanels() {
        double initialMaxAccel = FlywheelSimple.PARAMS.maxAccel;

        efSimpleMaxAccel = new EditableParamField("Max Accel (TPS\u00B2)", initialMaxAccel, "%.0f", 0, initialMaxAccel, v -> {
            engine.setSimpleParams(v);
        });

        simplePanel = new JPanel();
        simplePanel.setLayout(new BoxLayout(simplePanel, BoxLayout.Y_AXIS));
        simplePanel.add(efSimpleMaxAccel);

        efPFAccelMax = new EditableParamField("Accel Max (TPS\u00B2)", engine.getPFBAccelMax(), "%.0f", 0, 5000, v -> {
            engine.setPFParams(v, engine.getPFJerkIncreasing(), engine.getPFJerkDecreasing());
        });
        efPFRisingJerk = new EditableParamField("Rising Jerk (TPS\u00B3)", engine.getPFJerkIncreasing(), "%.0f", 0, 10000, v -> {
            engine.setPFParams(engine.getPFBAccelMax(), v, engine.getPFJerkDecreasing());
        });
        efPFFallingJerk = new EditableParamField("Falling Jerk (TPS\u00B3)", engine.getPFJerkDecreasing(), "%.0f", 0, 10000, v -> {
            engine.setPFParams(engine.getPFBAccelMax(), engine.getPFJerkIncreasing(), v);
        });

        pfPanel = new JPanel();
        pfPanel.setLayout(new BoxLayout(pfPanel, BoxLayout.Y_AXIS));
        pfPanel.add(efPFAccelMax);
        pfPanel.add(efPFRisingJerk);
        pfPanel.add(efPFFallingJerk);

        efSSModelStdDev = new EditableParamField("Model StdDev (rad/s)", engine.getSSModelStdDev(), "%.4f", 0.001, 100.0, v -> {
            engine.setSSParams(v, engine.getSSMeasurementStdDev());
        });
        efSSMeasurementStdDev = new EditableParamField("Meas StdDev (rad/s)", engine.getSSMeasurementStdDev(), "%.4f", 0.001, 100.0, v -> {
            engine.setSSParams(engine.getSSModelStdDev(), v);
        });

        ssPanel = new JPanel();
        ssPanel.setLayout(new BoxLayout(ssPanel, BoxLayout.Y_AXIS));
        ssPanel.add(efSSModelStdDev);
        ssPanel.add(efSSMeasurementStdDev);
    }

    private void updateControllerPanel() {
        FlywheelControllerType type = (FlywheelControllerType) typeCombo.getSelectedItem();
        simplePanel.setVisible(type == FlywheelControllerType.FLYWHEEL_SIMPLE);
        pfPanel.setVisible(type == FlywheelControllerType.VELOCITY_MOTOR_PF);
        ssPanel.setVisible(type == FlywheelControllerType.FLYWHEEL_STATE_SPACE);
        revalidate();
    }

    private void onTick() {
        engine.tick();
        recordCurrentSample();
    }

    private void recordCurrentSample() {
        buffer.add(
                engine.getElapsedSec(),
                engine.getTrueVelocity(),
                engine.getMeasuredVelocity(),
                engine.getFilteredVelocity(),
                engine.getProfiledVelocity(),
                engine.getTarget()
        );

        List<Double> times = buffer.getTimes();
        if (times.size() < 2) return;

        chart.updateXYSeries("True Velocity", times, buffer.getData(0), null);
        chart.updateXYSeries("Measured Velocity", times, buffer.getData(1), null);
        chart.updateXYSeries("Filtered Velocity", times, buffer.getData(2), null);
        chart.updateXYSeries("Profiled Velocity", times, buffer.getData(3), null);
        chart.updateXYSeries("Target", times, buffer.getData(4), null);
        chartPanel.repaint();
    }

    private static JLabel boldLabel(String text) {
        JLabel l = new JLabel(text);
        l.setFont(l.getFont().deriveFont(Font.BOLD));
        return l;
    }

    public void dispose() {
        simTimer.stop();
    }
}
