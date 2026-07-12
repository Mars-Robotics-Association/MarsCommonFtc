package org.marsroboticsassociation.controllab.arm;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Arc2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.function.DoubleConsumer;

/**
 * Animated 2D arm drawing. Paints, about a central pivot:
 * <ul>
 *   <li>the <b>load link</b> at {@link ArmEngine#getTrueLoadRad()} (the real arm),
 *   <li>the <b>motor-side link</b> at {@link ArmEngine#getMotorRad()} (thin, green when the teeth
 *       are engaged, red when separated) — so the backlash gap between them is visible,
 *   <li>a dashed <b>target ghost</b> at {@link ArmEngine#getTargetRad()},
 *   <li>the hard-stop arc between the min and max angles.
 * </ul>
 *
 * <p>Angles are radians from horizontal (positive above). Clicking or dragging on the canvas sets the
 * target to the pointer angle (clamped to the hard stops) via the supplied callback.
 */
public class ArmCanvas extends JPanel {

    private static final Color LOAD_COLOR    = new Color(0x2C, 0x7B, 0xE1);
    private static final Color ENGAGED_COLOR = new Color(0x2E, 0xA0, 0x43);
    private static final Color SEPARATED_COLOR = new Color(0xD0, 0x3A, 0x2E);
    private static final Color TARGET_COLOR  = new Color(0x88, 0x88, 0x88);
    private static final Color ARC_COLOR     = new Color(0xBB, 0xBB, 0xBB);

    private final ArmEngine engine;
    private final DoubleConsumer onTargetRad;

    public ArmCanvas(ArmEngine engine, DoubleConsumer onTargetRad) {
        this.engine = engine;
        this.onTargetRad = onTargetRad;
        setPreferredSize(new Dimension(420, 420));
        setBackground(Color.WHITE);

        MouseAdapter mouse = new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e)  { setTargetFromPoint(e); }
            @Override public void mouseDragged(MouseEvent e)  { setTargetFromPoint(e); }
        };
        addMouseListener(mouse);
        addMouseMotionListener(mouse);
    }

    private void setTargetFromPoint(MouseEvent e) {
        int px = getWidth() / 2;
        int py = getHeight() / 2;
        double dx = e.getX() - px;
        double dy = py - e.getY(); // invert: screen y grows downward
        // atan2 is in (−π, π]; unwrap into the plant range (which may extend past −π).
        double angle = unwrapIntoRange(Math.atan2(dy, dx),
                engine.getMinAngleRad(), engine.getMaxAngleRad());
        onTargetRad.accept(angle);
    }

    /**
     * Map a principal angle into {@code [min, max]} by adding/subtracting 2π, then clamp. Needed
     * when the hard-stop span goes past −π (e.g. Lineage A −224°…−45°).
     */
    static double unwrapIntoRange(double principalRad, double minRad, double maxRad) {
        double mid = 0.5 * (minRad + maxRad);
        double best = principalRad;
        double bestDist = Math.abs(principalRad - mid);
        for (int k = -1; k <= 1; k++) {
            if (k == 0) continue;
            double cand = principalRad + k * 2.0 * Math.PI;
            double dist = Math.abs(cand - mid);
            if (dist < bestDist) {
                bestDist = dist;
                best = cand;
            }
        }
        return Math.max(minRad, Math.min(maxRad, best));
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int px = getWidth() / 2;
        int py = getHeight() / 2;
        double armLen = Math.min(getWidth(), getHeight()) * 0.38;

        // Hard-stop arc (from min to max angle).
        double minA = engine.getMinAngleRad();
        double maxA = engine.getMaxAngleRad();
        g2.setColor(ARC_COLOR);
        g2.setStroke(new BasicStroke(2f));
        // Arc2D uses degrees CCW from +x with +y up (matches our angle convention).
        double startDeg = Math.toDegrees(minA);
        double extentDeg = Math.toDegrees(maxA - minA);
        double arcR = armLen * 1.06;
        g2.draw(new Arc2D.Double(px - arcR, py - arcR, 2 * arcR, 2 * arcR,
                startDeg, extentDeg, Arc2D.OPEN));
        drawStopTick(g2, px, py, minA, armLen);
        drawStopTick(g2, px, py, maxA, armLen);

        // Target ghost (dashed).
        g2.setColor(TARGET_COLOR);
        g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                10f, new float[]{8f, 6f}, 0f));
        drawLink(g2, px, py, engine.getTargetRad(), armLen);

        // Motor-side link (thin, colored by engagement).
        boolean engaged = engine.isEngaged();
        g2.setColor(engaged ? ENGAGED_COLOR : SEPARATED_COLOR);
        g2.setStroke(new BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        drawLink(g2, px, py, engine.getMotorRad(), armLen * 0.92);

        // Load link (thick).
        g2.setColor(LOAD_COLOR);
        g2.setStroke(new BasicStroke(8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        drawLink(g2, px, py, engine.getTrueLoadRad(), armLen);

        // Pivot.
        g2.setColor(Color.DARK_GRAY);
        g2.fill(new Ellipse2D.Double(px - 7, py - 7, 14, 14));

        // Load end knob.
        double[] end = endPoint(px, py, engine.getTrueLoadRad(), armLen);
        g2.setColor(LOAD_COLOR);
        g2.fill(new Ellipse2D.Double(end[0] - 9, end[1] - 9, 18, 18));

        drawLegend(g2, engaged);
    }

    private void drawLink(Graphics2D g2, int px, int py, double angleRad, double len) {
        double[] end = endPoint(px, py, angleRad, len);
        g2.draw(new Line2D.Double(px, py, end[0], end[1]));
    }

    private void drawStopTick(Graphics2D g2, int px, int py, double angleRad, double len) {
        double[] a = endPoint(px, py, angleRad, len * 1.0);
        double[] b = endPoint(px, py, angleRad, len * 1.12);
        g2.setStroke(new BasicStroke(2f));
        g2.setColor(ARC_COLOR);
        g2.draw(new Line2D.Double(a[0], a[1], b[0], b[1]));
    }

    private static double[] endPoint(int px, int py, double angleRad, double len) {
        double ex = px + len * Math.cos(angleRad);
        double ey = py - len * Math.sin(angleRad); // invert for screen coords
        return new double[]{ex, ey};
    }

    private void drawLegend(Graphics2D g2, boolean engaged) {
        int x = 12, y = 18, dy = 16;
        g2.setFont(g2.getFont().deriveFont(11f));
        g2.setColor(LOAD_COLOR);
        g2.drawString("━ load (true arm)", x, y);
        g2.setColor(engaged ? ENGAGED_COLOR : SEPARATED_COLOR);
        g2.drawString("━ motor side (" + (engaged ? "engaged" : "SEPARATED") + ")", x, y + dy);
        g2.setColor(TARGET_COLOR);
        g2.drawString("╌ target", x, y + 2 * dy);
    }
}
