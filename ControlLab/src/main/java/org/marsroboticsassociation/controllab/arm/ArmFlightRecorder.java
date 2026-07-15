package org.marsroboticsassociation.controllab.arm;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;

/**
 * Per-tick CSV log of an {@link ArmEngine} session, so an anomaly observed live in the GUI can be
 * reconstructed offline afterward. One file per recorder, one row per control tick, plus an
 * {@code event} column carrying session context (target changes, gain edits, plant swaps,
 * controller/profiler changes) on the rows where it changed.
 *
 * <p>Positions are logged in degrees (matching the GUI); velocities and accelerations in rad/s
 * and rad/s². I/O failures disable the recorder rather than disturb the simulation.
 */
final class ArmFlightRecorder {

    private static final String HEADER =
            "t,dt,target_deg,traj_pos_deg,traj_vel,traj_acc,est_pos_deg,est_vel,"
                    + "true_pos_deg,true_vel,motor_pos_deg,engaged,power,event";
    private static final int FLUSH_EVERY = 16; // ~0.25 s of lag when tailing a live session

    private final Path file;
    private BufferedWriter writer;
    private String pendingEvent = "";
    private int rowsSinceFlush = 0;
    private boolean failed = false;

    /** Create a recorder writing to {@code dir}, named by session timestamp. */
    static ArmFlightRecorder createIn(Path dir) throws IOException {
        Files.createDirectories(dir);
        String name = "armlab-"
                + LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"))
                + ".csv";
        ArmFlightRecorder recorder = new ArmFlightRecorder(dir.resolve(name));
        // The GUI has no clean-shutdown path; make sure the tail of the session hits disk.
        Runtime.getRuntime().addShutdownHook(new Thread(recorder::flush, "armlab-log-flush"));
        return recorder;
    }

    /** Push buffered rows to disk (shutdown hook; also handy before reading a live session). */
    synchronized void flush() {
        if (writer != null && !failed) {
            try {
                writer.flush();
                rowsSinceFlush = 0;
            } catch (IOException ignored) {
                // flushing a lab log; nothing sensible to do
            }
        }
    }

    private ArmFlightRecorder(Path file) throws IOException {
        this.file = file;
        this.writer = Files.newBufferedWriter(file, StandardCharsets.UTF_8);
        writer.write(HEADER);
        writer.newLine();
        writer.flush();
    }

    Path getFile() {
        return file;
    }

    /**
     * Note a session event (target change, gain edit, plant swap...). Attached to the next tick
     * row; multiple events between ticks are joined with {@code ;}.
     */
    void event(String description) {
        pendingEvent = pendingEvent.isEmpty() ? description : pendingEvent + "; " + description;
    }

    /** Write one control-tick row. Never throws; a failed write disables the recorder. */
    void tick(double t, double dt, double targetRad,
              double trajPosRad, double trajVel, double trajAcc,
              double estPosRad, double estVel,
              double truePosRad, double trueVel, double motorPosRad,
              boolean engaged, double power) {
        if (failed) {
            return;
        }
        try {
            writer.write(String.format(Locale.US,
                    "%.4f,%.4f,%.3f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%s",
                    t, dt, Math.toDegrees(targetRad),
                    Math.toDegrees(trajPosRad), trajVel, trajAcc,
                    Math.toDegrees(estPosRad), estVel,
                    Math.toDegrees(truePosRad), trueVel, Math.toDegrees(motorPosRad),
                    engaged ? 1 : 0, power, csvSafe(pendingEvent)));
            writer.newLine();
            pendingEvent = "";
            if (++rowsSinceFlush >= FLUSH_EVERY) {
                writer.flush();
                rowsSinceFlush = 0;
            }
        } catch (IOException e) {
            failed = true;
            System.err.println("ArmFlightRecorder disabled (" + file + "): " + e);
        }
    }

    void close() {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException ignored) {
                // closing a lab log; nothing sensible to do
            }
            writer = null;
            failed = true;
        }
    }

    private static String csvSafe(String s) {
        if (s.isEmpty()) {
            return s;
        }
        // Events may contain commas (gain lists); quote the field and escape quotes.
        return '"' + s.replace("\"", "\"\"") + '"';
    }
}
