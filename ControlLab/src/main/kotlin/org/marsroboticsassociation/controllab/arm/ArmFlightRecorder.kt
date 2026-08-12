package org.marsroboticsassociation.controllab.arm

import java.io.BufferedWriter
import java.io.IOException
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.Locale

/**
 * Per-tick CSV log of an [ArmEngine] session, so an anomaly observed live in the GUI can be
 * reconstructed offline afterward. One file per recorder, one row per control tick, plus an `event`
 * column carrying session context (target changes, gain edits, plant swaps, controller/profiler
 * changes) on the rows where it changed.
 *
 * Positions are logged in degrees (matching the GUI); velocities and accelerations in rad/s and
 * rad/s². I/O failures disable the recorder rather than disturb the simulation.
 */
internal class ArmFlightRecorder private constructor(private val file: Path) {
    private var writer: BufferedWriter? =
        Files.newBufferedWriter(file, StandardCharsets.UTF_8).also {
            it.write(HEADER)
            it.newLine()
            it.flush()
        }
    private var pendingEvent = ""
    private var rowsSinceFlush = 0
    private var failed = false

    fun getFile(): Path = file

    /**
     * Note a session event (target change, gain edit, plant swap...). Attached to the next tick
     * row; multiple events between ticks are joined with `;`.
     */
    fun event(description: String) {
        pendingEvent = if (pendingEvent.isEmpty()) description else "$pendingEvent; $description"
    }

    /** Write one control-tick row. Never throws; a failed write disables the recorder. */
    fun tick(
        t: Double,
        dt: Double,
        targetRad: Double,
        trajPosRad: Double,
        trajVel: Double,
        trajAcc: Double,
        estPosRad: Double,
        estVel: Double,
        truePosRad: Double,
        trueVel: Double,
        motorPosRad: Double,
        engaged: Boolean,
        power: Double,
    ) {
        if (failed) return
        try {
            val w = writer ?: return
            w.write(
                String.format(
                    Locale.US,
                    "%.4f,%.4f,%.3f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%s",
                    t,
                    dt,
                    Math.toDegrees(targetRad),
                    Math.toDegrees(trajPosRad),
                    trajVel,
                    trajAcc,
                    Math.toDegrees(estPosRad),
                    estVel,
                    Math.toDegrees(truePosRad),
                    trueVel,
                    Math.toDegrees(motorPosRad),
                    if (engaged) 1 else 0,
                    power,
                    csvSafe(pendingEvent),
                )
            )
            w.newLine()
            pendingEvent = ""
            if (++rowsSinceFlush >= FLUSH_EVERY) {
                w.flush()
                rowsSinceFlush = 0
            }
        } catch (e: IOException) {
            failed = true
            System.err.println("ArmFlightRecorder disabled ($file): $e")
        }
    }

    fun close() {
        val w = writer
        if (w != null) {
            try {
                w.flush()
                w.close()
            } catch (_: IOException) {
                // closing a lab log; nothing sensible to do
            }
            writer = null
            failed = true
        }
    }

    /** Push buffered rows to disk (shutdown hook; also handy before reading a live session). */
    @Synchronized
    fun flush() {
        val w = writer
        if (w != null && !failed) {
            try {
                w.flush()
                rowsSinceFlush = 0
            } catch (_: IOException) {
                // flushing a lab log; nothing sensible to do
            }
        }
    }

    companion object {
        private const val HEADER =
            "t,dt,target_deg,traj_pos_deg,traj_vel,traj_acc,est_pos_deg,est_vel," +
                "true_pos_deg,true_vel,motor_pos_deg,engaged,power,event"
        private const val FLUSH_EVERY = 16 // ~0.25 s of lag when tailing a live session

        /** Create a recorder writing to `dir`, named by session timestamp. */
        @JvmStatic
        @Throws(IOException::class)
        fun createIn(dir: Path): ArmFlightRecorder {
            Files.createDirectories(dir)
            val name =
                "armlab-" +
                    LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss")) +
                    ".csv"
            val recorder = ArmFlightRecorder(dir.resolve(name))
            // The GUI has no clean-shutdown path; make sure the tail of the session hits disk.
            Runtime.getRuntime().addShutdownHook(Thread(recorder::flush, "armlab-log-flush"))
            return recorder
        }

        private fun csvSafe(s: String): String {
            if (s.isEmpty()) return s
            // Events may contain commas (gain lists); quote the field and escape quotes.
            return "\"" + s.replace("\"", "\"\"") + "\""
        }
    }
}
