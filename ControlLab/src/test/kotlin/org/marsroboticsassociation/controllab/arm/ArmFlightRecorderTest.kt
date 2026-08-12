package org.marsroboticsassociation.controllab.arm

import java.nio.file.Files
import java.nio.file.Path
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import org.junit.jupiter.api.parallel.Execution
import org.junit.jupiter.api.parallel.ExecutionMode

@Execution(ExecutionMode.SAME_THREAD)
class ArmFlightRecorderTest {

    @TempDir lateinit var tempDir: Path

    @Test
    fun recordsTicksAndSessionEvents() {
        val engine = ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L)
        val log = engine.startFlightRecorder(tempDir)
        assertNotNull(log, "recorder starts")

        engine.setTargetRad(Math.toRadians(90.0))
        for (i in 0 until 200) {
            engine.tick()
        }
        engine.setPlantKind(ArmEngine.PlantKind.FLEX)
        for (i in 0 until 20) {
            engine.tick()
        }

        val lines = Files.readAllLines(log)
        // The recorder buffers up to FLUSH_EVERY rows; everything older must be on disk.
        assertTrue(lines.size >= 200, "rows on disk (buffered tail allowed): " + lines.size)
        assertTrue(lines[0].startsWith("t,dt,target_deg,traj_pos_deg"), "header row")

        val all = lines.joinToString("\n")
        assertTrue(all.contains("session start"), "session-start event")
        assertTrue(all.contains("target=90.00deg"), "target event")
        assertTrue(all.contains("plant="), "plant swap event")

        // Rows parse: every data row has the full column count, and the profile actually moved.
        // Keep trailing empty fields (Java String.split limit -1) for accurate column counts.
        val columns = lines[0].split(Regex(","), limit = Int.MAX_VALUE).size
        val lastData = lines[150].split(Regex(","), limit = Int.MAX_VALUE)
        assertEquals(columns, lastData.size, "column count on data rows")
        val trajPosDeg = lastData[3].toDouble()
        assertTrue(
            trajPosDeg < 226 && trajPosDeg > 80,
            "profile position logged plausibly: $trajPosDeg",
        )
    }
}
