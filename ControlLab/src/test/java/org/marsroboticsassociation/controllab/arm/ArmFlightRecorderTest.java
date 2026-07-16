package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

@Execution(ExecutionMode.SAME_THREAD)
class ArmFlightRecorderTest {

    @TempDir
    Path tempDir;

    @Test
    void recordsTicksAndSessionEvents() throws Exception {
        ArmEngine engine = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L);
        Path log = engine.startFlightRecorder(tempDir);
        assertNotNull(log, "recorder starts");

        engine.setTargetRad(Math.toRadians(90));
        for (int i = 0; i < 200; i++) {
            engine.tick();
        }
        engine.setPlantKind(ArmEngine.PlantKind.FLEX);
        for (int i = 0; i < 20; i++) {
            engine.tick();
        }

        List<String> lines = Files.readAllLines(log);
        // The recorder buffers up to FLUSH_EVERY rows; everything older must be on disk.
        assertTrue(lines.size() >= 200, "rows on disk (buffered tail allowed): " + lines.size());
        assertTrue(lines.get(0).startsWith("t,dt,target_deg,traj_pos_deg"), "header row");

        String all = String.join("\n", lines);
        assertTrue(all.contains("session start"), "session-start event");
        assertTrue(all.contains("target=90.00deg"), "target event");
        assertTrue(all.contains("plant="), "plant swap event");

        // Rows parse: every data row has the full column count, and the profile actually moved.
        int columns = lines.get(0).split(",", -1).length;
        String[] lastData = lines.get(150).split(",", -1);
        assertEquals(columns, lastData.length, "column count on data rows");
        double trajPosDeg = Double.parseDouble(lastData[3]);
        assertTrue(trajPosDeg < 226 && trajPosDeg > 80,
                "profile position logged plausibly: " + trajPosDeg);
    }
}
