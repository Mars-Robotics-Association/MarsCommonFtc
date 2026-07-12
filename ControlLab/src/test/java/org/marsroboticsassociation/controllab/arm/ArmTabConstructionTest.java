package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;

import javax.swing.SwingUtilities;
import java.awt.GraphicsEnvironment;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

/**
 * Constructs the full {@link ArmTab} (canvas + XChart + sidebar + timer) on the EDT to catch runtime
 * wiring errors the compiler cannot — e.g. an XChart multi-axis call or a Swing layout mistake. Skipped
 * when there is no display (headless CI); it runs on a developer's machine.
 */
class ArmTabConstructionTest {

    @Test
    void constructsWithoutThrowing() throws Exception {
        assumeFalse(GraphicsEnvironment.isHeadless(), "no display available");
        AtomicReference<Throwable> failure = new AtomicReference<>();
        SwingUtilities.invokeAndWait(() -> {
            try {
                ArmTab tab = new ArmTab();
                tab.dispose(); // stop the sim timer so the test process can exit
            } catch (Throwable t) {
                failure.set(t);
            }
        });
        assertNull(failure.get(), "ArmTab construction threw: " + failure.get());
    }
}
