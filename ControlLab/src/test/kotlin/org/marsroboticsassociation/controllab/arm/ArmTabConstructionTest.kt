package org.marsroboticsassociation.controllab.arm

import java.awt.GraphicsEnvironment
import java.util.concurrent.atomic.AtomicReference
import javax.swing.SwingUtilities
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assumptions.assumeFalse
import org.junit.jupiter.api.Test

/**
 * Constructs the full [ArmTab] (canvas + XChart + sidebar + timer) on the EDT to catch runtime
 * wiring errors the compiler cannot — e.g. an XChart multi-axis call or a Swing layout mistake.
 * Skipped when there is no display (headless CI); it runs on a developer's machine.
 */
class ArmTabConstructionTest {

    @Test
    fun constructsWithoutThrowing() {
        assumeFalse(GraphicsEnvironment.isHeadless(), "no display available")
        val failure = AtomicReference<Throwable>()
        SwingUtilities.invokeAndWait {
            try {
                val tab = ArmTab()
                tab.dispose() // stop the sim timer so the test process can exit
            } catch (t: Throwable) {
                failure.set(t)
            }
        }
        assertNull(failure.get(), "ArmTab construction threw: " + failure.get())
    }
}
