package org.marsroboticsassociation.controllib.mechanism

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class FeedbackGainSynthesisTest {

    @Test
    fun suggestPd_matchesSecondOrderPlacement() {
        // Heavy-arm SysID defaults from ControlLab: kV=1.2, kA=0.35
        val kV = 1.2
        val kA = 0.35
        val omegaN = 4.0
        val zeta = 0.8

        val s = FeedbackGainSynthesis.suggestPd(kV, kA, omegaN, zeta)

        assertEquals(kA * omegaN * omegaN, s.kP, 1e-12)
        assertEquals(2.0 * zeta * omegaN * kA - kV, s.kD, 1e-12)
        assertFalse(s.kDClampedToZero)
        // Spot-check numbers: kP = 5.6, kD = 1.04
        assertEquals(5.6, s.kP, 1e-9)
        assertEquals(1.04, s.kD, 1e-9)
    }

    @Test
    fun suggestPd_clampsNegativeKdWhenPlantAlreadyDamps() {
        // High kV, low ωₙ → plant damping exceeds 2ζωₙ kA
        val s = FeedbackGainSynthesis.suggestPd(/* kV= */ 5.0, /* kA= */ 0.2, /* ωₙ= */ 1.0, 0.7)
        assertEquals(0.0, s.kD, 0.0)
        assertTrue(s.kDClampedToZero)
        assertEquals(0.2, s.kP, 1e-12)
    }

    @Test
    fun suggestPd_fromMechanismModel() {
        val model = ArmModel(0.3, 1.2, 0.35, 3.5, 0.0)
        val s = FeedbackGainSynthesis.suggestPd(model, 4.0, 0.8)
        assertEquals(5.6, s.kP, 1e-9)
        assertEquals(1.04, s.kD, 1e-9)
    }

    @Test
    fun suggestPd_rejectsInvalidInputs() {
        assertThrows(IllegalArgumentException::class.java) {
            FeedbackGainSynthesis.suggestPd(1.0, 0.0, 4.0, 0.8)
        }
        assertThrows(IllegalArgumentException::class.java) {
            FeedbackGainSynthesis.suggestPd(-0.1, 0.35, 4.0, 0.8)
        }
        assertThrows(IllegalArgumentException::class.java) {
            FeedbackGainSynthesis.suggestPd(1.0, 0.35, 0.0, 0.8)
        }
        assertThrows(IllegalArgumentException::class.java) {
            FeedbackGainSynthesis.suggestPd(1.0, 0.35, 4.0, 0.0)
        }
    }
}
