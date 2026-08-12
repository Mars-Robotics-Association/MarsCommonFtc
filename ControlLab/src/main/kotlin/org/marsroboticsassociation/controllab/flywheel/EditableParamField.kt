package org.marsroboticsassociation.controllab.flywheel

import java.awt.Color
import java.awt.FlowLayout
import java.awt.event.KeyAdapter
import java.awt.event.KeyEvent
import java.util.function.Consumer
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.JTextField
import javax.swing.UIManager

/**
 * A helper component consisting of a label and a text field. Displays an asterisk when the value is
 * "dirty" (different from the last committed value). Pressing Enter commits the value; pressing
 * Escape reverts it.
 */
class EditableParamField : JPanel {
    private val label: JLabel
    private val textField: JTextField
    private val originalLabel: String
    private var committedValue: String
    private val onCommit: Consumer<Double>
    private val min: Double
    private val max: Double

    constructor(
        labelText: String,
        initialValue: Double,
        format: String,
        onCommit: Consumer<Double>,
    ) : this(
        labelText,
        initialValue,
        format,
        Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        onCommit,
    )

    constructor(
        labelText: String,
        initialValue: Double,
        format: String,
        min: Double,
        max: Double,
        onCommit: Consumer<Double>,
    ) {
        layout = FlowLayout(FlowLayout.LEFT, 5, 0)
        this.originalLabel = labelText
        this.committedValue = String.format(format, initialValue)
        this.onCommit = onCommit
        this.min = min
        this.max = max

        this.label = JLabel("$labelText: ")
        this.textField = JTextField(committedValue, 8)

        add(this.label)
        add(this.textField)

        textField.addKeyListener(
            object : KeyAdapter() {
                override fun keyPressed(e: KeyEvent) {
                    if (e.keyCode == KeyEvent.VK_ENTER) {
                        commit()
                        e.consume()
                    } else if (e.keyCode == KeyEvent.VK_ESCAPE) {
                        revert()
                        e.consume()
                    }
                }

                override fun keyReleased(e: KeyEvent) {
                    if (e.keyCode != KeyEvent.VK_ENTER && e.keyCode != KeyEvent.VK_ESCAPE) {
                        updateDirtyState()
                    }
                }
            }
        )
    }

    private fun updateDirtyState() {
        val dirty = textField.text != committedValue
        label.text = originalLabel + (if (dirty) "*: " else ": ")

        // Visual cue for invalid value
        try {
            val `val` = textField.text.toDouble()
            if (`val` < min || `val` > max) {
                textField.foreground = Color.RED
            } else {
                textField.foreground = UIManager.getColor("TextField.foreground")
            }
        } catch (_: NumberFormatException) {
            textField.foreground = Color.RED
        }
    }

    private fun commit() {
        try {
            val `val` = textField.text.toDouble()
            if (`val` >= min && `val` <= max) {
                committedValue = textField.text
                updateDirtyState()
                onCommit.accept(`val`)
            } else {
                revert() // Out of bounds
            }
        } catch (_: NumberFormatException) {
            revert() // Revert on invalid input
        }
    }

    private fun revert() {
        textField.text = committedValue
        updateDirtyState()
    }

    fun setValue(value: Double, format: String) {
        this.committedValue = String.format(format, value)
        this.textField.text = committedValue
        updateDirtyState()
    }

    /** Current field value (dirty text if parseable, else committed). JVM: getValue(). */
    val value: Double
        get() {
            return try {
                textField.text.toDouble().coerceIn(min, max)
            } catch (_: NumberFormatException) {
                committedValue.toDouble()
            }
        }
}
