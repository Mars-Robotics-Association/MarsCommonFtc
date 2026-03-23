package org.marsroboticsassociation.controllab.flywheel;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.function.Consumer;

/**
 * A helper component consisting of a label and a text field.
 * Displays an asterisk when the value is "dirty" (different from the last committed value).
 * Pressing Enter commits the value; pressing Escape reverts it.
 */
public class EditableParamField extends JPanel {
    private final JLabel label;
    private final JTextField textField;
    private final String originalLabel;
    private String committedValue;
    private final Consumer<Double> onCommit;

    public EditableParamField(String labelText, double initialValue, String format, Consumer<Double> onCommit) {
        setLayout(new FlowLayout(FlowLayout.LEFT, 5, 0));
        this.originalLabel = labelText;
        this.committedValue = String.format(format, initialValue);
        this.onCommit = onCommit;

        this.label = new JLabel(labelText + ": ");
        this.textField = new JTextField(committedValue, 8);

        add(this.label);
        add(this.textField);

        textField.addKeyListener(new KeyAdapter() {
            @Override
            public void keyReleased(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_ENTER) {
                    commit();
                } else if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
                    revert();
                } else {
                    updateDirtyState();
                }
            }
        });
    }

    private void updateDirtyState() {
        boolean dirty = !textField.getText().equals(committedValue);
        label.setText(originalLabel + (dirty ? "*: " : ": "));
    }

    private void commit() {
        try {
            double val = Double.parseDouble(textField.getText());
            committedValue = textField.getText();
            updateDirtyState();
            onCommit.accept(val);
            textField.transferFocus(); // Remove focus after commit
        } catch (NumberFormatException e) {
            revert(); // Revert on invalid input
        }
    }

    private void revert() {
        textField.setText(committedValue);
        updateDirtyState();
        textField.transferFocus();
    }

    public void setValue(double value, String format) {
        this.committedValue = String.format(format, value);
        this.textField.setText(committedValue);
        updateDirtyState();
    }

    public double getValue() {
        try {
            return Double.parseDouble(textField.getText());
        } catch (NumberFormatException e) {
            return Double.parseDouble(committedValue);
        }
    }
}
