package com.ruckig;

/**
 * Port of {@code Ruckig} from {@code ruckig.hpp} (upstream v0.17.3).
 * Dynamic DoFs only; Community feature set.
 */
public final class Ruckig {

    private final InputParameter current_input;
    private boolean current_input_initialized;

    public final Calculator calculator;
    public final int max_number_of_waypoints;
    public final int degrees_of_freedom;
    public double delta_time;

    private final boolean[] wasInterruptedScratch = new boolean[1];
    private final int[] sectionScratch = new int[1];

    public Ruckig(int dofs) {
        this(dofs, -1.0);
    }

    public Ruckig(int dofs, double delta_time) {
        this.current_input = new InputParameter(dofs);
        this.current_input_initialized = false;
        this.calculator = new Calculator(dofs);
        this.max_number_of_waypoints = 0;
        this.degrees_of_freedom = dofs;
        this.delta_time = delta_time;
    }

    public void reset() {
        current_input_initialized = false;
    }

    /**
     * @param throwValidationError when false, invalid input returns false (matches upstream
     *        {@code Ruckig&lt;DOFs&gt;} default {@code throw_error=false}).
     */
    public boolean validate_input(InputParameter input, boolean throwValidationError,
                                  boolean check_current_state_within_limits, boolean check_target_state_within_limits) {
        if (!input.validate(throwValidationError, check_current_state_within_limits, check_target_state_within_limits)) {
            return false;
        }

        if (delta_time <= 0.0 && input.duration_discretization != DurationDiscretization.Continuous) {
            if (throwValidationError) {
                throw new RuckigError("delta time (control rate) parameter " + delta_time + " should be larger than zero.");
            }
            return false;
        }

        return true;
    }

    public boolean validate_input(InputParameter input) {
        return validate_input(input, true, false, true);
    }

    public int calculate(InputParameter input, Trajectory trajectory) {
        return calculate(input, trajectory, wasInterruptedScratch);
    }

    public int calculate(InputParameter input, Trajectory trajectory, boolean[] was_interrupted) {
        // Upstream default Ruckig<> does not throw on invalid input — returns ErrorInvalidInput.
        if (!validate_input(input, false, false, true)) {
            return Result.ErrorInvalidInput;
        }

        return calculator.calculate(input, trajectory, delta_time, was_interrupted);
    }

    public int update(InputParameter input, OutputParameter output) {
        final long startNs = System.nanoTime();

        if (degrees_of_freedom != input.degrees_of_freedom || degrees_of_freedom != output.degrees_of_freedom) {
            throw new RuckigError("mismatch in degrees of freedom (vector size).");
        }

        output.new_calculation = false;

        int result = Result.Working;
        if (!current_input_initialized || input.differsFrom(current_input)) {
            result = calculate(input, output.trajectory, wasInterruptedScratch);
            output.was_calculation_interrupted = wasInterruptedScratch[0];
            if (result != Result.Working && result != Result.ErrorPositionalLimits) {
                return result;
            }

            current_input.copyFrom(input);
            current_input_initialized = true;
            output.time = 0.0;
            output.new_calculation = true;
        }

        final int old_section = output.new_section;
        output.time += delta_time;
        sectionScratch[0] = output.new_section;
        output.trajectory.at_time(output.time, output.new_position, output.new_velocity, output.new_acceleration, output.new_jerk, sectionScratch);
        output.new_section = sectionScratch[0];
        output.did_section_change = (output.new_section > old_section);

        final long stopNs = System.nanoTime();
        output.calculation_duration = (stopNs - startNs) / 1000.0;

        output.pass_to_input(current_input);

        if (output.time > output.trajectory.get_duration()) {
            return Result.Finished;
        }

        return result;
    }
}
