package com.ruckig;

/**
 * Port of {@code OutputParameter} from {@code output_parameter.hpp} (upstream v0.17.3).
 */
public final class OutputParameter {

    public final int degrees_of_freedom;

    public final Trajectory trajectory;

    public final double[] new_position;
    public final double[] new_velocity;
    public final double[] new_acceleration;
    public final double[] new_jerk;

    public double time;
    public int new_section;
    public boolean did_section_change;
    public boolean new_calculation;
    public boolean was_calculation_interrupted;
    public double calculation_duration; // [µs]

    public OutputParameter(int dofs) {
        this.degrees_of_freedom = dofs;
        this.trajectory = new Trajectory(dofs);
        new_position = new double[dofs];
        new_velocity = new double[dofs];
        new_acceleration = new double[dofs];
        new_jerk = new double[dofs];
        time = 0.0;
        new_section = 0;
        did_section_change = false;
        new_calculation = false;
        was_calculation_interrupted = false;
        calculation_duration = 0.0;
    }

    public void pass_to_input(InputParameter input) {
        System.arraycopy(new_position, 0, input.current_position, 0, degrees_of_freedom);
        System.arraycopy(new_velocity, 0, input.current_velocity, 0, degrees_of_freedom);
        System.arraycopy(new_acceleration, 0, input.current_acceleration, 0, degrees_of_freedom);
    }

    @Override
    public String toString() {
        StringBuilder ss = new StringBuilder("\nout.new_position = [");
        ss.append(Utils.join(new_position, true)).append("]\n");
        ss.append("out.new_velocity = [").append(Utils.join(new_velocity, true)).append("]\n");
        ss.append("out.new_acceleration = [").append(Utils.join(new_acceleration, true)).append("]\n");
        ss.append("out.new_jerk = [").append(Utils.join(new_jerk, true)).append("]\n");
        ss.append("out.time = [").append(String.format(java.util.Locale.US, "%.16g", time)).append("]\n");
        ss.append("out.calculation_duration = [").append(String.format(java.util.Locale.US, "%.16g", calculation_duration)).append("]\n");
        return ss.toString();
    }
}
