package com.ruckig;

/**
 * Port of {@code Calculator} from {@code calculator.hpp} (upstream v0.17.3).
 * Community version: target calculator only (no cloud waypoints).
 */
public final class Calculator {

    public final TargetCalculator target_calculator;

    public Calculator(int dofs) {
        target_calculator = new TargetCalculator(dofs);
    }

    public int calculate(InputParameter input, Trajectory trajectory, double delta_time, boolean[] was_interrupted) {
        return target_calculator.calculate(input, trajectory, delta_time, was_interrupted);
    }
}
