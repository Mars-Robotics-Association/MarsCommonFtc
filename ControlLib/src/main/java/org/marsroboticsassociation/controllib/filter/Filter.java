package org.marsroboticsassociation.controllib.filter;

public interface Filter {
    /**
     * Update filter with the next sample value using elapsed dt (seconds).
     * If dt <= 0, filter may decide to initialize or return the input.
     */
    double update(double x, double dt);

    /**
     * @return Derivative estimate, or NaN if filter type does not support it
     */
    double getRate();

    double getValue();

    /**
     * Reset internal state.
     */
    void reset();
}
