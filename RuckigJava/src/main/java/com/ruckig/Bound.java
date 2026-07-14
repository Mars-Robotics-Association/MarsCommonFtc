package com.ruckig;

/**
 * Port of {@code Bound} from {@code profile.hpp} (upstream v0.17.3).
 * Position extrema of a single-DoF profile.
 */
public final class Bound {
    public double min;
    public double max;
    public double t_min;
    public double t_max;

    public Bound() {
        min = 0.0;
        max = 0.0;
        t_min = 0.0;
        t_max = 0.0;
    }

    public void copyFrom(Bound o) {
        min = o.min;
        max = o.max;
        t_min = o.t_min;
        t_max = o.t_max;
    }
}
