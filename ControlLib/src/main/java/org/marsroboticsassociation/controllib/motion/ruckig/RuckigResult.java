package org.marsroboticsassociation.controllib.motion.ruckig;

/** Maps to ruckig::Result from result.hpp. */
public enum RuckigResult {
    WORKING(0),
    FINISHED(1),
    ERROR(-1),
    ERROR_INVALID_INPUT(-100),
    ERROR_TRAJECTORY_DURATION(-101),
    ERROR_ZERO_LIMITS(-104),
    ERROR_EXECUTION_TIME_CALCULATION(-110),
    ERROR_SYNCHRONIZATION_CALCULATION(-111);

    public final int code;

    RuckigResult(int code) {
        this.code = code;
    }

    public static RuckigResult fromCode(int code) {
        for (RuckigResult r : values()) {
            if (r.code == code) return r;
        }
        return ERROR;
    }
}
