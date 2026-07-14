package com.ruckig;

/**
 * Port of {@code ruckig/result.hpp} (upstream v0.17.3).
 */
public final class Result {
    public static final int Working = 0;
    public static final int Finished = 1;
    public static final int Error = -1;
    public static final int ErrorInvalidInput = -100;
    public static final int ErrorTrajectoryDuration = -101;
    public static final int ErrorPositionalLimits = -102;
    public static final int ErrorZeroLimits = -104;
    public static final int ErrorExecutionTimeCalculation = -110;
    public static final int ErrorSynchronizationCalculation = -111;

    private Result() {}
}
