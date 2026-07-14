package com.ruckig;

/**
 * Port of {@code ruckig/error.hpp} (upstream v0.17.3).
 */
public class RuckigError extends RuntimeException {
    public RuckigError(String message) {
        super("\n[ruckig] " + message);
    }
}
