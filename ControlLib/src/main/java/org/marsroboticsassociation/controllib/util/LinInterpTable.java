package org.marsroboticsassociation.controllib.util;

import java.util.Map;
import java.util.TreeMap;

public class LinInterpTable {
    private final TreeMap<Double, Double> map = new TreeMap<>();

    public double calculate(double x) {
        Map.Entry<Double, Double> ceiling = map.ceilingEntry(x);
        Map.Entry<Double, Double> lower = map.lowerEntry(x);
        if (ceiling != null) {
            if (lower != null) {
                return linearInterpolation(x, lower.getKey(), ceiling.getKey(), lower.getValue(), ceiling.getValue());
            } else {
                return ceiling.getValue();
            }
        } else {
            if (lower != null) {
                return lower.getValue();
            } else {
                throw new IllegalStateException("Table not initialized");
            }
        }
    }

    public void add(double x, double y) {
        map.put(x, y);
    }

    public static double linearInterpolation(double x, double x1, double x2, double y1, double y2) {
        return (x1 == x2) ? 0.0 : (y1 + (x - x1) * (y2 - y1) / (x2 - x1));
    }
}
