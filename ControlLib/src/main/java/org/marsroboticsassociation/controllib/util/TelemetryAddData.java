package org.marsroboticsassociation.controllib.util;

@FunctionalInterface
public interface TelemetryAddData {
    void addData(String caption, String format, Object value);
}
