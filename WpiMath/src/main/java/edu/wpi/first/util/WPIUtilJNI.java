// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.util;

/** Pure-Java stub for WPIUtilJNI that supports mock time for testing. */
public final class WPIUtilJNI {
  private static boolean mockTimeEnabled = false;
  private static long mockTimeMicros = 0;

  private WPIUtilJNI() {}

  /** Enables mock time. While enabled, {@link #now()} returns the mock time. */
  public static void enableMockTime() {
    mockTimeEnabled = true;
  }

  /** Disables mock time. After this, {@link #now()} returns real wall-clock time. */
  public static void disableMockTime() {
    mockTimeEnabled = false;
  }

  /**
   * Sets the mock time.
   *
   * @param timeMicros The mock time in microseconds.
   */
  public static void setMockTime(long timeMicros) {
    mockTimeMicros = timeMicros;
  }

  /**
   * Returns the current time in microseconds. Returns the mock time when mock time is enabled,
   * otherwise returns real wall-clock time.
   *
   * @return The current time in microseconds.
   */
  public static long now() {
    if (mockTimeEnabled) {
      return mockTimeMicros;
    }
    return System.nanoTime() / 1000;
  }
}
