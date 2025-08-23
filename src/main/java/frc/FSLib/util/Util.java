package frc.FSLib.util;

public class Util {
  /**
   * Checks if a given value is within the inclusive range defined by the minimum
   * and maximum bounds.
   * 
   * @param value the value to check
   * @param min   the minimum bound (inclusive)
   * @param max   the maximum bound (inclusive)
   * @return {@code true} if the value is within the range {@code [min, max]},
   *         {@code false} otherwise
   * @throws IllegalArgumentException if {@code max} is less than {@code min}
   */
  public static boolean isWithin(double value, double min, double max) {
    if (max < min) {
      throw new IllegalArgumentException("max must be greater than or equal to min");
    }
    return min <= value && value <= max;
  }

  /**
   * Clamps a given value to be within the inclusive range defined by the minimum
   * and maximum bounds.
   * If the value is less than the minimum, the minimum is returned; if it is
   * greater than the maximum,
   * the maximum is returned; otherwise, the value itself is returned.
   *
   * @param value the value to clamp
   * @param min   the minimum bound (inclusive)
   * @param max   the maximum bound (inclusive)
   * @return the clamped value within the range {@code [min, max]}
   * @throws IllegalArgumentException if {@code max} is less than {@code min}
   */
  public static double clamp(double value, double min, double max) {
    if (max < min) {
      throw new IllegalArgumentException("max must be greater than or equal to min");
    }
    return Math.min(Math.max(value, min), max);
  }
}
