package frc.robot.SwerveClasses;

public class Vector {
  public double x;
  public double y;

  public final double length;

  /**
   * Creates a new vector with the specified x and y coordinates.
   *
   * @param x The x coordinate
   * @param y The y coordinate
   * @since 0.1
   */
  public Vector(double x, double y) {
    this.x = x;
    this.y = y;

    this.length = Math.hypot(x, y);
  }

  /**
   * Multiplies each component of the vector by a scalar value.
   *
   * @param scalar The scalar to multiply each component by.
   * @return The vector scaled by the scalar.
   */
  public Vector scale(double scalar) {
    return multiply(scalar, scalar);
  }

  /**
   * Multiplies the components of this vector by two scalar values.
   *
   * @param x A scalar to multiply the x-coordinate by
   * @param y A scalar to multiply the y-coordinate by
   * @return A vector with the result of the multiplication
   * @since 0.1
   */
  public Vector multiply(double x, double y) {
    return new Vector(this.x * x, this.y * y);
  }
}
