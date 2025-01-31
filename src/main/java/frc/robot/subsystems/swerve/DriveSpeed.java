package frc.robot.subsystems.swerve;

/**
 * The DriveSpeed class is responsible for calculating the drive speeds based on joystick inputs.
 * It computes the speed and direction for the robot's movement, including a braking mechanism.
 */
public class DriveSpeed {
  public double xSpeedControlInput; // Renamed from xControlInput for clarity
  public double ySpeedControlInput; // Renamed from yControlInput for clarity
  private double brakeSpeedRate; // Renamed from brakeSpeed for clarity
  private double lastSpeedDistance; // Renamed from lastDistance for clarity
  private double lastSpeedAngle; // Renamed from lastAngle for clarity

  /**
   * Constructor for DriveSpeed.
   * Initializes the drive speed calculation with a specified brake speed.
   * @param brakeSpeedRate The speed at which the robot should brake.
   */
  public DriveSpeed(double brakeSpeedRate) {
    this.brakeSpeedRate = brakeSpeedRate;
  }

  /**
   * Computes the drive speeds based on joystick inputs.
   * This method calculates the x and y speeds based on the provided joystick inputs.
   * @param xSpeedControlInput The x-axis input from the joystick.
   * @param ySpeedControlInput The y-axis input from the joystick.
   * @return An array containing the computed x and y speeds.
   */
  public double[] compute(double xSpeedControlInput, double ySpeedControlInput) {
    double distance = Math.sqrt(Math.pow(xSpeedControlInput, 2) + Math.pow(ySpeedControlInput, 2)); //Pythagorean Theorem
    double angle = Math.atan2(ySpeedControlInput, xSpeedControlInput);

    // if distance is 0, start decreasing the speed by the brake speed
    if (distance == 0) {
      lastSpeedDistance = Math.max(lastSpeedDistance - brakeSpeedRate, 0);

      updateSpeeds(lastSpeedDistance, lastSpeedAngle);
    } else {
      lastSpeedDistance = distance;
      lastSpeedAngle = angle;

      updateSpeeds(distance, angle);
    }

    return new double[]{xSpeedControlInput, ySpeedControlInput};
  }

  /**
   * Updates the speeds based on the computed distance and angle.
   * This method updates the internal x and y control inputs based on the provided distance and angle.
   * @param distance The computed distance.
   * @param lastSpeedAngle The computed angle.
   */
  private void updateSpeeds(double distance, double lastSpeedAngle) {
    xSpeedControlInput = distance * Math.cos(lastSpeedAngle);
    ySpeedControlInput = distance * Math.sin(lastSpeedAngle);
  }
}
