/*
 * CREDIT FOR THESE UTILITITES GOES TO TEAM 1706
 * Thanks for unknowingly helping our beginner swerve drive!
 */

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.OperatorConstants;

public class MathUtils {
  public static double toUnitCircAngle(double angle) {
    double rotations = angle / (2 * Math.PI);
    return (angle - Math.round(rotations - 0.500) * Math.PI * 2.0);
  }

  public static double cubicLinear(double input, double a, double b) {
    return (a * Math.pow(input, 3) + b * input);
  }

  public static double applyDeadband(double input) {
    if (Math.abs(input) < OperatorConstants.kDriverDeadband) {
      return 0.0;
    } else if (input < 0.0) {
      return (input + OperatorConstants.kDriverDeadband) * (1.0 / (1 - OperatorConstants.kDriverDeadband));
    } else if (input > 0.0) {
      return (input - OperatorConstants.kDriverDeadband) * (1.0 / (1 - OperatorConstants.kDriverDeadband));
    } else {
      return 0.0;
    }
  }

  public static double inputTransform(double input) {
    return cubicLinear(applyDeadband(input), OperatorConstants.kCubic, OperatorConstants.kLinear);
  }

  public static double[] inputTransform(double x, double y) {
    x = applyDeadband(x);
    y = applyDeadband(y);
    double mag = new Translation2d(x, y).getDistance(new Translation2d());

    if (mag > 1.00) {
      mag = 1.00;
    }

    if (mag != 0) {
      x = x / mag * cubicLinear(mag, OperatorConstants.kCubic, OperatorConstants.kLinear);
      y = y / mag * cubicLinear(mag, OperatorConstants.kCubic, OperatorConstants.kLinear);
    } else {
      x = 0;
      y = 0;
    }

    return new double[] { x, y };
  }

}
