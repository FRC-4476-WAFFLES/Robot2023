package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class JoystickDrive {
  private final DoubleSupplier forwardAxis;
  private final DoubleSupplier rightAxis;
  private final DoubleSupplier rotationAxis;

  public JoystickDrive(DoubleSupplier forwardAxis, DoubleSupplier rightAxis, DoubleSupplier rotationAxis) {
    this.forwardAxis = forwardAxis;
    this.rightAxis = rightAxis;
    this.rotationAxis = rotationAxis;
  }

  public JoystickDrive() {
    this.forwardAxis = leftJoystick::getY;
    this.rightAxis = leftJoystick::getX;
    this.rotationAxis = rightJoystick::getX;
  }
  
  public double getForward() {
    double forward = forwardAxis.getAsDouble();
    forward = MathUtil.applyDeadband(forward, 0.05);
    forward *= -Constants.DriveConstants.maxAttainableSpeedMetersPerSecond;
    return forward;
  }

  public double getRight() {
    double right = rightAxis.getAsDouble();
    right = MathUtil.applyDeadband(right, 0.05);
    right *= -Constants.DriveConstants.maxAttainableSpeedMetersPerSecond;
    return right;
  }

  public double getRotation() {
    double rotation = rotationAxis.getAsDouble();
    rotation = MathUtil.applyDeadband(rotation, 0.05);
    rotation *= -Constants.DriveConstants.maxAttainableRotationRateRadiansPerSecond;
    return rotation;
  }
}
