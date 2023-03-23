// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Height;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.kinematics.ThreeJointArmKinematics;
import frc.robot.kinematics.ThreeJointArmState;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // RobotBase.startRobot(Robot::new);

    ThreeJointArmKinematics kinematics = new ThreeJointArmKinematics(1.06680, 0.9144, 0.11223);
    // 0.11223

    // 0.06185
    // 0.09364
    // 33.45 intake angle offset

    // SetPoint setPoint = setPoints.get(new ArmState(Height.LOW, GamePiece.CONE, false, false));
    // System.out.println(setPoint);
    // System.out.println();

    // double unadjustedShoudlerAngle = shoulderToDegrees(setPoint.shoulderPos);
    // double shoulderAngle = 90.0 - unadjustedShoudlerAngle;
    // double elbowAngle = -elbowToDegrees(setPoint.elbowPos) - 90.0 + unadjustedShoudlerAngle * Constants.ArmConstants.elbowChainRunRatio;
    // double relativeWristAngle = wristToDegrees(setPoint.wristPos);
    // double wristAngle = -relativeWristAngle + shoulderAngle + elbowAngle;

    // System.out.println("shoulder angle: " + String.valueOf(shoulderAngle));
    // System.out.println("elbow angle: " + String.valueOf(elbowAngle));
    // System.out.println(relativeWristAngle);
    // System.out.println("wrist angle: " + String.valueOf(wristAngle));

    // System.out.println();

    // Pose2d armPose = kinematics.toPose2d(new ThreeJointArmState(
    //   Rotation2d.fromDegrees(shoulderAngle), 
    //   Rotation2d.fromDegrees(elbowAngle),
    //   Rotation2d.fromDegrees(wristAngle + 56.55))
    // );
    // System.out.println(armPose);

    Pose2d armPose = new Pose2d(1.49, 1.16, Rotation2d.fromDegrees(-33));

    ThreeJointArmState state = kinematics.toArmState(armPose);
    
    System.out.println(state);
    System.out.println(kinematics.toPose2d(state));

    double shoulderMotorPose = shoulderDegreesRelativeToMotorPos(shoulderDegreesAbsoluteToRelative(state.q1.getDegrees()));
    double elbowMotorPose = elbowDegreesRelativeToMotorPos(elbowDegreesAbsoluteToRelative(state.q2.getDegrees(), state.q1.getDegrees()));
    double wristMotorPose = wristDegreesRelativeToMotorPos(wristDegreesAbsoluteToRelative(state.q3.getDegrees(), state.q2.getDegrees(), state.q1.getDegrees()));

    System.out.println();
    System.out.println(String.format("Shoulder motor pose: %6.0f", shoulderMotorPose));
    System.out.println(String.format("Elbow motor pose: %6.0f", elbowMotorPose));
    System.out.println(String.format("Wrist motor pose: %2.2f", wristMotorPose));

    System.out.println();

    double shoulderAngleDegreesRelative = shoulderMotorPosToDegreesRelative(shoulderMotorPose);
    double shoulderAngleDegreesAbsolute = shoulderDegreesRelativeToAbsolute(shoulderAngleDegreesRelative);
    double elbowAngleDegreesAbsolute = elbowDegreesRelativeToAbsolute(elbowMotorPosToDegreesRelative(elbowMotorPose), shoulderAngleDegreesRelative);
    double wristAngleDegreesAbsolute = wristDegreesRelativeToAbsolute(wristMotorPosToDegreesRelative(wristMotorPose), elbowAngleDegreesAbsolute, shoulderAngleDegreesAbsolute);

    System.out.println(String.format("Shoulder absolute angle: %3.2f", shoulderAngleDegreesAbsolute));
    System.out.println(String.format("Elbow absolute angle: %3.2f", elbowAngleDegreesAbsolute));
    System.out.println(String.format("Wrist absolute angle: %3.2f", wristAngleDegreesAbsolute));

    System.out.println();

    Pose2d armPose2 = kinematics.toPose2d(new ThreeJointArmState(
      Rotation2d.fromDegrees(shoulderAngleDegreesAbsolute), 
      Rotation2d.fromDegrees(elbowAngleDegreesAbsolute),
      Rotation2d.fromDegrees(wristAngleDegreesAbsolute))
    );
    System.out.println(armPose2);
  }

  private static double shoulderMotorPosToDegreesRelative(double motorPos) {
    return motorPos / 2048.0 * 360.0 * Constants.ArmConstants.shoulderRatio;
  }

  private static double shoulderDegreesRelativeToMotorPos(double degrees) {
    return degrees / 360.0 * 2048.0 / Constants.ArmConstants.shoulderRatio;
  }

  private static double shoulderDegreesAbsoluteToRelative(double degreesAbsolute) {
    return 90.0 - degreesAbsolute;
  }

  private static double shoulderDegreesRelativeToAbsolute(double degreesRelative) {
    return 90.0 - degreesRelative;
  }

  private static double elbowMotorPosToDegreesRelative(double motorPos) {
    return motorPos / 2048.0 * 360.0 * Constants.ArmConstants.elbowBaseRatio * Constants.ArmConstants.elbowChainRunRatio;
  }

  private static double elbowDegreesRelativeToMotorPos(double degrees) {
    return -degrees * 2048.0 / 360.0 / Constants.ArmConstants.elbowBaseRatio / Constants.ArmConstants.elbowChainRunRatio;
  }

  private static double elbowDegreesAbsoluteToRelative(double degreesAbsolute, double shoulderDegreesAbsolute) {
    double degreesAbsoluteWrapped = degreesAbsolute;
    if (degreesAbsoluteWrapped > 90) {
      degreesAbsoluteWrapped -= 360;
    }

    return degreesAbsoluteWrapped + 90.0 + (shoulderDegreesAbsolute - 90.0) * Constants.ArmConstants.elbowChainRunRatio;
  }

  private static double elbowDegreesRelativeToAbsolute(double degreesRelative, double shoulderDegreesRelative) {
    return -degreesRelative - 90.0 + shoulderDegreesRelative * Constants.ArmConstants.elbowChainRunRatio;
  }

  private static double wristMotorPosToDegreesRelative(double motorPos) {
    return motorPos * Constants.ArmConstants.wristRatio * 360.0;
  }

  private static double wristDegreesRelativeToMotorPos(double degrees) {
    return -degrees / Constants.ArmConstants.wristRatio / 360.0;
  }

  private static double wristDegreesAbsoluteToRelative(double degreesAbsolute, double elbowDegreesAbsolute, double shoulderDegreesAbsolute) {
    return degreesAbsolute - elbowDegreesAbsolute - shoulderDegreesAbsolute - 56.55;
  }

  private static double wristDegreesRelativeToAbsolute(double degreesRelative, double elbowAngleDegreesAbsolute, double shoulderDegreesAbsolute) {
    return -degreesRelative + elbowAngleDegreesAbsolute + shoulderDegreesAbsolute + 56.55;
  }
}
