// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // 0.06185
    // 0.09364
    // 33.45 intake angle offset

    // SetPoint setPoint = setPoints.get(i);
    // SetPoint setPoint = setPoints.get(new ArmState(ArmState.Side.FRONT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, false, false).hashCode());
    SetPoint setPoint = setPoints.get(new ArmState(Height.LOW, GamePiece.CONE, false, false));
    System.out.println(setPoint);

    double unadjustedShoudlerAngle = setPoint.shoulderPos / 2048.0 * 360.0 * Constants.ArmConstants.shoulderRatio;
    double shoulderAngle = 90.0 - unadjustedShoudlerAngle;
    double elbowAngle = -setPoint.elbowPos / 2048.0 * 360.0 * Constants.ArmConstants.elbowBaseRatio * Constants.ArmConstants.elbowChainRunRatio - 90.0 + unadjustedShoudlerAngle * Constants.ArmConstants.elbowChainRunRatio;
    double relativeWristAngle = setPoint.wristPos * Constants.ArmConstants.wristRatio * 360.0;
    double wristAngle = relativeWristAngle - shoulderAngle - elbowAngle;

    System.out.println("shoulder angle: " + String.valueOf(shoulderAngle));
    System.out.println("elbow angle: " + String.valueOf(elbowAngle));
    System.out.println(relativeWristAngle);
    System.out.println("wrist angle: " + String.valueOf(wristAngle));

    System.out.println();

    Pose2d armPose = kinematics.toPose2d(new ThreeJointArmState(
      Rotation2d.fromDegrees(shoulderAngle), 
      Rotation2d.fromDegrees(elbowAngle),
      Rotation2d.fromDegrees(wristAngle + 56.55))
    );
    System.out.println(armPose);

    ThreeJointArmState state = kinematics.toArmState(armPose);
    
    System.out.println(state);

    System.out.println(kinematics.toPose2d(state));

    System.out.println();

    double shoulderMotorPose = (90.0 - state.q1.getDegrees()) / 360.0 * 2048.0 / Constants.ArmConstants.shoulderRatio;

    double elbowAnglePose = state.q2.getDegrees();
    if (elbowAnglePose > 90) {
      elbowAnglePose -= 360;
    }

    double elbowMotorPose = -((elbowAnglePose + 90.0 + (state.q1.getDegrees() - 90.0) * Constants.ArmConstants.elbowChainRunRatio) / 360.0 * 2048.0 / Constants.ArmConstants.elbowChainRunRatio / Constants.ArmConstants.elbowChainRunRatio);

    double wristMotorPose = (state.q3.minus(state.q2).minus(state.q1).getDegrees() - 56.55) / Constants.ArmConstants.wristRatio / 360.0;

    System.out.println(shoulderMotorPose);
    System.out.println(elbowMotorPose);
    System.out.println(wristMotorPose);

    System.out.println();

    // y = (-x / 2048.0 * 360.0 * ratio * chainRatio) - 90.0 + (shoulderAngle * chainRatio)
    // y + 90 = (-x / 2048.0 * 360.0 * ratio * chainRatio) + (shoulderAngle * chainRatio)
    // y + 90 - (shoulderAngle * chainRatio) = (-x / 2048.0 * 360.0 * ratio * chainRatio)
    // (y + 90 - (shoulderAngle * chainRatio)) * 2048 / 360 / ratio / chainRatio = -x

    // + 90.0 -  (90.0 - state.q1.getDegrees() * Constants.ArmConstants.stage2ChainRatio)

    // System.out.println(setPoint.arm1Pos);

    // Rotation2d currentQ1 = Rotation2d.fromDegrees(setPoint.arm1Pos / 2048.0 * Constants.arm1LeftConstants.ratio * 360.0).plus(Rotation2d.fromDegrees(90));
    // Rotation2d currentQ2 = Rotation2d.fromDegrees(-setPoint.arm1Pos / 2048.0 * Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio * 360.0).plus(currentQ1.times(Constants.ArmConstants.stage2ChainRatio)).plus(Rotation2d.fromDegrees(90));
    // Rotation2d currentQ3 = Rotation2d.fromDegrees(setPoint.arm1Pos * Constants.intakePivotLeftConstants.ratio * 360.0).plus(currentQ2).plus(currentQ1);

    // System.out.println(currentQ1);
    // System.out.println(currentQ2);
    // System.out.println(currentQ3);
    // // System.out.println(currentQ1);
    // Pose2d currentPose = kinematics.toPose2d(new ThreeJointArmState(currentQ1, currentQ2, currentQ3));

    // System.out.println(currentPose);

    // ThreeJointArmState recalculated = kinematics.toArmState(currentPose);

    // double recalculatedQ1 = recalculated.q1.minus(Rotation2d.fromDegrees(90)).getDegrees() / Constants.arm1LeftConstants.ratio * 2048.0 / 360.0;
    // double recalculatedQ2 = recalculated.q2.minus(Rotation2d.fromDegrees(90)).minus(recalculated.q1.div(Constants.ArmConstants.stage2ChainRatio)).getDegrees() / Constants.arm2LeftConstants.ratio / Constants.ArmConstants.stage2ChainRatio * 2048.0 / 360.0;
    // double recalculatedQ3 = recalculated.q3.minus(recalculated.q1).minus(recalculated.q2).getDegrees() / Constants.intakePivotLeftConstants.ratio / 360.0;

    // // System.out.println((int) currentQ1.getDegrees() == recalculatedQ1 && (int) currentQ2.getDegrees() == recalculatedQ2 && (int) currentQ3.getDegrees() == recalculatedQ3);

    // System.out.println(recalculatedQ1);
    // System.out.println(recalculatedQ2);
    // System.out.println(recalculatedQ3);
    // System.out.println(Math.abs(Math.abs(recalculatedQ1) - Math.abs(currentQ1.getDegrees())) < 5);
  
  }
}
