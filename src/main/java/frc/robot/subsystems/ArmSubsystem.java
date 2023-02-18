// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.kinematics.ThreeJointArmKinematics;
import frc.robot.kinematics.ThreeJointArmState;
import frc.robot.kinematics.constraint.CircleConstraint;
import frc.robot.utils.SparkMaxAbsoluteControlHelper;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMaxAbsoluteControlHelper shoulder1;
  private final SparkMaxAbsoluteControlHelper shoulder2;
  private final SparkMaxAbsoluteControlHelper elbow1;
  private final SparkMaxAbsoluteControlHelper elbow2;
  private final SparkMaxAbsoluteControlHelper wrist1;
  private final SparkMaxAbsoluteControlHelper wrist2;

  private Pose2d targetPos = new Pose2d();

  private final ThreeJointArmKinematics kinematics;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    shoulder1 = new SparkMaxAbsoluteControlHelper(Constants.armMotors[0]);
    shoulder2 = new SparkMaxAbsoluteControlHelper(Constants.armMotors[1]);
    elbow1 = new SparkMaxAbsoluteControlHelper(Constants.armMotors[2]);
    elbow2 = new SparkMaxAbsoluteControlHelper(Constants.armMotors[3]);
    wrist1 = new SparkMaxAbsoluteControlHelper(Constants.armMotors[4]);
    wrist2 = new SparkMaxAbsoluteControlHelper(Constants.armMotors[5]);

    kinematics = new ThreeJointArmKinematics(Constants.ArmConstants.a1, Constants.ArmConstants.a2, Constants.ArmConstants.a3);
    kinematics.addConstraint(
      new CircleConstraint(
        (rotation) -> Constants.ArmConstants.a3 * rotation.getCos(), 
        (rotation) -> Constants.ArmConstants.a3 * rotation.getSin(), 
        Constants.ArmConstants.a1 + Constants.ArmConstants.a2, 
        true
      )
    );
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Rotation2d currentQ1 = Rotation2d.fromDegrees(shoulder1.getPosition());
    Rotation2d currentQ2 = Rotation2d.fromDegrees(elbow1.getPosition());
    Rotation2d currentQ3 = Rotation2d.fromDegrees(wrist1.getPosition()).plus(currentQ2).plus(currentQ1);

    Pose2d currentPose = kinematics.toPose2d(new ThreeJointArmState(currentQ1, currentQ2, currentQ3));
    SmartDashboard.putNumber("/arm/pose/x", currentPose.getX());
    SmartDashboard.putNumber("/arm/pose/y", currentPose.getY());
    SmartDashboard.putNumber("/arm/pose/angle", currentPose.getRotation().getDegrees());

    ThreeJointArmState targetArmState = kinematics.toArmState(targetPos);

    shoulder1.setTarget(targetArmState.q1.getDegrees());
    shoulder2.setTarget(targetArmState.q1.getDegrees());
    elbow1.setTarget(targetArmState.q2.getDegrees());
    elbow2.setTarget(targetArmState.q2.getDegrees());
    wrist1.setTarget(targetArmState.q3.minus(targetArmState.q2).minus(targetArmState.q1).getDegrees());
    wrist2.setTarget(targetArmState.q3.minus(targetArmState.q2).minus(targetArmState.q1).getDegrees());
  }
}