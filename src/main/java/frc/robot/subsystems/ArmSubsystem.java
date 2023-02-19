// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.kinematics.ThreeJointArmKinematics;
import frc.robot.kinematics.ThreeJointArmState;
import frc.robot.kinematics.constraint.CircleConstraint;
import frc.robot.utils.SparkMaxAbsoluteControlHelper;

public class ArmSubsystem extends SubsystemBase {
  // private Pose2d targetPos = new Pose2d();

  private final CANSparkMax climb1Left;
  private final CANSparkMax climb1Right;
  private final CANSparkMax climb2Left;
  private final CANSparkMax climb2Right;
  private final CANSparkMax intakePivotLeft;
  private final CANSparkMax intakePivotRight;
  
  private final SparkMaxPIDController climb1LeftPID;
  private final SparkMaxPIDController climb1RightPID;
  private final SparkMaxPIDController climb2LeftPID;
  private final SparkMaxPIDController climb2RightPID;
  private final SparkMaxPIDController intakePivotLeftPID;

  private final RelativeEncoder climb1LeftRelativeEncoder;
  private final RelativeEncoder climb1RightRelativeEncoder;
  private final RelativeEncoder climb2LeftRelativeEncoder;
  private final RelativeEncoder climb2RightRelativeEncoder;
  private final RelativeEncoder intakePivotLeftEncoder;
  private final RelativeEncoder intakePivotRightEncoder;

  private final DutyCycleEncoder climb1LeftAbsoluteEncoder;
  private final DutyCycleEncoder climb1RightEncoder; 
  private final DutyCycleEncoder climb2Encoder;
  private final DutyCycleEncoder intakePivotEncoder;

  private final ThreeJointArmKinematics kinematics;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    climb1Left = new CANSparkMax(Constants.climb1LeftConstants.motorID, MotorType.kBrushless);
    climb1Right = new CANSparkMax(Constants.climb1RightConstants.motorID, MotorType.kBrushless);
    climb2Left = new CANSparkMax(Constants.climb2LeftConstants.motorID, MotorType.kBrushless);
    climb2Right = new CANSparkMax(Constants.climb2RightConstants.motorID, MotorType.kBrushless);
    intakePivotLeft = new CANSparkMax(Constants.intakePivotLeftConstants.motorID, MotorType.kBrushless);
    intakePivotRight = new CANSparkMax(Constants.intakePivotRightConstants.motorID, MotorType.kBrushless);

    climb1Left.restoreFactoryDefaults();
    climb1Right.restoreFactoryDefaults();
    climb2Left.restoreFactoryDefaults();
    climb2Right.restoreFactoryDefaults();
    intakePivotLeft.restoreFactoryDefaults();
    intakePivotRight.restoreFactoryDefaults();

    climb1Left.setSmartCurrentLimit(Constants.climb1LeftConstants.currentLimit);
    climb1Right.setSmartCurrentLimit(Constants.climb1RightConstants.currentLimit);
    climb2Left.setSmartCurrentLimit(Constants.climb2LeftConstants.currentLimit);
    climb2Right.setSmartCurrentLimit(Constants.climb2RightConstants.currentLimit);
    intakePivotLeft.setSmartCurrentLimit(Constants.intakePivotLeftConstants.currentLimit);
    intakePivotRight.setSmartCurrentLimit(Constants.intakePivotRightConstants.currentLimit);

    climb1Left.setIdleMode(IdleMode.kBrake);
    climb1Right.setIdleMode(IdleMode.kBrake);
    climb2Left.setIdleMode(IdleMode.kBrake);
    climb2Right.setIdleMode(IdleMode.kBrake);
    intakePivotLeft.setIdleMode(IdleMode.kBrake);
    intakePivotRight.setIdleMode(IdleMode.kBrake);

    climb1Left.setControlFramePeriodMs(40);
    climb1Right.setControlFramePeriodMs(40);
    climb2Left.setControlFramePeriodMs(40);
    climb2Right.setControlFramePeriodMs(40);
    intakePivotLeft.setControlFramePeriodMs(40);
    intakePivotRight.setControlFramePeriodMs(40);

    climb1Left.setInverted(Constants.climb1LeftConstants.isInverted);
    climb1Right.setInverted(Constants.climb1RightConstants.isInverted);
    climb2Left.setInverted(Constants.climb2LeftConstants.isInverted);
    climb2Right.setInverted(Constants.climb2RightConstants.isInverted);
    intakePivotLeft.setInverted(Constants.intakePivotLeftConstants.isInverted);

    intakePivotRight.follow(intakePivotLeft, Constants.intakePivotRightConstants.isInverted);

    climb1LeftPID = climb1Left.getPIDController();
    climb1RightPID = climb1Right.getPIDController();
    climb2LeftPID = climb2Left.getPIDController();
    climb2RightPID = climb2Right.getPIDController();
    intakePivotLeftPID = intakePivotLeft.getPIDController();

    climb1LeftRelativeEncoder = climb1Left.getEncoder();
    climb1RightRelativeEncoder = climb1Right.getEncoder();
    climb2LeftRelativeEncoder = climb2Left.getEncoder();
    climb2RightRelativeEncoder = climb2Right.getEncoder();
    intakePivotLeftEncoder = intakePivotLeft.getEncoder();
    intakePivotRightEncoder = intakePivotRight.getEncoder();

    climb1LeftAbsoluteEncoder = new DutyCycleEncoder(Constants.climb1LeftEncoder);
    climb1RightEncoder = new DutyCycleEncoder(Constants.climb1RightEncoder);
    climb2Encoder = new DutyCycleEncoder(Constants.climb2LeftEncoder);
    intakePivotEncoder = new DutyCycleEncoder(Constants.intakePivotEncoder);

    climb1LeftAbsoluteEncoder.setDistancePerRotation(360);
    climb1RightEncoder.setDistancePerRotation(360);
    climb2Encoder.setDistancePerRotation(360);
    intakePivotEncoder.setDistancePerRotation(360);

    kinematics = new ThreeJointArmKinematics(Constants.ArmConstants.a1, Constants.ArmConstants.a2, Constants.ArmConstants.a3);
    // kinematics.addConstraint(
    //   new CircleConstraint(
    //     (rotation) -> Constants.ArmConstants.a3 * rotation.getCos(), 
    //     (rotation) -> Constants.ArmConstants.a3 * rotation.getSin(), 
    //     Constants.ArmConstants.a1 + Constants.ArmConstants.a2, 
    //     true
    //   )
    // );
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Rotation2d currentQ1 = Rotation2d.fromDegrees(climb1LeftRelativeEncoder.getPosition() * 360 * Constants.climb1LeftConstants.ratio);
    Rotation2d currentQ2 = Rotation2d.fromDegrees(climb2LeftRelativeEncoder.getPosition() * 360 * Constants.climb2LeftConstants.ratio);
    Rotation2d currentQ3 = Rotation2d.fromDegrees(intakePivotLeftEncoder.getPosition() * 360 * Constants.intakePivotLeftConstants.ratio).plus(currentQ2).plus(currentQ1);

    Pose2d currentPose = kinematics.toPose2d(new ThreeJointArmState(currentQ1, currentQ2, currentQ3));
    SmartDashboard.putNumber("/arm/pose/x", currentPose.getX());
    SmartDashboard.putNumber("/arm/pose/y", currentPose.getY());
    SmartDashboard.putNumber("/arm/pose/angle", currentPose.getRotation().getDegrees());

    // ThreeJointArmState targetArmState = kinematics.toArmState(targetPos);
    
    SmartDashboard.putNumber("Left 1 encoder", climb1LeftAbsoluteEncoder.getDistance());
    SmartDashboard.putNumber("Right 1 encoder", climb1RightEncoder.getDistance());
    SmartDashboard.putNumber("Left 2 Encoder", climb2Encoder.getDistance());
    SmartDashboard.putNumber("Wrist Encoder", intakePivotEncoder.getDistance());
  }
}
