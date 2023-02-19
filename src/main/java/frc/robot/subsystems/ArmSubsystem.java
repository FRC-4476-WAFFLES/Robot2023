// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.fasterxml.jackson.databind.annotation.JsonAppend.Attr;
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

  private static class ArmState {
    enum Side{
      RIGHT,
      LEFT,
    }
    enum Height{
      HIGH,
      MEDIUM,
      LOW, 
      HPPICKUP,
    }
    enum GamePiece{
      CUBE,
      CONE,
    }
    public Side side;
    public Height height;
    public GamePiece piece;
    public boolean altpickupl;
    public boolean fudge;
    public boolean isFudgeElbow;
    ArmState(Side side, Height height, GamePiece piece, boolean altpickupl, boolean fudge){
      this.side= side;
      this.height= height;
      this.piece= piece;
      this.altpickupl= altpickupl;
      this.fudge=fudge;
      isFudgeElbow = false;
    }

    public boolean equals(ArmState o){
      return this.side==o.side && this.height==o.height && this.piece==o.piece && this.altpickupl==o.altpickupl;
    }
    public int hashcode(){
      int temp= 0;
      temp += side.ordinal();
      temp += height.ordinal()*2;
      temp += piece.ordinal()*4;
      temp += altpickupl ? 16 : 0;
      return temp;
    }
  }
  private static class SetPoint{

  }
  HashMap<ArmState,SetPoint> setPoints = new HashMap<ArmState, SetPoint>() {{
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.LOW, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.LOW, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.LOW, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.LOW, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, false, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, false, false), new SetPoint());

    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.LOW, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.LOW, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.LOW, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.LOW, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.RIGHT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, true, false), new SetPoint());
    put(new ArmState(ArmState.Side.LEFT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, true, false), new SetPoint());
  }};



  private ArmState state;
  private final CANSparkMax shoulderLeft;
  private final CANSparkMax shoulderRight;
  private final CANSparkMax elbowLeft;
  private final CANSparkMax elbowRight;
  private final CANSparkMax wristLeft;
  private final CANSparkMax wristRight;
  
  private final SparkMaxPIDController shoulderLeftPID;
  private final SparkMaxPIDController shoulderRightPID;
  private final SparkMaxPIDController elbowLeftPID;
  private final SparkMaxPIDController elbowRightPID;
  private final SparkMaxPIDController wristLeftPID;

  private final RelativeEncoder shoulderLeftRelativeEncoder;
  private final RelativeEncoder shoulderRightRelativeEncoder;
  private final RelativeEncoder elbowLeftRelativeEncoder;
  private final RelativeEncoder elbowRightRelativeEncoder;
  private final RelativeEncoder wristLeftEncoder;
  private final RelativeEncoder wristRightEncoder;

  private final DutyCycleEncoder shoulderLeftAbsoluteEncoder;
  private final DutyCycleEncoder shoulderRightEncoder; 
  private final DutyCycleEncoder elbowEncoder;
  private final DutyCycleEncoder wristEncoder;

  private final ThreeJointArmKinematics kinematics;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    shoulderLeft = new CANSparkMax(Constants.shoulderLeftConstants.motorID, MotorType.kBrushless);
    shoulderRight = new CANSparkMax(Constants.shoulderRightConstants.motorID, MotorType.kBrushless);
    elbowLeft = new CANSparkMax(Constants.elbowLeftConstants.motorID, MotorType.kBrushless);
    elbowRight = new CANSparkMax(Constants.elbowRightConstants.motorID, MotorType.kBrushless);
    wristLeft = new CANSparkMax(Constants.wristLeftConstants.motorID, MotorType.kBrushless);
    wristRight = new CANSparkMax(Constants.wristRightConstants.motorID, MotorType.kBrushless);

    shoulderLeft.restoreFactoryDefaults();
    shoulderRight.restoreFactoryDefaults();
    elbowLeft.restoreFactoryDefaults();
    elbowRight.restoreFactoryDefaults();
    wristLeft.restoreFactoryDefaults();
    wristRight.restoreFactoryDefaults();

    shoulderLeft.setSmartCurrentLimit(Constants.shoulderLeftConstants.currentLimit);
    shoulderRight.setSmartCurrentLimit(Constants.shoulderRightConstants.currentLimit);
    elbowLeft.setSmartCurrentLimit(Constants.elbowLeftConstants.currentLimit);
    elbowRight.setSmartCurrentLimit(Constants.elbowRightConstants.currentLimit);
    wristLeft.setSmartCurrentLimit(Constants.wristLeftConstants.currentLimit);
    wristRight.setSmartCurrentLimit(Constants.wristRightConstants.currentLimit);

    shoulderLeft.setIdleMode(IdleMode.kBrake);
    shoulderRight.setIdleMode(IdleMode.kBrake);
    elbowLeft.setIdleMode(IdleMode.kBrake);
    elbowRight.setIdleMode(IdleMode.kBrake);
    wristLeft.setIdleMode(IdleMode.kBrake);
    wristRight.setIdleMode(IdleMode.kBrake);

    shoulderLeft.setControlFramePeriodMs(40);
    shoulderRight.setControlFramePeriodMs(40);
    elbowLeft.setControlFramePeriodMs(40);
    elbowRight.setControlFramePeriodMs(40);
    wristLeft.setControlFramePeriodMs(40);
    wristRight.setControlFramePeriodMs(40);

    shoulderLeft.setInverted(Constants.shoulderLeftConstants.isInverted);
    shoulderRight.setInverted(Constants.shoulderRightConstants.isInverted);
    elbowLeft.setInverted(Constants.elbowLeftConstants.isInverted);
    elbowRight.setInverted(Constants.elbowRightConstants.isInverted);
    wristLeft.setInverted(Constants.wristLeftConstants.isInverted);

    wristRight.follow(wristLeft, Constants.wristRightConstants.isInverted);

    shoulderLeftPID = shoulderLeft.getPIDController();
    shoulderRightPID = shoulderRight.getPIDController();
    elbowLeftPID = elbowLeft.getPIDController();
    elbowRightPID = elbowRight.getPIDController();
    wristLeftPID = wristLeft.getPIDController();

    shoulderLeftRelativeEncoder = shoulderLeft.getEncoder();
    shoulderRightRelativeEncoder = shoulderRight.getEncoder();
    elbowLeftRelativeEncoder = elbowLeft.getEncoder();
    elbowRightRelativeEncoder = elbowRight.getEncoder();
    wristLeftEncoder = wristLeft.getEncoder();
    wristRightEncoder = wristRight.getEncoder();

    shoulderLeftAbsoluteEncoder = new DutyCycleEncoder(Constants.shoulderLeftEncoder);
    shoulderRightEncoder = new DutyCycleEncoder(Constants.shoulderRightEncoder);
    elbowEncoder = new DutyCycleEncoder(Constants.elbowLeftEncoder);
    wristEncoder = new DutyCycleEncoder(Constants.wristEncoder);

    shoulderLeftAbsoluteEncoder.setDistancePerRotation(360);
    shoulderRightEncoder.setDistancePerRotation(360);
    elbowEncoder.setDistancePerRotation(360);
    wristEncoder.setDistancePerRotation(360);

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
    Rotation2d currentQ1 = Rotation2d.fromDegrees(shoulderLeftRelativeEncoder.getPosition() * 360 * Constants.shoulderLeftConstants.ratio);
    Rotation2d currentQ2 = Rotation2d.fromDegrees(elbowLeftRelativeEncoder.getPosition() * 360 * Constants.elbowLeftConstants.ratio);
    Rotation2d currentQ3 = Rotation2d.fromDegrees(wristLeftEncoder.getPosition() * 360 * Constants.wristLeftConstants.ratio).plus(currentQ2).plus(currentQ1);

    Pose2d currentPose = kinematics.toPose2d(new ThreeJointArmState(currentQ1, currentQ2, currentQ3));
    SmartDashboard.putNumber("/arm/pose/x", currentPose.getX());
    SmartDashboard.putNumber("/arm/pose/y", currentPose.getY());
    SmartDashboard.putNumber("/arm/pose/angle", currentPose.getRotation().getDegrees());

    // TODO: here we should put the current state into the setPoints map to get the next target. If we are in 
    // fudge mode, we will disregard these and set the speed and direction of the motors directly based on the operator joysticks.
    // the state also has a variable to track the elbow vs wrist movement.
    // ThreeJointArmState targetArmState = kinematics.toArmState(targetPos);
    
    SmartDashboard.putNumber("Left 1 encoder", shoulderLeftAbsoluteEncoder.getDistance());
    SmartDashboard.putNumber("Right 1 encoder", shoulderRightEncoder.getDistance());
    SmartDashboard.putNumber("Left 2 Encoder", elbowEncoder.getDistance());
    SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getDistance());
  }

  public void updateSideLeft() {
    state.side = ArmState.Side.LEFT;
    state.altpickupl = false;
  }

  public void updateSideRight() {
    state.side = ArmState.Side.RIGHT;
    state.altpickupl = false;
  }

  public void updateHeightHigh() {
    state.height = ArmState.Height.HIGH;
    state.altpickupl = false;
  }

  public void updateHeightMedium() {
    state.height = ArmState.Height.MEDIUM;
    state.altpickupl = false;
  }

  public void updateHeightLow() {
    state.height = ArmState.Height.LOW;
    state.altpickupl = false;
  }

  public void updateHeightPickup() {
    state.height = ArmState.Height.HPPICKUP;
    state.altpickupl = false;
  }

  public void updateGamePieceCube() {
    state.piece = ArmState.GamePiece.CUBE;
    state.altpickupl = false;
  }

  public void updateGamePieceCone() {
    state.piece = ArmState.GamePiece.CONE;
    state.altpickupl = false;
  }

  public void updateAltPickuplTrue() {
    state.altpickupl = true;
  }

  public void updateFudgeJoint(boolean isFudgeElbow) {
    state.isFudgeElbow = isFudgeElbow;
  }
}

