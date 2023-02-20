// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.kinematics.ThreeJointArmKinematics;
import frc.robot.kinematics.ThreeJointArmState;
import frc.robot.kinematics.constraint.CircleConstraint;
import frc.robot.utils.SparkMaxAbsoluteControlHelper;

public class ArmSubsystem extends SubsystemBase {
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

  // private Pose2d targetPos = new Pose2d();

  // private final CANSparkMax intakePivotLeft;
  // private final CANSparkMax intakePivotRight;

  private final TalonFX arm1Left;
  private final TalonFX arm1Right;
  private final TalonFX arm2Left;
  private final TalonFX arm2Right;

  // private final SparkMaxPIDController intakePivotLeftPID;

  // private final RelativeEncoder intakePivotLeftEncoder;
  // private final RelativeEncoder intakePivotRightEncoder;

  private final DutyCycleEncoder arm1LeftAbsoluteEncoder;
  private final DutyCycleEncoder arm1RightAbsoluteEncoder; 
  private final DutyCycleEncoder arm2AbsoluteEncoder;
  // private final DutyCycleEncoder intakePivotEncoder;

  // private final ThreeJointArmKinematics kinematics;

  private double arm1TargetPosition = 0;
  private double arm2TargetPosition = 0;

  private final Timer timer = new Timer();

  private double previousTime = 0;
  private double previousLoopTime = 0;

  private double arm1LeftAdjustedCalibration;
  private double arm2AdjustedCalibration;

  private double arm1kP = 0.1;
  private double arm2kP = 0.1;

  // arm 1 left limit 1 : 39 degrees (at back, decreases going to front), zero at -17
  // arm 1 right limit 1 : 229 degrees (at back, increases going to front), zero at 284

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    arm1Left = new TalonFX(Constants.arm1LeftConstants.motorID);
    arm1Right = new TalonFX(Constants.arm1RightConstants.motorID);
    arm2Left = new TalonFX(Constants.arm2LeftConstants.motorID);
    arm2Right = new TalonFX(Constants.arm2RightConstants.motorID);

    arm1Left.configFactoryDefault();
    arm1Right.configFactoryDefault();
    arm2Left.configFactoryDefault();
    arm2Right.configFactoryDefault();

    arm1Left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.arm1LeftConstants.currentLimit, Constants.arm1LeftConstants.currentLimit, 0.03));
    arm1Right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.arm1RightConstants.currentLimit, Constants.arm1RightConstants.currentLimit, 0.03));
    arm2Left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.arm2LeftConstants.currentLimit, Constants.arm2LeftConstants.currentLimit, 0.03));
    arm2Right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.arm2RightConstants.currentLimit, Constants.arm2RightConstants.currentLimit, 0.03));

    arm1Left.setNeutralMode(NeutralMode.Brake);
    arm1Right.setNeutralMode(NeutralMode.Brake);
    arm2Left.setNeutralMode(NeutralMode.Brake);
    arm2Right.setNeutralMode(NeutralMode.Brake);

    arm1Left.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    arm1Right.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    arm2Left.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    arm2Right.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);

    arm1Left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    arm1Right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    arm2Left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    arm2Right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    arm1Left.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    arm1Right.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    arm2Left.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    arm2Right.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);

    arm1Left.configVelocityMeasurementWindow(4);
    arm1Right.configVelocityMeasurementWindow(4);
    arm2Left.configVelocityMeasurementWindow(4);
    arm2Right.configVelocityMeasurementWindow(4);

    arm1Left.configVoltageCompSaturation(12);
    arm1Right.configVoltageCompSaturation(12);
    arm2Left.configVoltageCompSaturation(12);
    arm2Right.configVoltageCompSaturation(12);

    arm1Left.enableVoltageCompensation(true);
    arm1Right.enableVoltageCompensation(true);
    arm2Left.enableVoltageCompensation(true);
    arm2Right.enableVoltageCompensation(true);

    arm1Left.config_kP(0, 0.1);
    arm1Left.config_kI(0, 0);
    arm1Left.config_kD(0, 0);

    arm1Right.follow(arm1Left);
    arm1Right.setInverted(InvertType.OpposeMaster);

    arm2Left.config_kP(0, 0.1);
    arm2Left.config_kI(0, 0);
    arm2Left.config_kD(0, 0);
    
    arm2Right.follow(arm2Left);
    arm2Right.setInverted(InvertType.OpposeMaster);

    // intakePivotLeft = new CANSparkMax(Constants.intakePivotLeftConstants.motorID, MotorType.kBrushless);
    // intakePivotRight = new CANSparkMax(Constants.intakePivotRightConstants.motorID, MotorType.kBrushless);

    // intakePivotLeft.restoreFactoryDefaults();
    // intakePivotRight.restoreFactoryDefaults();

    // intakePivotLeft.setSmartCurrentLimit(Constants.intakePivotLeftConstants.currentLimit);
    // intakePivotRight.setSmartCurrentLimit(Constants.intakePivotRightConstants.currentLimit);

    // intakePivotLeft.setIdleMode(IdleMode.kBrake);
    // intakePivotRight.setIdleMode(IdleMode.kBrake);

    // intakePivotLeft.setControlFramePeriodMs(40);
    // intakePivotRight.setControlFramePeriodMs(40);

    // intakePivotLeft.setInverted(Constants.intakePivotLeftConstants.isInverted);

    // intakePivotRight.follow(intakePivotLeft, Constants.intakePivotRightConstants.isInverted);

    // intakePivotLeftPID = intakePivotLeft.getPIDController();

    // intakePivotLeftEncoder = intakePivotLeft.getEncoder();
    // intakePivotRightEncoder = intakePivotRight.getEncoder();

    arm1LeftAbsoluteEncoder = new DutyCycleEncoder(Constants.arm1LeftEncoder);
    arm1RightAbsoluteEncoder = new DutyCycleEncoder(Constants.arm1RightEncoder);
    arm2AbsoluteEncoder = new DutyCycleEncoder(Constants.arm2LeftEncoder);
    // intakePivotEncoder = new DutyCycleEncoder(Constants.intakePivotEncoder);

    arm1LeftAbsoluteEncoder.setDistancePerRotation(360);
    arm1RightAbsoluteEncoder.setDistancePerRotation(360);
    arm2AbsoluteEncoder.setDistancePerRotation(360);
    // intakePivotEncoder.setDistancePerRotation(360);

    // kinematics = new ThreeJointArmKinematics(Constants.ArmConstants.a1, Constants.ArmConstants.a2, Constants.ArmConstants.a3);
    // kinematics.addConstraint(
    //   new CircleConstraint(
    //     (rotation) -> Constants.ArmConstants.a3 * rotation.getCos(), 
    //     (rotation) -> Constants.ArmConstants.a3 * rotation.getSin(), 
    //     Constants.ArmConstants.a1 + Constants.ArmConstants.a2, 
    //     true
    //   )
    // );
  
    arm1LeftAdjustedCalibration = Constants.arm1LeftConstants.calibration - 180;
    while (arm1LeftAdjustedCalibration < 0) {
      arm1LeftAdjustedCalibration += 360;
    }

    arm2AdjustedCalibration = Constants.arm2LeftConstants.calibration - 180;
    while (arm2AdjustedCalibration < 0) {
      arm2AdjustedCalibration += 360;
    }

    resetArm1LeftEncoder();
    // resetArm1RightEncoder();
    resetArm2Encoder();

    SmartDashboard.setDefaultNumber("Arm 1 kP", arm1kP);
    SmartDashboard.setDefaultNumber("Arm 2 kP", arm2kP);

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Rotation2d currentQ1 = Rotation2d.fromDegrees(arm1LeftRelativeEncoder.getPosition() * 360 * Constants.arm1LeftConstants.ratio);
    // Rotation2d currentQ2 = Rotation2d.fromDegrees(arm2LeftRelativeEncoder.getPosition() * 360 * Constants.arm2LeftConstants.ratio);
    // Rotation2d currentQ3 = Rotation2d.fromDegrees(intakePivotLeftEncoder.getPosition() * 360 * Constants.intakePivotLeftConstants.ratio).plus(currentQ2).plus(currentQ1);

    // Pose2d currentPose = kinematics.toPose2d(new ThreeJointArmState(currentQ1, currentQ2, currentQ3));
    // SmartDashboard.putNumber("/arm/pose/x", currentPose.getX());
    // SmartDashboard.putNumber("/arm/pose/y", currentPose.getY());
    // SmartDashboard.putNumber("/arm/pose/angle", currentPose.getRotation().getDegrees());

    // ThreeJointArmState targetArmState = kinematics.toArmState(targetPos);
    
    SmartDashboard.putNumber("Left 1 absolute encoder", getArm1LeftAdjustedAbsoluteEncoderPos());
    SmartDashboard.putNumber("Right 1 absolute encoder", getArm1RightAbsoluteEncoderPos());

    SmartDashboard.putNumber("Left 1 calculated relative pos", (-(getArm1LeftAdjustedAbsoluteEncoderPos() - arm1LeftAdjustedCalibration) / Constants.arm1LeftConstants.ratio / 360.0) * 2048);

    SmartDashboard.putNumber("Left 2 absolute encoder", getArm2AbsoluteEncoderPos());

    SmartDashboard.putNumber("Left 2 calculated relative pos", ((getArm2LeftAdjustedAbsoluteEncoderPos() - arm2AdjustedCalibration) / Constants.arm2LeftConstants.ratio / 360.0) * 2048);

    // SmartDashboard.putNumber("Left 2 Encoder", arm2Encoder.getDistance());
    // SmartDashboard.putNumber("Wrist Encoder", intakePivotEncoder.getDistance());

    //arm1LeftPID.setReference(arm1TargetPosition, ControlType.kPosition);

    previousLoopTime = timer.get() - previousTime;
    previousTime = timer.get();

    double arm1p = SmartDashboard.getNumber("Arm 1 kP", 0);
    double arm2p = SmartDashboard.getNumber("Arm 2 kP", 0);

    if(arm1p != arm1kP) {
      arm1Left.config_kP(0, arm1p); 
      // arm1RightPID.setP(arm1p);
      arm1kP = arm1p;
    }

    if(arm2p != arm2kP) {
      arm2Left.config_kP(0, arm2p); 
      // arm1RightPID.setP(arm1p);
      arm2kP = arm2p;
    }

    // TODO: here we should put the current state into the setPoints map to get the next target. If we are in 
    // fudge mode, we will disregard these and set the speed and direction of the motors directly based on the operator joysticks.
    // the state also has a variable to track the elbow vs wrist movement.
  }

  public void stop() {
    arm1Left.set(ControlMode.PercentOutput, 0.0);
    arm1Right.set(ControlMode.PercentOutput, 0.0);
  }

  public void moveArm1Setpoint(double amountToMove) {
    arm1TargetPosition += amountToMove;
  }

  public void setArm1Setpoint(double setpoint) {
    arm1TargetPosition = setpoint;
  }

  public void moveArm1WithAnalogStick(double analogStickValue) {
    moveArm1Setpoint(analogStickValue * previousLoopTime * 5.0 * 2048);
  }

  public void moveArm2Setpoint(double amountToMove) {
    arm2TargetPosition += amountToMove;
  }

  public void setArm2Setpoint(double setpoint) {
    arm2TargetPosition = setpoint;
  }

  public void moveArm2WithAnalogStick(double analogStickValue) {
    moveArm2Setpoint(analogStickValue * previousLoopTime * 4.0 * 2048);
  }

  private double getArm1LeftAdjustedAbsoluteEncoderPos() {
    double adjustedPos = arm1LeftAbsoluteEncoder.getDistance() - 180;
    while (adjustedPos < 0) {
      adjustedPos += 360;
    }
    return adjustedPos;
  }

  private double getArm2LeftAdjustedAbsoluteEncoderPos() {
    double adjustedPos = arm2AbsoluteEncoder.getDistance() - 180;
    while (adjustedPos < 0) {
      adjustedPos += 360;
    }
    return adjustedPos;
  }

  private double getArm1RightAbsoluteEncoderPos() {
    return Math.abs(arm1RightAbsoluteEncoder.getDistance());
  }

  private double getArm2AbsoluteEncoderPos() {
    return arm2AbsoluteEncoder.getDistance();
  }

  private double getArm1AbsoluteEncoderAverage() {
    return (-(getArm1LeftAdjustedAbsoluteEncoderPos() - arm1LeftAdjustedCalibration)
    + (getArm1RightAbsoluteEncoderPos() - Constants.arm1RightConstants.calibration)
    ) / 2;
  }

  private void resetArm1LeftEncoder() {
    arm1Left.setSelectedSensorPosition((getArm1AbsoluteEncoderAverage() / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
  }

  private void resetArm2Encoder() {
    arm2Left.setSelectedSensorPosition(((getArm2LeftAdjustedAbsoluteEncoderPos() - arm2AdjustedCalibration) / Constants.arm2LeftConstants.ratio / 360.0) * 2048.0);
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
