// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.ArmConstants.SetPoint;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Height;
import frc.robot.utils.LazyTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private ArmState state = new ArmState(Height.SCORE_LOW, GamePiece.CONE, false);
  private boolean deploy = false;

  private final CANSparkMax wristLeft;
  private final CANSparkMax wristRight;

  private final LazyTalonFX shoulderLeft;
  private final LazyTalonFX shoulderRight;
  private final LazyTalonFX elbowLeft;
  private final LazyTalonFX elbowRight;

  private final SparkMaxPIDController wristPID;

  private final RelativeEncoder wristLeftEncoder;

  private final DutyCycleEncoder shoulderLeftAbsoluteEncoder;
  private final DutyCycleEncoder shoulderRightAbsoluteEncoder; 
  private final DutyCycleEncoder elbowAbsoluteEncoder;
  private final DutyCycleEncoder wristAbsoluteEncoder;

  private final double armRetractPosition = 0;

  private double shoulderTargetPosition = 0;
  private double elbowTargetPosition = 0;
  private double wristTargetPosition = 0;

  private final Timer timer = new Timer();

  private double previousTime = 0;
  private double previousLoopTime = 0;

  private double shoulderLeftAdjustedCalibration;
  private double shoulderRightAdjustedCalibration;
  private double elbowAdjustedCalibration;
  private double wristAdjustedCalibration;

  private boolean shoulderLeftEncoderEnabled = true;
  private boolean shoulderRightEncoderEnabled = true;
  private boolean elbowEncoderEnabled = true;
  private boolean wristEncoderEnabled = true;

  private double shoulderLeftEncoderPreviousValue = 0;
  private double shoulderRightEncoderPreviousValue = 0;
  private double elbowEncoderPreviousValue = 0;
  private double wristEncoderPreviousValue = 0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    shoulderLeft = new LazyTalonFX(Constants.shoulderLeft);
    shoulderRight = new LazyTalonFX(Constants.shoulderRight);
    elbowLeft = new LazyTalonFX(Constants.elbowLeft);
    elbowRight = new LazyTalonFX(Constants.elbowRight);

    shoulderLeft.configFactoryDefault();
    shoulderRight.configFactoryDefault();
    elbowLeft.configFactoryDefault();
    elbowRight.configFactoryDefault();

    shoulderLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    shoulderRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    elbowLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 0.03));
    elbowRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 0.03));

    shoulderLeft.setNeutralMode(NeutralMode.Brake);
    shoulderRight.setNeutralMode(NeutralMode.Brake);
    elbowLeft.setNeutralMode(NeutralMode.Brake);
    elbowRight.setNeutralMode(NeutralMode.Brake);

    shoulderLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    shoulderRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    elbowLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    elbowRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);

    shoulderLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shoulderRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elbowLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elbowRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    shoulderLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    shoulderRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    elbowLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    elbowRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);

    shoulderLeft.configVelocityMeasurementWindow(4);
    shoulderRight.configVelocityMeasurementWindow(4);
    elbowLeft.configVelocityMeasurementWindow(4);
    elbowRight.configVelocityMeasurementWindow(4);

    shoulderLeft.configVoltageCompSaturation(12);
    shoulderRight.configVoltageCompSaturation(12);
    elbowLeft.configVoltageCompSaturation(12);
    elbowRight.configVoltageCompSaturation(12);

    shoulderLeft.enableVoltageCompensation(true);
    shoulderRight.enableVoltageCompensation(true);
    elbowLeft.enableVoltageCompensation(true);
    elbowRight.enableVoltageCompensation(true);

    shoulderLeft.configForwardSoftLimitThreshold(40000);
    elbowLeft.configForwardSoftLimitThreshold(0);

    shoulderLeft.configReverseSoftLimitThreshold(-30000);
    elbowLeft.configReverseSoftLimitThreshold((-135.0 / (Constants.ArmConstants.elbowBaseRatio * Constants.ArmConstants.elbowChainRunRatio) / 360.0) * 2048.0);

    shoulderLeft.configForwardSoftLimitEnable(true);
    elbowLeft.configForwardSoftLimitEnable(true);
    
    shoulderLeft.configReverseSoftLimitEnable(true);
    elbowLeft.configReverseSoftLimitEnable(true);

    shoulderLeft.config_kP(0, 0.03);
    shoulderLeft.config_kI(0, 0);
    shoulderLeft.config_kD(0, 0.006);
    shoulderLeft.configNeutralDeadband(0.05);

    shoulderRight.follow(shoulderLeft);
    shoulderRight.setInverted(InvertType.OpposeMaster);

    elbowLeft.config_kP(0, 0.015); // 0.15
    elbowLeft.config_kI(0, 0);
    elbowLeft.config_kD(0, 0.015);
    elbowLeft.configNeutralDeadband(0.06);
    
    elbowRight.follow(elbowLeft);
    elbowRight.setInverted(InvertType.OpposeMaster);

    wristLeft = new CANSparkMax(Constants.wristLeft, MotorType.kBrushless);
    wristRight = new CANSparkMax(Constants.wristRight, MotorType.kBrushless);

    wristLeft.restoreFactoryDefaults();
    wristRight.restoreFactoryDefaults();

    wristLeft.setSmartCurrentLimit(20);
    wristRight.setSmartCurrentLimit(20);

    wristLeft.setIdleMode(IdleMode.kBrake);
    wristRight.setIdleMode(IdleMode.kBrake);

    wristLeft.setControlFramePeriodMs(40);
    wristRight.setControlFramePeriodMs(40);

    wristLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
    wristLeft.setSoftLimit(SoftLimitDirection.kForward, (float) (135.0 / Constants.ArmConstants.wristRatio / 360.0));

    wristLeft.setInverted(true);
    wristRight.follow(wristLeft, true);

    wristPID = wristLeft.getPIDController();

    wristPID.setP(0.075);
    wristPID.setD(0.0);
    wristPID.setOutputRange(-0.5, 0.5);

    wristLeftEncoder = wristLeft.getEncoder();

    shoulderLeftAbsoluteEncoder = new DutyCycleEncoder(Constants.shoulderLeftEncoder);
    shoulderRightAbsoluteEncoder = new DutyCycleEncoder(Constants.shoulderRightEncoder);
    elbowAbsoluteEncoder = new DutyCycleEncoder(Constants.elbowLeftEncoder);
    wristAbsoluteEncoder = new DutyCycleEncoder(Constants.wristEncoder);

    shoulderLeftAbsoluteEncoder.setDistancePerRotation(360);
    shoulderRightAbsoluteEncoder.setDistancePerRotation(360);
    elbowAbsoluteEncoder.setDistancePerRotation(360);
    wristAbsoluteEncoder.setDistancePerRotation(360);
  
    shoulderLeftAdjustedCalibration = Constants.ArmConstants.shoulderLeftCalibration - 180;
    while (shoulderLeftAdjustedCalibration < 0) {
      shoulderLeftAdjustedCalibration += 360;
    }

    shoulderRightAdjustedCalibration = Constants.ArmConstants.shoulderRightCalibration - 180;
    while (shoulderRightAdjustedCalibration < 0) {
      shoulderRightAdjustedCalibration += 360;
    }

    elbowAdjustedCalibration = Constants.ArmConstants.elbowCalibration - 180;
    while (elbowAdjustedCalibration < 0) {
      elbowAdjustedCalibration += 360;
    }

    wristAdjustedCalibration = Constants.ArmConstants.wristCalibration - 180;
    while (wristAdjustedCalibration < 0) {
      wristAdjustedCalibration += 360;
    }

    // There needs to be a chillout time for the wrist motor controllers to reset their encoders properly
    try {
      Thread.sleep(2000);
    } catch (Exception e) {

    }

    resetEncoders();

    // There needs to be a chillout time for the wrist motor controllers to reset their encoders properly
    try {
      Thread.sleep(500);
    } catch (Exception e) {

    }

    setShoulderSetpoint(shoulderLeft.getSelectedSensorPosition());
    setElbowSetpoint(elbowLeft.getSelectedSensorPosition());
    setWristSetpoint(wristLeftEncoder.getPosition());

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    previousLoopTime = timer.get() - previousTime;
    previousTime = timer.get();

    // Disable absolute encoders if they're value isn't changing (The value constantly fluxuates, so an unchanging value is a red flag)
    if (shoulderLeftEncoderPreviousValue == getShoulderLeftAdjustedAbsoluteEncoderPos()) shoulderLeftEncoderEnabled = false;
    if (shoulderRightEncoderPreviousValue == getShoulderRightAbsoluteEncoderPos()) shoulderRightEncoderEnabled = false;
    if (elbowEncoderPreviousValue == getElbowAbsoluteEncoderPos()) elbowEncoderEnabled = false;
    if (wristEncoderPreviousValue == getWristAbsoluteEncoderPos()) wristEncoderEnabled = false;

    if (state.piece == GamePiece.CONE && state.height == Height.SCORE_HIGH && Math.abs(elbowTargetPosition - elbowLeft.getSelectedSensorPosition()) > 50000) {
      shoulderTargetPosition = shoulderLeft.getSelectedSensorPosition();
    }

    if (!deploy) {
      setShoulderSetpoint(-12000);
      setElbowSetpoint(-7000);
      setWristSetpoint(armRetractPosition);
    }
    
    if (Math.abs(getElbowCompensatedAbsoluteEncoderPos()) < 10) {
      shoulderTargetPosition = Math.min(shoulderTargetPosition, 2.0 * Constants.ArmConstants.shoulderRatio * 2048.0 / 360.0);
    }

    if (Math.abs(getShoulderAbsoluteEncoderAverage()) < 15 && Math.abs(getElbowCompensatedAbsoluteEncoderPos()) < 20) { // 20, 30
      wristTargetPosition = armRetractPosition;
    } else if (
      Math.abs(getShoulderAbsoluteEncoderAverage()) < 30 
      && Math.abs(getElbowCompensatedAbsoluteEncoderPos()) < 40 
      && Math.abs(elbowLeft.getSelectedSensorVelocity() * 10 / 2048.0) > 2000 // Falcon is spinning at 2000 rpm or faster
    ) {
      wristTargetPosition = armRetractPosition;
    }

    wristTargetPosition = MathUtil.clamp(wristTargetPosition, 0, 50);

    shoulderLeft.set(ControlMode.Position, shoulderTargetPosition);
    elbowLeft.set(ControlMode.Position, elbowTargetPosition);
    wristPID.setReference(wristTargetPosition, ControlType.kPosition);

    SmartDashboard.putNumber("Shoulder Left Adjusted Calibration", shoulderLeftAdjustedCalibration);
    SmartDashboard.putNumber("Shoulder Right Adjusted Calibration", shoulderRightAdjustedCalibration);
    
    SmartDashboard.putNumber("Shoulder left absolute encoder", getShoulderLeftAbsoluteEncoderPos());
    SmartDashboard.putNumber("Shoulder right absolute encoder", getShoulderRightAbsoluteEncoderPos());
    SmartDashboard.putNumber("Shoulder absolute pos", getShoulderAbsoluteEncoderAverage());

    SmartDashboard.putNumber("Shoulder calculated relative pos", (-(getShoulderLeftAdjustedAbsoluteEncoderPos() - shoulderLeftAdjustedCalibration) / Constants.ArmConstants.shoulderRatio / 360.0) * 2048.0);

    SmartDashboard.putNumber("Shoulder relative pos", shoulderLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder target position", shoulderTargetPosition);

    SmartDashboard.putNumber("Elbow absolute encoder", getElbowAbsoluteEncoderPos());

    SmartDashboard.putNumber("Shoulder applied power", shoulderLeft.getMotorOutputPercent());

    SmartDashboard.putNumber("Elbow calculated relative pos", (getElbowCompensatedAbsoluteEncoderPos() / (Constants.ArmConstants.elbowBaseRatio * Constants.ArmConstants.elbowChainRunRatio) / 360.0) * 2048.0);
    SmartDashboard.putNumber("Elbow relative pos", elbowLeft.getSelectedSensorPosition());

    SmartDashboard.putNumber("Elbow calibrated absolute pos", getElbowAbsoluteEncoderPos() - Constants.ArmConstants.elbowCalibration);
    SmartDashboard.putNumber("Elbow compensated angle", getElbowCompensatedAbsoluteEncoderPos());
    SmartDashboard.putNumber("Elbow absolute pos", elbowAbsoluteEncoder.getDistance());

    SmartDashboard.putNumber("Elbow target position", elbowTargetPosition);

    SmartDashboard.putNumber("Elbow applied power", elbowLeft.getMotorOutputPercent());

    SmartDashboard.putNumber("Wrist absolute pos", wristAbsoluteEncoder.getDistance());
    SmartDashboard.putNumber("Wrist relative pos", wristLeftEncoder.getPosition());
    SmartDashboard.putNumber("Wrist calculated relative pos", getWristCalibratedAbsoluteEncoderPos() / Constants.ArmConstants.wristRatio / 360.0);
    SmartDashboard.putNumber("Wrist target pos", wristTargetPosition);
    SmartDashboard.putNumber("Wrist calibrated absolute pos", getWristCalibratedAbsoluteEncoderPos());
    SmartDashboard.putNumber("Wrist applied power", wristLeft.get());

    SmartDashboard.putBoolean("Arm target setpoint is cube", state.piece == GamePiece.CUBE);
    SmartDashboard.putString("Arm target setpoint height", state.height.name());
    SmartDashboard.putBoolean("Arm target setpoint is in fudge mod", state.fudge);
    SmartDashboard.putBoolean("Arm should be deployed", deploy);

    SmartDashboard.putBoolean("Shoulder left encoder enabled", shoulderLeftEncoderEnabled);
    SmartDashboard.putBoolean("Shoulder right encoder enabled", shoulderRightEncoderEnabled);
    SmartDashboard.putBoolean("Elbow encoder enabled", elbowEncoderEnabled);
    SmartDashboard.putBoolean("Wrist encoder enabled", wristEncoderEnabled);
  }

  public void stop() {
    shoulderLeft.set(ControlMode.PercentOutput, 0.0);
    elbowLeft.set(ControlMode.PercentOutput, 0.0);
    wristLeft.stopMotor();
  }

  public void setpointsFromStateMachine() {
    try{
      SetPoint target = Constants.ArmConstants.setPoints.get(state);
      setShoulderSetpoint(target.shoulderPos);
      setElbowSetpoint(target.elbowPos);
      setWristSetpoint(target.wristPos);
    } catch (Exception e) {
      System.err.println(e);
    }
  }

  public void setShoulderSetpoint(double setpoint) {
    shoulderTargetPosition = setpoint;
  }

  public void fudgeShoulderSetpoint(double amountToMove) {
    shoulderTargetPosition += amountToMove;
  }

  public void fudgeShoulderWithAnalogStick(double analogStickValue) {
    fudgeShoulderSetpoint(analogStickValue * previousLoopTime * 8.0 * 2048.0);
  }

  public void setElbowSetpoint(double setpoint) {
    elbowTargetPosition = setpoint;
  }

  public void fudgeElbowSetpoint(double amountToMove) {
    elbowTargetPosition += amountToMove;
  }

  public void fudgeElbowWithAnalogStick(double analogStickValue) {
    fudgeElbowSetpoint(analogStickValue * previousLoopTime * 24.0 * 2048.0);
  }

  public void setWristSetpoint(double setpoint) {
    wristTargetPosition = setpoint;
  }

  public void fudgeWristSetpoint(double amountToMove) {
    wristTargetPosition += amountToMove;
  }

  public void fudgeWristWithAnalogStick(double analogStickValue) {
    fudgeWristSetpoint(analogStickValue * previousLoopTime * 16.0);
  }

  public void resetEncoders() {
    resetShoulderLeftEncoder();
    resetElbowEncoder();
    resetWristEncoder();
  }

  private double getShoulderLeftAdjustedAbsoluteEncoderPos() {
    double adjustedPos = shoulderLeftAbsoluteEncoder.getDistance() - 180.0;
    while (adjustedPos < 0) {
      adjustedPos += 360.0;
    }
    return adjustedPos;
  }

  private double getShoulderLeftAbsoluteEncoderPos() {
    return shoulderLeftAbsoluteEncoder.getDistance();
  }

  private double getShoulderRightAbsoluteEncoderPos() {
    return shoulderRightAbsoluteEncoder.getDistance();
  }

  private double getShoulderRightAdjustedAbsoluteEncoderPos() {
    double adjustedPos = shoulderRightAbsoluteEncoder.getDistance() - 180.0;
    while (adjustedPos < 0) {
      adjustedPos += 360.0;
    }
    return adjustedPos;
  }

  private double getElbowAdjustedAbsoluteEncoderPos() {
    return elbowAbsoluteEncoder.getDistance();
  }

  private double getElbowAbsoluteEncoderPos() {
    return elbowAbsoluteEncoder.getDistance();
  }

  private double getElbowCompensatedAbsoluteEncoderPos() {
    return getElbowAdjustedAbsoluteEncoderPos() - Constants.ArmConstants.elbowCalibration + getShoulderAbsoluteEncoderAverage() * Constants.ArmConstants.elbowChainRunRatio;
  }

  private double getShoulderAbsoluteEncoderAverage() {
    // return (-(getShoulderLeftAdjustedAbsoluteEncoderPos() - shoulderLeftAdjustedCalibration)
    // + (getShoulderRightAbsoluteEncoderPos() - Constants.ArmConstants.shoulderRightCalibration)
    // ) / 2;
    return (-(getShoulderLeftAdjustedAbsoluteEncoderPos() - shoulderLeftAdjustedCalibration)
    + (getShoulderRightAdjustedAbsoluteEncoderPos() - shoulderRightAdjustedCalibration)
    ) / 2;
  }

  private double getWristAbsoluteEncoderPos() {
    return wristAbsoluteEncoder.getDistance();
  }

  private double getWristCalibratedAbsoluteEncoderPos() {
    return getWristAbsoluteEncoderPos() - Constants.ArmConstants.wristCalibration;
  }

  private void resetShoulderLeftEncoder() {
    if (shoulderLeftEncoderEnabled && shoulderRightEncoderEnabled) shoulderLeft.setSelectedSensorPosition((getShoulderAbsoluteEncoderAverage() / Constants.ArmConstants.shoulderRatio / 360.0) * 2048.0);
    else if (shoulderLeftEncoderEnabled && ! shoulderRightEncoderEnabled) shoulderLeft.setSelectedSensorPosition((-(getShoulderLeftAdjustedAbsoluteEncoderPos() - shoulderLeftAdjustedCalibration) / Constants.ArmConstants.shoulderRatio / 360.0) * 2048.0);
    else if (!shoulderLeftEncoderEnabled && shoulderRightEncoderEnabled) shoulderLeft.setSelectedSensorPosition(((getShoulderRightAbsoluteEncoderPos() - Constants.ArmConstants.shoulderRightCalibration) / Constants.ArmConstants.shoulderRatio / 360.0) * 2048.0);
  }

  private void resetElbowEncoder() {
    if (elbowEncoderEnabled) elbowLeft.setSelectedSensorPosition((getElbowCompensatedAbsoluteEncoderPos() / (Constants.ArmConstants.elbowBaseRatio * Constants.ArmConstants.elbowChainRunRatio) / 360.0) * 2048.0);
  }

  private void resetWristEncoder() {
    if (wristEncoderEnabled) wristLeftEncoder.setPosition(getWristCalibratedAbsoluteEncoderPos() / Constants.ArmConstants.wristRatio / 360.0);
  }

  public void updateHeightScoreHigh() {
    state.height = Height.SCORE_HIGH;
  }

  public void updateHeightScoreMedium() {
    state.height = Height.SCORE_MEDIUM;
  }

  public void updateHeightScoreLow() {
    state.height = Height.SCORE_LOW;
  }

  public void updateHeightPickupShelf() {
    state.height = Height.PICKUP_SHELF;
  }

  public void updateHeightPickupChute() {
    state.height = Height.PICKUP_CHUTE;
  }

  public void updateHeightPickupGround() {
    state.height = Height.PICKUP_GROUND;
  }

  public void updateGamePieceCube() {
    state.piece = GamePiece.CUBE;
  }

  public void updateGamePieceCone() {
    state.piece = GamePiece.CONE;
  }

  public void togglePiece() {
    if (state.piece == GamePiece.CUBE) {
      state.piece = GamePiece.CONE;
    } else {
      state.piece = GamePiece.CUBE;
    }
  }

  public void updateFudgeTrue() {
    if (!state.fudge) {
      state.fudge = true;
      setShoulderSetpoint(shoulderLeft.getSelectedSensorPosition());
      setElbowSetpoint(elbowLeft.getSelectedSensorPosition());
      setWristSetpoint(wristLeftEncoder.getPosition());
    }
  }

  public void updateFudgeFalse() {
    state.fudge = false;
  }

  public void updateDeployTrue() {
    deploy = true;
  }

  public void updateDeployFalse() {
    deploy = false;
  }

  public GamePiece getPiece() {
    return state.piece;
  }

  public Height getHeight() {
    return state.height;
  }

  public boolean getFudge() {
    return state.fudge;
  }

  public boolean getDeploy() {
    return deploy;
  }
}
