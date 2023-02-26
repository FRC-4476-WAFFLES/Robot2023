// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import edu.wpi.first.math.filter.LinearFilter;
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
import frc.robot.subsystems.ArmSubsystem.ArmState.GamePiece;
import frc.robot.subsystems.ArmSubsystem.ArmState.Side;
import frc.robot.utils.LazyTalonFX;

public class ArmSubsystem extends SubsystemBase {
  public static class ArmState {
    public enum Side{
      FRONT,
      BACK,
    }

    enum Height{
      HIGH,
      MEDIUM,
      LOW, 
      HPPICKUP,
    }

    public enum GamePiece{
      CUBE,
      CONE,
    }

    public Side side;
    public Height height;
    public GamePiece piece;
    public boolean altpickupl;
    public boolean fudge;
    // public boolean isFudgeElbow;

    ArmState(Side side, Height height, GamePiece piece, boolean altpickupl, boolean fudge){
      this.side= side;
      this.height= height;
      this.piece= piece;
      this.altpickupl= altpickupl;
      this.fudge=fudge;
      // isFudgeElbow = false;
    }

    public boolean equals(ArmState o){
      return this.side==o.side && this.height==o.height && this.piece==o.piece && this.altpickupl==o.altpickupl;
    }

    @Override
    public int hashCode(){
      int temp= 0;
      temp += side.ordinal();
      temp += height.ordinal()*2;
      temp += piece.ordinal()*8;
      temp += altpickupl ? 16 : 0;
      return temp;
    }
  }

  private static class SetPoint{
    // TODO: setpointNumber is for testing purposes. Replace this with the actual values when applicable
    private final int arm1Pos;
    private final int arm2Pos;
    private final double wristPos;

    private SetPoint(int arm1Pos, int arm2Pos, double wristPos) {
      this.arm1Pos = arm1Pos;
      this.arm2Pos = arm2Pos;
      this.wristPos = wristPos;
    }
  }

  HashMap<Integer, SetPoint> setPoints = new HashMap<Integer, SetPoint>() {{
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(26500, -56000, 38));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(0, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(-27300, 53300, -33.5));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HIGH, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(3, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(21000, -46000, 38));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(5, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(-17000, 41800, -31.2));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(7, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.LOW, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(29000, -27000, -36.0)); // Done
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.LOW, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(9, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.LOW, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(10, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.LOW, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(11, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(12, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(13, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, false, false).hashCode(), new SetPoint(14, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, false, false).hashCode(), new SetPoint(15, 0, 0));

    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(16, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HIGH, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(17, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HIGH, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(18, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HIGH, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(19, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(20, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(21, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(22, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(23, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.LOW, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(24, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.LOW, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(25, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.LOW, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(26, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.LOW, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(27, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(28, 0, 0));
    put(new ArmState(ArmState.Side.FRONT, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(29, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, true, false).hashCode(), new SetPoint(30, 0, 0));
    put(new ArmState(ArmState.Side.BACK, ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, true, false).hashCode(), new SetPoint(31, 0, 0));
  }};

  private ArmState state = new ArmState(ArmState.Side.FRONT, ArmState.Height.LOW, ArmState.GamePiece.CONE, false, true);

  // private Pose2d targetPos = new Pose2d();

  private final CANSparkMax intakePivotLeft;
  private final CANSparkMax intakePivotRight;

  private final LazyTalonFX arm1Left;
  private final LazyTalonFX arm1Right;
  private final LazyTalonFX arm2Left;
  private final LazyTalonFX arm2Right;

  private final SparkMaxPIDController intakePivotPID;

  private final RelativeEncoder intakePivotLeftEncoder;

  private final DutyCycleEncoder arm1LeftAbsoluteEncoder;
  private final DutyCycleEncoder arm1RightAbsoluteEncoder; 
  private final DutyCycleEncoder arm2AbsoluteEncoder;
  private final DutyCycleEncoder intakePivotAbsoluteEncoder;

  // private final ThreeJointArmKinematics kinematics;

  private double arm1TargetPosition = 0;
  private double arm2TargetPosition = 0;
  private double intakeTargetPosition = 0;

  private final Timer timer = new Timer();

  private double previousTime = 0;
  private double previousLoopTime = 0;

  private double arm1LeftAdjustedCalibration;
  private double arm2AdjustedCalibration;
  private double intakeAdjustedCalibration;

  private final double arm2MaxFeedforward = 0.05;

  private double arm1TimeLastMoved = 0;
  private double arm2TimeLastMoved = 0;
  private double wristTimeLastMoved = 0;

  // arm 1 left limit 1 : 39 degrees (at back, decreases going to front), zero at -17
  // arm 1 right limit 1 : 229 degrees (at back, increases going to front), zero at 284
  // arm 2 chain ratio: 32:74

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    arm1Left = new LazyTalonFX(Constants.arm1Left);
    arm1Right = new LazyTalonFX(Constants.arm1Right);
    arm2Left = new LazyTalonFX(Constants.arm2Left);
    arm2Right = new LazyTalonFX(Constants.arm2Right);

    arm1Left.configFactoryDefault();
    arm1Right.configFactoryDefault();
    arm2Left.configFactoryDefault();
    arm2Right.configFactoryDefault();

    arm1Left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    arm1Right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    arm2Left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 0.03));
    arm2Right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 0.03));

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

    arm1Left.configForwardSoftLimitThreshold((40.0 / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
    arm2Left.configForwardSoftLimitThreshold((135.0 / (Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio) / 360.0) * 2048.0);

    arm1Left.configReverseSoftLimitThreshold((-40.0 / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
    arm2Left.configReverseSoftLimitThreshold((-135.0 / (Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio) / 360.0) * 2048.0);

    arm1Left.configForwardSoftLimitEnable(true);
    arm2Left.configForwardSoftLimitEnable(true);
    
    arm1Left.configReverseSoftLimitEnable(true);
    arm2Left.configReverseSoftLimitEnable(true);

    arm1Left.config_kP(0, 0.07);
    arm1Left.config_kI(0, 0);
    arm1Left.config_kD(0, 0.003);
    arm1Left.configNeutralDeadband(0.04);
    //arm1Left.configClosedloopRamp(0.25);

    arm1Right.follow(arm1Left);
    arm1Right.setInverted(InvertType.OpposeMaster);

    arm2Left.config_kP(0, 0.015); // 0.15
    arm2Left.config_kI(0, 0);
    arm2Left.config_kD(0, 0.015);
    arm2Left.configNeutralDeadband(0.06);
    //arm2Left.configClosedloopRamp(1.0);
    
    arm2Right.follow(arm2Left);
    arm2Right.setInverted(InvertType.OpposeMaster);

    intakePivotLeft = new CANSparkMax(Constants.intakePivotLeft, MotorType.kBrushless);
    intakePivotRight = new CANSparkMax(Constants.intakePivotRight, MotorType.kBrushless);

    intakePivotLeft.restoreFactoryDefaults();
    intakePivotRight.restoreFactoryDefaults();

    intakePivotLeft.setSmartCurrentLimit(20);
    intakePivotRight.setSmartCurrentLimit(20);

    intakePivotLeft.setIdleMode(IdleMode.kBrake);
    intakePivotRight.setIdleMode(IdleMode.kBrake);

    intakePivotLeft.setControlFramePeriodMs(40);
    intakePivotRight.setControlFramePeriodMs(40);

    intakePivotLeft.setSoftLimit(SoftLimitDirection.kReverse, (float) (-135.0 / Constants.intakePivotLeftConstants.ratio / 360.0));
    intakePivotLeft.setSoftLimit(SoftLimitDirection.kForward, (float) (135.0 / Constants.intakePivotLeftConstants.ratio / 360.0));

    intakePivotLeft.setInverted(true);
    intakePivotRight.follow(intakePivotLeft, true);

    intakePivotPID = intakePivotLeft.getPIDController();

    intakePivotPID.setP(0.075);
    intakePivotPID.setD(0.0);

    intakePivotLeftEncoder = intakePivotLeft.getEncoder();

    arm1LeftAbsoluteEncoder = new DutyCycleEncoder(Constants.arm1LeftEncoder);
    arm1RightAbsoluteEncoder = new DutyCycleEncoder(Constants.arm1RightEncoder);
    arm2AbsoluteEncoder = new DutyCycleEncoder(Constants.arm2LeftEncoder);
    intakePivotAbsoluteEncoder = new DutyCycleEncoder(Constants.intakePivotEncoder);

    arm1LeftAbsoluteEncoder.setDistancePerRotation(360);
    arm1RightAbsoluteEncoder.setDistancePerRotation(360);
    arm2AbsoluteEncoder.setDistancePerRotation(360);
    intakePivotAbsoluteEncoder.setDistancePerRotation(360);

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

    intakeAdjustedCalibration = Constants.intakePivotLeftConstants.calibration - 180;
    while (intakeAdjustedCalibration < 0) {
      intakeAdjustedCalibration += 360;
    }

    resetArm1LeftEncoder();
    resetArm2Encoder();
    resetIntakeEncoder();

    setArm1Setpoint(arm1Left.getSelectedSensorPosition());
    setArm2Setpoint(arm2Left.getSelectedSensorPosition());
    setIntakeSetpoint(intakePivotLeftEncoder.getPosition());

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
    
    // SmartDashboard.putNumber("Left 1 absolute encoder", getArm1LeftAdjustedAbsoluteEncoderPos());
    // SmartDashboard.putNumber("Right 1 absolute encoder", getArm1RightAbsoluteEncoderPos());
    SmartDashboard.putNumber("Arm 1 absolute pos", getArm1AbsoluteEncoderAverage());

    SmartDashboard.putNumber("Left 1 calculated relative pos", (-(getArm1LeftAdjustedAbsoluteEncoderPos() - arm1LeftAdjustedCalibration) / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);

    SmartDashboard.putNumber("Left 1 relative pos", arm1Left.getSelectedSensorPosition());

    SmartDashboard.putNumber("Left 2 absolute encoder", getArm2AbsoluteEncoderPos());

    SmartDashboard.putNumber("Left 1 Applied Power", arm1Left.getMotorOutputPercent());

    SmartDashboard.putNumber("Left 2 calculated relative pos", (getArm2LeftCompensatedAbsoluteEncoderPos() / (Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio) / 360.0) * 2048.0);
    SmartDashboard.putNumber("Left 2 relative pos", arm2Left.getSelectedSensorPosition());

    SmartDashboard.putNumber("Arm 2 calibrated absolute pos", getArm2AbsoluteEncoderPos() - Constants.arm2LeftConstants.calibration);
    SmartDashboard.putNumber("Arm 2 compensated angle", getArm2LeftCompensatedAbsoluteEncoderPos());
    SmartDashboard.putNumber("Arm 2 absolute pos", arm2AbsoluteEncoder.getDistance());

    SmartDashboard.putNumber("Arm 2 Target Position", arm2TargetPosition);

    SmartDashboard.putNumber("Left 2 Applied Power", arm2Left.getMotorOutputPercent());

    SmartDashboard.putNumber("Wrist absolute pos", intakePivotAbsoluteEncoder.getDistance());
    SmartDashboard.putNumber("Wrist relative pos", intakePivotLeftEncoder.getPosition());
    SmartDashboard.putNumber("Wrist calculated relative pos", getIntakeCalibratedAbsoluteEncoderPos() / Constants.intakePivotLeftConstants.ratio / 360.0);
    SmartDashboard.putNumber("Wrist target pos", intakeTargetPosition);
    SmartDashboard.putNumber("Wrist calibrated absolute pos", getIntakeCalibratedAbsoluteEncoderPos());

    SmartDashboard.putBoolean("Arm target setpoint is front", state.side == Side.FRONT);
    SmartDashboard.putBoolean("Arm target setpoint is cube", state.piece == GamePiece.CUBE);
    SmartDashboard.putString("Arm target setpoint height", state.height.name());
    SmartDashboard.putBoolean("Arm target setpoint is alt pickup 1", state.altpickupl);
    SmartDashboard.putBoolean("Arm target setpoint is in fudge mod", state.fudge);

    previousLoopTime = timer.get() - previousTime;
    previousTime = timer.get();

    double currentDegrees = arm2Left.getSelectedSensorPosition() * Constants.arm2LeftConstants.ratio * 360.0 / 2048.0;
    double radians = Math.toRadians(currentDegrees);
    double sineScalar = Math.sin(radians);

    SmartDashboard.putNumber("Arm 2 Sine Scalar", sineScalar);
    SmartDashboard.putNumber("Arm 2 feedforward", arm2MaxFeedforward);
    SmartDashboard.putNumber("Arm 2 applied feedforward", sineScalar * arm2MaxFeedforward);

    if (Math.abs(arm1Left.getSelectedSensorVelocity()) > 200) {
      arm1TimeLastMoved = timer.get();
    } else if (timer.get() - arm1TimeLastMoved > 1.0) {
      resetArm1LeftEncoder();
      arm1TimeLastMoved = timer.get();
    }

    if (Math.abs(arm2Left.getSelectedSensorVelocity()) > 200) {
      arm2TimeLastMoved = timer.get();
    } else if (timer.get() - arm2TimeLastMoved > 1.0) {
      resetArm2Encoder();
      arm2TimeLastMoved = timer.get();
    }

    // resetArm2Encoder();
    
    if (Math.abs(intakePivotLeftEncoder.getVelocity()) > 10) {
      wristTimeLastMoved = timer.get();
    } else if (timer.get() - wristTimeLastMoved > 1.0) {
      resetIntakeEncoder();
      wristTimeLastMoved = timer.get();
    }

    if (Math.abs(getArm1AbsoluteEncoderAverage()) < 25 && Math.abs(getArm2LeftCompensatedAbsoluteEncoderPos()) < 30) {
      intakeTargetPosition = 0;
    }

    arm1Left.set(ControlMode.Position, arm1TargetPosition);
    arm2Left.set(ControlMode.Position, arm2TargetPosition, DemandType.ArbitraryFeedForward, sineScalar * arm2MaxFeedforward);
    intakePivotPID.setReference(intakeTargetPosition, ControlType.kPosition);

    // TODO: here we should put the current state into the setPoints map to get the next target. If we are in 
    // fudge mode, we will disregard these and set the speed and direction of the motors directly based on the operator joysticks.
    // the state also has a variable to track the elbow vs wrist movement.
  }

  // public void easyRun(double power) {
  //   power = MathUtil.clamp(power, -0.3, 0.3);
  //   arm2Left.set(ControlMode.PercentOutput, power);
  // }

  public void stop() {
    arm1Left.set(ControlMode.PercentOutput, 0.0);
    arm2Left.set(ControlMode.PercentOutput, 0.0);
    intakePivotLeft.stopMotor();
  }

  public void setpointsFromStateMachine() {
    try{
      SetPoint target = setPoints.get(state.hashCode());
      setArm1Setpoint(target.arm1Pos);
      setArm2Setpoint(target.arm2Pos);
      setIntakeSetpoint(target.wristPos);
    } catch (Exception e) {
      System.err.println(e);
    }
  }

  public void setArm1Setpoint(double setpoint) {
    arm1TargetPosition = setpoint;
  }

  public void fudgeArm1Setpoint(double amountToMove) {
    arm1TargetPosition += amountToMove;
  }

  public void fudgeArm1WithAnalogStick(double analogStickValue) {
    fudgeArm1Setpoint(analogStickValue * previousLoopTime * 8.0 * 2048.0);
  }

  public void setArm2Setpoint(double setpoint) {
    arm2TargetPosition = setpoint;
  }

  public void fudgeArm2Setpoint(double amountToMove) {
    arm2TargetPosition += amountToMove;
  }

  public void fudgeArm2WithAnalogStick(double analogStickValue) {
    fudgeArm2Setpoint(analogStickValue * previousLoopTime * 24.0 * 2048.0);
  }

  public void setIntakeSetpoint(double setpoint) {
    intakeTargetPosition = setpoint;
  }

  public void fudgeIntakeSetpoint(double amountToMove) {
    intakeTargetPosition += amountToMove;
  }

  public void fudgeIntakeWithAnalogStick(double analogStickValue) {
    fudgeIntakeSetpoint(analogStickValue * previousLoopTime * 16.0);
  }

  private double getArm1LeftAdjustedAbsoluteEncoderPos() {
    double adjustedPos = arm1LeftAbsoluteEncoder.getDistance() - 180.0;
    while (adjustedPos < 0) {
      adjustedPos += 360.0;
    }
    return adjustedPos;
  }

  private double getArm1RightAbsoluteEncoderPos() {
    return Math.abs(arm1RightAbsoluteEncoder.getDistance());
  }

  private double getArm2LeftAdjustedAbsoluteEncoderPos() {
    // double adjustedPos = arm2AbsoluteEncoder.getDistance() - 180.0;
    // while (adjustedPos < 0) {
    //   adjustedPos += 360.0;
    // }
    // return adjustedPos;

    return arm2AbsoluteEncoder.getDistance();
  }

  private double getArm2AbsoluteEncoderPos() {
    return arm2AbsoluteEncoder.getDistance();
  }

  private double getArm2LeftCompensatedAbsoluteEncoderPos() {
    // return getArm2LeftAdjustedAbsoluteEncoderPos() - arm2AdjustedCalibration + getArm1AbsoluteEncoderAverage() * Constants.ArmConstants.stage2ChainRatio;
    return getArm2LeftAdjustedAbsoluteEncoderPos() - Constants.arm2LeftConstants.calibration + getArm1AbsoluteEncoderAverage() * Constants.ArmConstants.stage2ChainRatio;
  }

  private double getArm1AbsoluteEncoderAverage() {
    return (-(getArm1LeftAdjustedAbsoluteEncoderPos() - arm1LeftAdjustedCalibration)
    + (getArm1RightAbsoluteEncoderPos() - Constants.arm1RightConstants.calibration)
    ) / 2;
  }

  private double getIntakeAbsoluteEncoderPos() {
    return intakePivotAbsoluteEncoder.getDistance();
  }

  private double getIntakeAdjustedAbsoluteEncoderPos() {
    double adjustedPos = getIntakeAbsoluteEncoderPos() - 180.0;
    while (adjustedPos < 0) {
      adjustedPos += 360.0;
    }
    return adjustedPos;
  }

  private double getIntakeCalibratedAbsoluteEncoderPos() {
    return -getIntakeAdjustedAbsoluteEncoderPos() + intakeAdjustedCalibration;
  }

  private void resetArm1LeftEncoder() {
    arm1Left.setSelectedSensorPosition((getArm1AbsoluteEncoderAverage() / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
  }

  private void resetArm2Encoder() {
    arm2Left.setSelectedSensorPosition((getArm2LeftCompensatedAbsoluteEncoderPos() / (Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio) / 360.0) * 2048.0);
  }

  private void resetIntakeEncoder() {
    intakePivotLeftEncoder.setPosition(getIntakeCalibratedAbsoluteEncoderPos() / Constants.intakePivotLeftConstants.ratio / 360.0);
  }

  public void updateSideBack() {
    state.side = ArmState.Side.BACK;
    state.altpickupl = false;
  }

  public void updateSideFront() {
    state.side = ArmState.Side.FRONT;
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

  public void toggleAltPickuplTrue() {
    state.altpickupl = !state.altpickupl;
  }

  public void updateFudge(boolean fudge) {
    state.fudge = fudge;
  }

  public ArmState.Side getSide() {
    return state.side;
  }

  public ArmState.GamePiece getPiece() {
    return state.piece;
  }

  public boolean getFudge() {
    return state.fudge;
  }
}
