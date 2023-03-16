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
import frc.robot.subsystems.ArmSubsystem.ArmState.GamePiece;
import frc.robot.utils.LazyTalonFX;

public class ArmSubsystem extends SubsystemBase {
  public static class ArmState {
    public enum Height{
      HIGH,
      MEDIUM,
      LOW, 
      HPPICKUP,
    }

    public enum GamePiece{
      CUBE,
      CONE,
    }

    public Height height;
    public GamePiece piece;
    public boolean altpickupl;
    public boolean fudge;

    ArmState(Height height, GamePiece piece, boolean altpickupl, boolean fudge){
      this.height= height;
      this.piece= piece;
      this.altpickupl= altpickupl;
      this.fudge=fudge;
    }

    @Override
    public boolean equals(Object obj){
      if (obj == null) {
        return false;
    }
    
    if (getClass() != obj.getClass()) {
        return false;
    }

    final ArmState armStateObj = (ArmState) obj;
      return this.height==armStateObj.height && this.piece==armStateObj.piece && this.altpickupl==armStateObj.altpickupl;
    }

    @Override
    public int hashCode(){
      int temp= 0;
      temp += height.ordinal();
      temp += piece.ordinal()*4;
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

  HashMap<ArmState, SetPoint> setPoints = new HashMap<ArmState, SetPoint>() {{
    put(new ArmState(ArmState.Height.HIGH, ArmState.GamePiece.CUBE, false, false), new SetPoint(12300, -97600, 0));
    put(new ArmState(ArmState.Height.HIGH, ArmState.GamePiece.CONE, false, false), new SetPoint(25500, -175000, 40)); 
    put(new ArmState(ArmState.Height.MEDIUM, ArmState.GamePiece.CUBE, false, false), new SetPoint(2200, -64500, 0)); 
    put(new ArmState(ArmState.Height.MEDIUM, ArmState.GamePiece.CONE, false, false), new SetPoint(2000, -115000, 30)); 
    put(new ArmState(ArmState.Height.LOW, ArmState.GamePiece.CUBE, false, false), new SetPoint(33000, -27000, 0)); 
    put(new ArmState(ArmState.Height.LOW, ArmState.GamePiece.CONE, false, false), new SetPoint(33000, -27000, 0)); 
    put(new ArmState(ArmState.Height.HPPICKUP, ArmState.GamePiece.CUBE, false, false), new SetPoint(-17000, -136000, 40)); 
    put(new ArmState(ArmState.Height.HPPICKUP, ArmState.GamePiece.CONE, false, false), new SetPoint(-17000, -136000, 40)); 
  }};

  private ArmState state = new ArmState(ArmState.Height.LOW, ArmState.GamePiece.CONE, false, false);
  private boolean deploy = false;

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

  private final double armRetractPosition = 0;

  private double arm1TargetPosition = 0;
  private double arm2TargetPosition = 0;
  private double intakeTargetPosition = 0;

  private final Timer timer = new Timer();

  private double previousTime = 0;
  private double previousLoopTime = 0;

  private double arm1LeftAdjustedCalibration;
  private double arm2AdjustedCalibration;
  private double intakeAdjustedCalibration;

  private boolean arm1LeftEncoderEnabled = true;
  private boolean arm1RightEncoderEnabled = true;
  private boolean arm2EncoderEnabled = true;
  private boolean intakePivotEncoderEnabled = true;

  private double arm1LeftEncoderPreviousValue = 0;
  private double arm1RightEncoderPreviousValue = 0;
  private double arm2EncoderPreviousValue = 0;
  private double intakePivotEncoderPreviousValue = 0;

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

    arm1Left.configForwardSoftLimitThreshold(40000);
    arm2Left.configForwardSoftLimitThreshold(0);

    arm1Left.configReverseSoftLimitThreshold(-30000);
    arm2Left.configReverseSoftLimitThreshold((-135.0 / (Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio) / 360.0) * 2048.0);

    arm1Left.configForwardSoftLimitEnable(true);
    arm2Left.configForwardSoftLimitEnable(true);
    
    arm1Left.configReverseSoftLimitEnable(true);
    arm2Left.configReverseSoftLimitEnable(true);

    arm1Left.config_kP(0, 0.03);
    arm1Left.config_kI(0, 0);
    arm1Left.config_kD(0, 0.006);
    arm1Left.configNeutralDeadband(0.05);

    arm1Right.follow(arm1Left);
    arm1Right.setInverted(InvertType.OpposeMaster);

    arm2Left.config_kP(0, 0.015); // 0.15
    arm2Left.config_kI(0, 0);
    arm2Left.config_kD(0, 0.015);
    arm2Left.configNeutralDeadband(0.06);
    
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

    intakePivotLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
    intakePivotLeft.setSoftLimit(SoftLimitDirection.kForward, (float) (135.0 / Constants.intakePivotLeftConstants.ratio / 360.0));

    intakePivotLeft.setInverted(true);
    intakePivotRight.follow(intakePivotLeft, true);

    intakePivotPID = intakePivotLeft.getPIDController();

    intakePivotPID.setP(0.075);
    intakePivotPID.setD(0.0);
    intakePivotPID.setOutputRange(-0.5, 0.5);

    intakePivotLeftEncoder = intakePivotLeft.getEncoder();

    arm1LeftAbsoluteEncoder = new DutyCycleEncoder(Constants.arm1LeftEncoder);
    arm1RightAbsoluteEncoder = new DutyCycleEncoder(Constants.arm1RightEncoder);
    arm2AbsoluteEncoder = new DutyCycleEncoder(Constants.arm2LeftEncoder);
    intakePivotAbsoluteEncoder = new DutyCycleEncoder(Constants.intakePivotEncoder);

    arm1LeftAbsoluteEncoder.setDistancePerRotation(360);
    arm1RightAbsoluteEncoder.setDistancePerRotation(360);
    arm2AbsoluteEncoder.setDistancePerRotation(360);
    intakePivotAbsoluteEncoder.setDistancePerRotation(360);
  
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

    // There needs to be a chillout time for the intake motor controllers to reset their encoders properly
    try {
      Thread.sleep(2000);
    } catch (Exception e) {

    }

    resetEncoders();

    // There needs to be a chillout time for the intake motor controllers to reset their encoders properly
    try {
      Thread.sleep(500);
    } catch (Exception e) {

    }

    setArm1Setpoint(arm1Left.getSelectedSensorPosition());
    setArm2Setpoint(arm2Left.getSelectedSensorPosition());
    setIntakeSetpoint(intakePivotLeftEncoder.getPosition());

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    previousLoopTime = timer.get() - previousTime;
    previousTime = timer.get();

    // Disable absolute encoders if they're value isn't changing (The value constantly fluxuates, so an unchanging value is a red flag)
    if (arm1LeftEncoderPreviousValue == getArm1LeftAdjustedAbsoluteEncoderPos()) arm1LeftEncoderEnabled = false;
    if (arm1RightEncoderPreviousValue == getArm1RightAbsoluteEncoderPos()) arm1RightEncoderEnabled = false;
    if (arm2EncoderPreviousValue == getArm2AbsoluteEncoderPos()) arm2EncoderEnabled = false;
    if (intakePivotEncoderPreviousValue == getIntakeAbsoluteEncoderPos()) intakePivotEncoderEnabled = false;

    if (!deploy) {
      setArm1Setpoint(-12000);
      setArm2Setpoint(-7000);
      setIntakeSetpoint(armRetractPosition);
    }
    
    if (Math.abs(getArm2LeftCompensatedAbsoluteEncoderPos()) < 10) {
      arm1TargetPosition = Math.min(arm1TargetPosition, 2.0 * Constants.arm1LeftConstants.ratio * 2048.0 / 360.0);
    }

    if (Math.abs(getArm1AbsoluteEncoderAverage()) < 20 && Math.abs(getArm2LeftCompensatedAbsoluteEncoderPos()) < 30) {
      intakeTargetPosition = armRetractPosition;
    } else if (
      Math.abs(getArm1AbsoluteEncoderAverage()) < 30 
      && Math.abs(getArm2LeftCompensatedAbsoluteEncoderPos()) < 40 
      && Math.abs(arm2Left.getSelectedSensorVelocity() * 10 / 2048.0) > 2000 // Falcon is spinning at 2000 rpm or faster
    ) {
      intakeTargetPosition = armRetractPosition;
    }

    intakeTargetPosition = MathUtil.clamp(intakeTargetPosition, 0, 50);

    arm1Left.set(ControlMode.Position, arm1TargetPosition);
    arm2Left.set(ControlMode.Position, arm2TargetPosition);
    intakePivotPID.setReference(intakeTargetPosition, ControlType.kPosition);
    
    SmartDashboard.putNumber("Left 1 absolute encoder", getArm1LeftAdjustedAbsoluteEncoderPos());
    // SmartDashboard.putNumber("Right 1 absolute encoder", getArm1RightAbsoluteEncoderPos());
    SmartDashboard.putNumber("Arm 1 absolute pos", getArm1AbsoluteEncoderAverage());

    SmartDashboard.putNumber("Left 1 calculated relative pos", (-(getArm1LeftAdjustedAbsoluteEncoderPos() - arm1LeftAdjustedCalibration) / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);

    SmartDashboard.putNumber("Left 1 relative pos", arm1Left.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm 1 Target Position", arm1TargetPosition);

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
    SmartDashboard.putNumber("Wrist applied power", intakePivotLeft.get());

    SmartDashboard.putBoolean("Arm target setpoint is cube", state.piece == ArmState.GamePiece.CUBE);
    SmartDashboard.putString("Arm target setpoint height", state.height.name());
    SmartDashboard.putBoolean("Arm target setpoint is alt pickup 1", state.altpickupl);
    SmartDashboard.putBoolean("Arm target setpoint is in fudge mod", state.fudge);
    SmartDashboard.putBoolean("Arm should be deployed", deploy);

    SmartDashboard.putBoolean("Left 1 Encoder Enabled", arm1LeftEncoderEnabled);
    SmartDashboard.putBoolean("Right 1 Encoder Enabled", arm1RightEncoderEnabled);
    SmartDashboard.putBoolean("Arm 2 Encoder Enabled", arm2EncoderEnabled);
    SmartDashboard.putBoolean("Intake Pivot Encoder Enabled", intakePivotEncoderEnabled);

    SmartDashboard.putString("Hello World", "Is this changing");
  }

  public void stop() {
    arm1Left.set(ControlMode.PercentOutput, 0.0);
    arm2Left.set(ControlMode.PercentOutput, 0.0);
    intakePivotLeft.stopMotor();
  }

  public void setpointsFromStateMachine() {
    try{
      SetPoint target = setPoints.get(state);
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

  public void resetEncoders() {
    resetArm1LeftEncoder();
    resetArm2Encoder();
    resetIntakeEncoder();
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
    return arm2AbsoluteEncoder.getDistance();
  }

  private double getArm2AbsoluteEncoderPos() {
    return arm2AbsoluteEncoder.getDistance();
  }

  private double getArm2LeftCompensatedAbsoluteEncoderPos() {
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

  private double getIntakeCalibratedAbsoluteEncoderPos() {
    return getIntakeAbsoluteEncoderPos() - Constants.intakePivotLeftConstants.calibration;
  }

  private void resetArm1LeftEncoder() {
    if (arm1LeftEncoderEnabled && arm1RightEncoderEnabled) arm1Left.setSelectedSensorPosition((getArm1AbsoluteEncoderAverage() / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
    else if (arm1LeftEncoderEnabled && ! arm1RightEncoderEnabled) arm1Left.setSelectedSensorPosition((-(getArm1LeftAdjustedAbsoluteEncoderPos() - arm1LeftAdjustedCalibration) / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
    else if (!arm1LeftEncoderEnabled && arm1RightEncoderEnabled) arm1Left.setSelectedSensorPosition(((getArm1RightAbsoluteEncoderPos() - Constants.arm1RightConstants.calibration) / Constants.arm1LeftConstants.ratio / 360.0) * 2048.0);
  }

  private void resetArm2Encoder() {
    if (arm2EncoderEnabled) arm2Left.setSelectedSensorPosition((getArm2LeftCompensatedAbsoluteEncoderPos() / (Constants.arm2LeftConstants.ratio * Constants.ArmConstants.stage2ChainRatio) / 360.0) * 2048.0);
  }

  private void resetIntakeEncoder() {
    if (intakePivotEncoderEnabled) intakePivotLeftEncoder.setPosition(getIntakeCalibratedAbsoluteEncoderPos() / Constants.intakePivotLeftConstants.ratio / 360.0);
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

  public void togglePiece() {
    if (state.piece == GamePiece.CUBE) {
      state.piece = GamePiece.CONE;
    } else {
      state.piece = GamePiece.CUBE;
    }
    state.altpickupl = false;
  }

  public void toggleAltPickupl() {
    state.altpickupl = !state.altpickupl;
  }

  public void updateFudgeTrue() {
    if (!state.fudge) {
      state.fudge = true;
      setArm1Setpoint(arm1Left.getSelectedSensorPosition());
      setArm2Setpoint(arm2Left.getSelectedSensorPosition());
      setIntakeSetpoint(intakePivotLeftEncoder.getPosition());
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

  public ArmState.GamePiece getPiece() {
    return state.piece;
  }

  public ArmState.Height getHeight() {
    return state.height;
  }

  public boolean getFudge() {
    return state.fudge;
  }

  public boolean getDeploy() {
    return deploy;
  }
}
