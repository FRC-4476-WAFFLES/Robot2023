package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;

public class SparkMaxAbsoluteControlHelper {
  private final CANSparkMax m_sparkMax;
  private final SparkMaxPIDController m_PIDController;
  private final SparkMaxAbsoluteEncoder m_absoluteEncoder;

  public SparkMaxAbsoluteControlHelper(ArmConstants constants) {
    m_sparkMax = new CANSparkMax(constants.motorID, MotorType.kBrushless);
    m_sparkMax.restoreFactoryDefaults();
    m_sparkMax.setSmartCurrentLimit(constants.currentLimit);
    m_sparkMax.setIdleMode(IdleMode.kBrake);
    m_sparkMax.setControlFramePeriodMs(40);
    m_sparkMax.setInverted(constants.isInverted);

    m_absoluteEncoder = m_sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_absoluteEncoder.setZeroOffset(constants.calibration);
    m_absoluteEncoder.setPositionConversionFactor(360); // TODO: I think that the absolute encoder returns position in degrees, which would be great. Otherwise this should be altered to make it be degrees

    m_PIDController = m_sparkMax.getPIDController();
    m_PIDController.setFeedbackDevice(m_absoluteEncoder);

    // TODO: set these values to better values determined through testing. 
    // set PID coefficients
    m_PIDController.setP(0.1);
    m_PIDController.setI(0);
    m_PIDController.setD(0);
    m_PIDController.setIZone(0);
    m_PIDController.setFF(0);
    m_PIDController.setOutputRange(-0.5, 0.5);
  }

  public void setTarget(double degrees) {
    m_PIDController.setReference(degrees, ControlType.kPosition);
  }

  public void setTarget(double degrees, double arbFeedforward) {
    m_PIDController.setReference(degrees, ControlType.kPosition, 0, arbFeedforward);
  }

  public double getPosition() {
    return m_absoluteEncoder.getPosition();
  }

  public void setP(double kP) {
    m_PIDController.setP(kP);
  }

  public void setI(double kI) {
    m_PIDController.setI(kI);
  }

  public void setD(double kD) {
    m_PIDController.setD(kD);
  }

  public void setF(double kFF) {
    m_PIDController.setFF(kFF);
  }
}
