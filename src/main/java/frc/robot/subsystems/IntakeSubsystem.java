// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intake = new CANSparkMax(Constants.intake, MotorType.kBrushless);

    intake.restoreFactoryDefaults();

    intake.setSmartCurrentLimit(20);

    intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Roller pos", intake.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake Roller power", intake.get());
  }

  public void setPower(double power) {
    // intake.set(power);
  }

  public double getPower() {
    return intake.get();
  }

  public void stop() {
    intake.stopMotor();
  }
}
