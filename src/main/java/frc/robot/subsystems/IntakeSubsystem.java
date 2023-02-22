// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeLeft;
  private final CANSparkMax intakeRight;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeLeft = new CANSparkMax(Constants.intakeLeft, MotorType.kBrushless);
    intakeRight = new CANSparkMax(Constants.intakeRight, MotorType.kBrushless);

    intakeLeft.restoreFactoryDefaults();
    intakeRight.restoreFactoryDefaults();

    intakeLeft.setSmartCurrentLimit(20);
    intakeRight.setSmartCurrentLimit(20);

    intakeLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLeftPower(double power) {
    intakeLeft.set(power);
  }

  public void setRightPower(double power) {
    intakeRight.set(power);
  }

  public void stop() {
    intakeLeft.stopMotor();
    intakeRight.stopMotor();
  }
}
