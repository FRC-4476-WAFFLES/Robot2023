// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.EncoderTest;

public class IntakeSubsystem extends SubsystemBase {
  private EncoderTest encoderTest;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    encoderTest = new EncoderTest(Constants.intakePivotEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder", encoderTest.getPosition());
  }
}
