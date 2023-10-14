// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class DriveAutoBalance extends CommandBase {
  private final PIDController yawController = new PIDController(-4.0, 0, -0);

  private int loopCount = 0;

  /** Creates a new DriveAutoBalance. */
  public DriveAutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = driveSubsystem.getPitch();

    double rotationSpeed = yawController.calculate(driveSubsystem.getOdometryLocation().getRotation().getRadians());

    if (Math.abs(currentPitch) < 5) {
      driveSubsystem.robotDrive(0, 0, 0, false);
      loopCount += 1;
      
    } else if (currentPitch < -10.0) {
      driveSubsystem.robotDrive(0.25, 0, rotationSpeed, false);
      loopCount = 0;
    } else if (currentPitch < -8.0) {
      driveSubsystem.robotDrive(0.15, 0, rotationSpeed, false);
    } else if (currentPitch > 8.0) {
      driveSubsystem.robotDrive(-0.15, 0, rotationSpeed, false);
    }else if (currentPitch > 10.0) {
      driveSubsystem.robotDrive(-0.25, 0, rotationSpeed, false);
      loopCount = 0;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getPitch()) < 2.2 && loopCount > 20;
  }
}
