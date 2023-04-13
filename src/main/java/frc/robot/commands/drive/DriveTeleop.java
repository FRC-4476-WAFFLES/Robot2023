// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.utils.JoystickDrive;
import frc.robot.utils.TurnToNearest90;

import static frc.robot.RobotContainer.*;

public class DriveTeleop extends CommandBase {
  private final JoystickDrive drive;
  private final TurnToNearest90 nearest90;

  public DriveTeleop() {
    // Tell the scheduler that no other drive commands can be running while
    // this one is running.
    drive = new JoystickDrive();
    nearest90 = new TurnToNearest90();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armSubsystem.getDeploy() && !armSubsystem.getHeight().equals(Constants.Height.PICKUP_GROUND)) {
      driveSubsystem.robotDrive(
        drive.getForward(), 
        drive.getRight(), 
        nearest90.calculate(driveSubsystem.getOdometryLocation().getRotation().getDegrees()), 
        true
      );
    } else {
      driveSubsystem.robotDrive(
        drive.getForward(), 
        drive.getRight(), 
        drive.getRotation(), 
        true
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors when the command ends. If we didn't do this, they might
    // continue running during the next command.
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
