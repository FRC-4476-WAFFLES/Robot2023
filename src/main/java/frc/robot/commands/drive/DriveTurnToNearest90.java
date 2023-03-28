// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.JoystickDrive;
import frc.robot.utils.TurnToNearest90;

import static frc.robot.RobotContainer.*;

public class DriveTurnToNearest90 extends CommandBase {
  private final TurnToNearest90 turn;
  private final JoystickDrive drive;

  /** Creates a new TurnToNearest90. */
  public DriveTurnToNearest90() {
    this.turn = new TurnToNearest90();
    this.drive = new JoystickDrive();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.robotDrive(
      drive.getForward(), 
      drive.getRight(), 
      turn.calculate(driveSubsystem.getOdometryLocation().getRotation().getDegrees()), 
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getOdometryLocation().getRotation().getDegrees() % 90) < 5.0;
  }
}
