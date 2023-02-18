// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;


public class DriveTest extends CommandBase {
  private Timer timer = new Timer();
  private double maxCurrent = 0.0;
  /** Creates a new DriveTest. */
  public DriveTest() {
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetSteerEncoders();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 5) {
      driveSubsystem.robotDrive(1, 0, 0, false);
    } else if(timer.get() < 10) {
      driveSubsystem.robotDrive(-1, 0, 0, false);
    } else if(timer.get() < 15) {
      driveSubsystem.robotDrive(0, 1, 0, false);
    } else if(timer.get() < 20) {
      driveSubsystem.robotDrive(0, -1, 0, false);
    } else if(timer.get() < 25) {
      driveSubsystem.robotDrive(-1, 0, 0, false);
    } else if(timer.get() < 30) {
      driveSubsystem.robotDrive(1, 0, 0, false);
    } else if(timer.get() < 35) {
      driveSubsystem.robotDrive(0, -1, 0, false);
    } else if(timer.get() < 40) {
      driveSubsystem.robotDrive(0, 1, 0, false);
    } else if(timer.get() < 45) {
      driveSubsystem.robotDrive(0, 0, Math.toRadians(72), false);
    } else if(timer.get() < 50) {
      driveSubsystem.robotDrive(0, 0, -Math.toRadians(72), false);
    } else if(timer.get() < 55) {
      driveSubsystem.robotDrive(0, 0, 0, false);
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
    return false;
  }
}
