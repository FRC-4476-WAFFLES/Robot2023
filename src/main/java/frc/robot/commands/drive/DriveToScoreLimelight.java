// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera.CameraLEDMode;
import frc.robot.subsystems.Camera.Pipeline;
import frc.robot.utils.JoystickDrive;
import frc.robot.utils.TurnToNearest90;

import static frc.robot.RobotContainer.*;
import static frc.robot.Constants.*;

public class DriveToScoreLimelight extends CommandBase {
  private final PIDController alignController = new PIDController(-0.07, 0, -0.0);
  private final JoystickDrive drive = new JoystickDrive();
  private final TurnToNearest90 rotationController = new TurnToNearest90();

  /** Creates a new DriveToScoreLimelight. */
  public DriveToScoreLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setLEDMode(CameraLEDMode.On);
    alignController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armSubsystem.getPiece() == GamePiece.CUBE) {
      camera.setPipeline(Pipeline.AprilTags);
    } else {
      camera.setPipeline(Pipeline.RetroReflective);
    }
    
    if (camera.getHasTarget()) {
      driveSubsystem.robotDrive(
        drive.getForward(), 
        alignController.calculate(camera.getFilteredHorizontal()), 
        rotationController.calculate(driveSubsystem.getOdometryLocation().getRotation().getDegrees()), 
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

    driveSubsystem.setAutoAiming(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    camera.setLEDMode(CameraLEDMode.Off);
    driveSubsystem.stop();
    driveSubsystem.setAutoAiming(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return camera.getHasTarget() && Math.abs(camera.getFilteredHorizontal()) < 3.0;
    return false;
  }
}
