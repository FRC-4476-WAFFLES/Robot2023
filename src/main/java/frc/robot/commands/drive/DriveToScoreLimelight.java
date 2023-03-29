// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.DriveState;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;
import static frc.robot.Constants.*;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class DriveToScoreLimelight extends CommandBase {
  private PPSwerveControllerCommand swerveCommand;
  private boolean hasFoundTarget = false;
  private boolean isDriving = false;

  /** Creates a new DriveToScoreLimelight. */
  public DriveToScoreLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setLEDMode(CameraLEDMode.On);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(camera.getHasTarget() && !hasFoundTarget) {
      Pose2d cameraRobotPose = camera.getRobotPose2d();
      driveSubsystem.resetOdometry(new Pose2d(cameraRobotPose.getTranslation(), cameraRobotPose.getRotation().plus(Rotation2d.fromDegrees(180))));

      // If one of the HP April Tags are in view, drive to pickup. Otherwise, drive to score
      HashMap<DriveState, Pose2d> locations;
      int numberOfOptions;
      if (camera.getAprilTagID() == 4 || camera.getAprilTagID() == 5) {
        locations = DriveConstants.pickupLocations;
        numberOfOptions = 2;
      } else {
        locations = DriveConstants.scoringLocations;
        numberOfOptions = armSubsystem.getPiece().equals(GamePiece.CUBE) ? 3 : 6;
      }

      double lowestDistance = Double.MAX_VALUE;
      double curPosY = driveSubsystem.getOdometryLocation().getY();
      Pose2d closestOption = new Pose2d();
  
      for(int i = 0; i < numberOfOptions; i++) {
        Pose2d pose = locations.get(new DriveConstants.DriveState(armSubsystem.getPiece(), DriverStation.getAlliance(), i));
        double diff = Math.abs(curPosY - pose.getY());
        if(diff < lowestDistance) {
          lowestDistance = diff;
          closestOption = pose;
        }
      }

      System.err.print("Closest target: ");
      System.err.println(closestOption);

      System.err.print("Current Pose: ");
      System.err.println(driveSubsystem.getOdometryLocation());
  
      PathPlannerTrajectory pathToLocation = PathPlanner.generatePath(
        new PathConstraints(1, 1), 
        new PathPoint(driveSubsystem.getOdometryLocation().getTranslation(), DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0), driveSubsystem.getOdometryLocation().getRotation()), // position, heading(direction of travel), holonomic rotation
        new PathPoint(closestOption.getTranslation(), DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180), closestOption.getRotation() // .plus(Rotation2d.fromDegrees(180))// position, heading(direction of travel), holonomic rotation
      ));
  
      swerveCommand = new PPSwerveControllerCommand(
        pathToLocation, 
        driveSubsystem::getOdometryLocation,
        driveSubsystem.kinematics,
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        new PIDController(-1, 0, 0),
        driveSubsystem::setModuleStates,
        false,
        driveSubsystem
      );

      swerveCommand.initialize();
  
      isDriving = true;
      hasFoundTarget = true;
    }
    
    if (swerveCommand != null && isDriving && hasFoundTarget) {
      swerveCommand.execute();
      // System.err.println("Driving to target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveCommand.end(interrupted);
    isDriving = false;
    hasFoundTarget = false;
    camera.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveCommand.isFinished();
  }
}
