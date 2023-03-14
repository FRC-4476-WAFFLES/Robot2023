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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

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

    // if(camera.getHasTarget()) {
    //   driveSubsystem.resetOdometry(camera.getRobotPose2d());
    // }

    // double lowestDistance = Double.MAX_VALUE;
    // double curPosY = driveSubsystem.getOdometryLocation().getY();
    // Pose2d lowestScoringOption = new Pose2d();

    // for(int i = 0; i < (armSubsystem.getPiece().equals(ArmSubsystem.ArmState.GamePiece.CUBE) ? 3 : 6); i++) {
    //   Pose2d pose = driveSubsystem.map.get(
    //     new DriveSubsystem.DriveState(armSubsystem.getPiece() == ArmSubsystem.ArmState.GamePiece.CUBE ? DriveSubsystem.DriveState.GamePiece.CUBE : DriveSubsystem.DriveState.GamePiece.CONE,
    //     DriverStation.getAlliance() == Alliance.Red ? DriveSubsystem.DriveState.Alliance.RED : DriveSubsystem.DriveState.Alliance.BLUE, i));
    //   double diff = Math.abs(curPosY - pose.getY());
    //   if(diff < lowestDistance) {
    //     lowestDistance = diff;
    //     lowestScoringOption = pose;
    //   }
    // }

    // PathPlannerTrajectory traj2 = PathPlanner.generatePath(
    //   new PathConstraints(4, 3), 
    //   new PathPoint(driveSubsystem.getOdometryLocation().getTranslation(), DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0), driveSubsystem.getOdometryLocation().getRotation()), // position, heading(direction of travel), holonomic rotation
    //   new PathPoint(lowestScoringOption.getTranslation(), DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180), lowestScoringOption.getRotation()// position, heading(direction of travel), holonomic rotation
    // ));

    // swerveCommand = new PPSwerveControllerCommand(
    //   traj2, 
    //   driveSubsystem::getOdometryLocation,
    //   driveSubsystem.kinematics,
    //   new PIDController(2, 0, 0),
    //   new PIDController(2, 0, 0),
    //   new PIDController(-1, 0, 0),
    //   driveSubsystem::setModuleStates,
    //   false,
    //   driveSubsystem
    // );

    // swerveCommand.initialize();

    // isDriving = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(camera.getHasTarget() && !hasFoundTarget) {
      driveSubsystem.resetOdometry(new Pose2d(camera.getRobotPose2d().getTranslation(), camera.getRobotPose2d().getRotation().plus(Rotation2d.fromDegrees(180))));

      double lowestDistance = Double.MAX_VALUE;
      double curPosY = driveSubsystem.getOdometryLocation().getY();
      Pose2d lowestScoringOption = new Pose2d();
  
      for(int i = 0; i < (armSubsystem.getPiece().equals(ArmSubsystem.ArmState.GamePiece.CUBE) ? 3 : 6); i++) {
        Pose2d pose = driveSubsystem.scoringLocations.get(
          new DriveSubsystem.DriveState(armSubsystem.getPiece() == ArmSubsystem.ArmState.GamePiece.CUBE ? DriveSubsystem.DriveState.GamePiece.CUBE : DriveSubsystem.DriveState.GamePiece.CONE,
          DriverStation.getAlliance() == Alliance.Red ? DriveSubsystem.DriveState.Alliance.RED : DriveSubsystem.DriveState.Alliance.BLUE, i));
        double diff = Math.abs(curPosY - pose.getY());
        if(diff < lowestDistance) {
          lowestDistance = diff;
          lowestScoringOption = pose;
        }
      }

      System.err.print("Closest target: ");
      System.err.println(lowestScoringOption);

      System.err.print("Current Pose: ");
      System.err.println(driveSubsystem.getOdometryLocation());
  
      PathPlannerTrajectory traj2 = PathPlanner.generatePath(
        new PathConstraints(1, 1), 
        new PathPoint(driveSubsystem.getOdometryLocation().getTranslation(), DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0), driveSubsystem.getOdometryLocation().getRotation()), // position, heading(direction of travel), holonomic rotation
        new PathPoint(lowestScoringOption.getTranslation(), DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180), lowestScoringOption.getRotation() // .plus(Rotation2d.fromDegrees(180))// position, heading(direction of travel), holonomic rotation
      ));
  
      swerveCommand = new PPSwerveControllerCommand(
        traj2, 
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

      System.err.println(traj2);
  
      isDriving = true;
      hasFoundTarget = true;
    }
    
    if (swerveCommand != null && isDriving && hasFoundTarget) {
      swerveCommand.execute();
      System.err.println("Driving to target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveCommand.end(interrupted);
    isDriving = false;
    hasFoundTarget = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveCommand.isFinished();
  }
}
