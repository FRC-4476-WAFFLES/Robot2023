// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveAutoBalance;

import static frc.robot.RobotContainer.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCubeAndMobilityAndBalance extends SequentialCommandGroup {
  /** Creates a new OneCubeAndMobilityAndBalance. */
  public OneCubeAndMobilityAndBalance() {
    PathPlannerTrajectory driveToScore = PathPlanner.loadPath("Start to Scoring", new PathConstraints(1, 1));
    PathPlannerTrajectory driveToMobility = PathPlanner.loadPath("1 Cube Climb", new PathConstraints(1, 1));
    PathPlannerTrajectory driveToClimb = PathPlanner.loadPath("Mobility to Climb", new PathConstraints(1, 1));

    addCommands(
      new InstantCommand(() -> {
        armSubsystem.updateGamePieceCube();
        armSubsystem.updateHeightScoreHigh();
        armSubsystem.updateFudgeFalse();
      }, armSubsystem),
      new InstantCommand(() -> intakeSubsystem.setPower(0.1)),
      
      new InstantCommand(() -> driveSubsystem.resetOdometry(driveToScore.getInitialHolonomicPose()), driveSubsystem),
      new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem),
      new SequentialCommandGroup(
        new InstantCommand(armSubsystem::updateDeployTrue, armSubsystem), 
        new WaitCommand(1.5), 
        new PPSwerveControllerCommand(
          driveToScore,
          driveSubsystem::getOdometryLocation,
          driveSubsystem.kinematics,
          new PIDController(2, 0, 0),
          new PIDController(2, 0, 0),
          new PIDController(-1, 0, 0),
          driveSubsystem::setModuleStates,
          false,
          driveSubsystem
        ).withTimeout(1), 
        new InstantCommand(() -> intakeSubsystem.setPower(-0.3)),
        new WaitCommand(0.5),
        new InstantCommand(intakeSubsystem::stop),

        new InstantCommand(() -> driveSubsystem.resetOdometry(driveToMobility.getInitialHolonomicPose()), driveSubsystem)
      ).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine).repeatedly()),

      new PPSwerveControllerCommand(
        driveToMobility,
        driveSubsystem::getOdometryLocation,
        driveSubsystem.kinematics,
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        new PIDController(-1, 0, 0),
        driveSubsystem::setModuleStates,
        false,
        driveSubsystem
      ).alongWith(
        new WaitCommand(0.5).andThen(new InstantCommand(armSubsystem::updateDeployFalse, armSubsystem))
      ),

      new InstantCommand(() -> driveSubsystem.resetOdometry(driveToClimb.getInitialHolonomicPose()), driveSubsystem),
      
      new PPSwerveControllerCommand(
        driveToClimb,
        driveSubsystem::getOdometryLocation,
        driveSubsystem.kinematics,
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        new PIDController(-1, 0, 0),
        driveSubsystem::setModuleStates,
        false,
        driveSubsystem
      )//.until(() -> Math.abs(driveSubsystem.getPitch()) > 13),
      .raceWith(new SequentialCommandGroup(new WaitUntilCommand(() -> Math.abs(driveSubsystem.getPitch()) > 13), new WaitCommand(1.5))),
      
      new DriveAutoBalance(),
      new InstantCommand(driveSubsystem::updateLockWheelsTrue, driveSubsystem)
    );
    addRequirements(intakeSubsystem);
  }
}
