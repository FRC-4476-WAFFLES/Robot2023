// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAutoBalance;

import static frc.robot.RobotContainer.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCubeAndPickupAndBalance extends SequentialCommandGroup {
  /** Creates a new OneCubeAndPickupAndBalance. */
  public OneCubeAndPickupAndBalance() {
    PathPlannerTrajectory driveToScore = PathPlanner.loadPath("Start to Scoring", new PathConstraints(1, 1));
    PathPlannerTrajectory driveToPickup = PathPlanner.loadPath("Scoring to Pickup", new PathConstraints(3, 2));
    PathPlannerTrajectory driveToClimb = PathPlanner.loadPath("Pickup to Climb", new PathConstraints(3, 2));

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

        new InstantCommand(() -> {
          PathPlannerState initialState = driveToPickup.getInitialState();
          PathPlannerState transformedInitialState = PathPlannerTrajectory.transformStateForAlliance(initialState, DriverStation.getAlliance());
          driveSubsystem.resetOdometry(new Pose2d(transformedInitialState.poseMeters.getTranslation(), transformedInitialState.holonomicRotation));
        }, driveSubsystem),

        new PPSwerveControllerCommand(
          driveToPickup,
          driveSubsystem::getOdometryLocation,
          driveSubsystem.kinematics,
          new PIDController(2, 0, 0),
          new PIDController(2, 0, 0),
          new PIDController(-1, 0, 0),
          driveSubsystem::setModuleStates,
          true,
          driveSubsystem
        ).alongWith(
          new SequentialCommandGroup(
            new WaitCommand(0.5),
            new InstantCommand(armSubsystem::updateDeployFalse, armSubsystem),
            new WaitCommand(1.0),
            new InstantCommand(() -> {
              armSubsystem.updateGamePieceCube();
              armSubsystem.updateHeightScoreLow();
              armSubsystem.updateDeployTrue();
            }, armSubsystem),
            new InstantCommand(() -> intakeSubsystem.setPower(0.5))
          )
        ),

        new WaitCommand(0.5),
        new InstantCommand(() -> intakeSubsystem.setPower(0.1)),
        new InstantCommand(armSubsystem::updateDeployFalse),

        new PPSwerveControllerCommand(
          driveToClimb,
          driveSubsystem::getOdometryLocation,
          driveSubsystem.kinematics,
          new PIDController(2, 0, 0),
          new PIDController(2, 0, 0),
          new PIDController(-1, 0, 0),
          driveSubsystem::setModuleStates,
          true,
          driveSubsystem
        )
      ).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine).repeatedly()),

      new DriveAutoBalance()
    );
    addRequirements(intakeSubsystem);
  }
}
