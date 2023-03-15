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

import static frc.robot.RobotContainer.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCubeAndPickup extends SequentialCommandGroup {
  /** Creates a new OnePieceAndBalance. */
  public TwoCubeAndPickup() {
    PathPlannerTrajectory driveToScore1 = PathPlanner.loadPath("Start to Scoring", new PathConstraints(1, 1));
    PathPlannerTrajectory driveToPickup1 = PathPlanner.loadPath("Scoring to Pickup", new PathConstraints(3, 2));
    PathPlannerTrajectory driveToScore2 = PathPlanner.loadPath("Pickup to Scoring Medium", new PathConstraints(3, 2));
    PathPlannerTrajectory driveToPickup2 = PathPlanner.loadPath("Scoring to Pickup 2", new PathConstraints(3, 2));

    addCommands(
      new InstantCommand(() -> {
        armSubsystem.updateGamePieceCube();
        armSubsystem.updateHeightHigh();
        armSubsystem.updateFudgeFalse();
        armSubsystem.updateDeployTrue();
        driveSubsystem.resetOdometry(driveToScore1.getInitialHolonomicPose());
        intakeSubsystem.setPower(0.1);
      }, armSubsystem, driveSubsystem),
      new SequentialCommandGroup(
        new WaitCommand(1.5), 
        new PPSwerveControllerCommand(
          driveToScore1,
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
          PathPlannerState initialState = driveToPickup1.getInitialState();
          PathPlannerState transformedInitialState = PathPlannerTrajectory.transformStateForAlliance(initialState, DriverStation.getAlliance());
          driveSubsystem.resetOdometry(new Pose2d(transformedInitialState.poseMeters.getTranslation(), transformedInitialState.holonomicRotation));
        }, driveSubsystem),

        new PPSwerveControllerCommand(
          driveToPickup1,
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
              armSubsystem.updateHeightLow();
              armSubsystem.updateDeployTrue();
            }, armSubsystem),
            new InstantCommand(() -> intakeSubsystem.setPower(0.5))
          )
        ),

        new WaitCommand(0.5),
        new InstantCommand(() -> intakeSubsystem.setPower(0.1)),
        new InstantCommand(armSubsystem::updateDeployFalse),

        new PPSwerveControllerCommand(
          driveToScore2,
          driveSubsystem::getOdometryLocation,
          driveSubsystem.kinematics,
          new PIDController(2, 0, 0),
          new PIDController(2, 0, 0),
          new PIDController(-1, 0, 0),
          driveSubsystem::setModuleStates,
          true,
          driveSubsystem
        ).alongWith(
          new WaitCommand(2.0).andThen(new InstantCommand(() -> {
            armSubsystem.updateGamePieceCube();
            armSubsystem.updateHeightMedium();
            armSubsystem.updateDeployTrue();
          }, armSubsystem))
        ),
        new InstantCommand(() -> intakeSubsystem.setPower(-0.3)),
        new WaitCommand(0.5),
        new InstantCommand(() -> intakeSubsystem.setPower(0)),

        new PPSwerveControllerCommand(
          driveToPickup2,
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
              armSubsystem.updateHeightLow();
              armSubsystem.updateDeployTrue();
            }, armSubsystem),
            new InstantCommand(() -> intakeSubsystem.setPower(0.5))
          )
        ),

        new WaitCommand(0.5),
        new InstantCommand(() -> intakeSubsystem.setPower(0.1)),
        new InstantCommand(armSubsystem::updateDeployFalse)
      ).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine).repeatedly())
    );
    addRequirements(intakeSubsystem);
  }
}
