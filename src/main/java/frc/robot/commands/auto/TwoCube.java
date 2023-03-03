// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCube extends SequentialCommandGroup {
  /** Creates a new OnePieceAndBalance. */
  public TwoCube() {
    PathPlannerTrajectory driveToScore = PathPlanner.loadPath("Start to Scoring", new PathConstraints(1, 1));
    PathPlannerTrajectory driveToPickup = PathPlanner.loadPath("Scoring to Pickup", new PathConstraints(3, 2));

    addCommands(
      new InstantCommand(() -> {
        armSubsystem.updateGamePieceCube();
        armSubsystem.updateHeightHigh();
        armSubsystem.updateSideFront();
        armSubsystem.updateFudgeFalse();
      }, armSubsystem),
      new InstantCommand(() -> driveSubsystem.resetOdometry(driveToScore.getInitialHolonomicPose()), driveSubsystem),
      new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem),
      new SequentialCommandGroup(
        new InstantCommand(armSubsystem::updateDeployTrue, armSubsystem), 
        new WaitCommand(1.5), 
        // new InstantCommand(() -> {
        //   armSubsystem.resetArm1LeftEncoder();
        //   armSubsystem.resetArm2Encoder();
        //   armSubsystem.resetIntakeEncoder();
        // }),
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
        new InstantCommand(() -> intakeSubsystem.setPower(-0.3), intakeSubsystem),
        new WaitCommand(0.5),
        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),

        new InstantCommand(() -> driveSubsystem.resetOdometry(driveToPickup.getInitialHolonomicPose()), driveSubsystem),

        new PPSwerveControllerCommand(
          driveToPickup,
          driveSubsystem::getOdometryLocation,
          driveSubsystem.kinematics,
          new PIDController(2, 0, 0),
          new PIDController(2, 0, 0),
          new PIDController(-1, 0, 0),
          driveSubsystem::setModuleStates,
          false,
          driveSubsystem
        ).alongWith(
          new WaitCommand(0.5).andThen(new InstantCommand(() -> {
            armSubsystem.updateGamePieceCube();
            armSubsystem.updateHeightLow();
            armSubsystem.updateSideBack();
          }, armSubsystem)),
          new InstantCommand(() -> intakeSubsystem.setPower(0.5))
        ),
        new WaitCommand(0.5),
        new InstantCommand(() -> intakeSubsystem.setPower(0)),
        new InstantCommand(armSubsystem::updateDeployFalse)
      ).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine).repeatedly())
    );
  }
}
