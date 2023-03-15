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
public class AutoPlaceGamepiece extends SequentialCommandGroup {
  /** Creates a new AutoPlaceGamepiece. */
  public AutoPlaceGamepiece() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory path = PathPlanner.loadPath("Start to Scoring", new PathConstraints(1, 1));
    
    addCommands(
      new InstantCommand(() -> {
        armSubsystem.updateGamePieceCone();
        armSubsystem.updateHeightHigh();
        armSubsystem.updateFudgeFalse();
      }, armSubsystem),
      new InstantCommand(() -> driveSubsystem.resetOdometry(path.getInitialHolonomicPose()), driveSubsystem),
      new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem),
      new InstantCommand(armSubsystem::updateDeployTrue, armSubsystem), 
      new WaitCommand(1.0).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem).repeatedly()), 
      // new InstantCommand(armSubsystem::resetArm2Encoder),
      new PPSwerveControllerCommand(
        path,
        driveSubsystem::getOdometryLocation,
        driveSubsystem.kinematics,
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        new PIDController(0, 0, 0),
        driveSubsystem::setModuleStates,
        false,
        driveSubsystem
      ).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem).repeatedly()), 
      new InstantCommand(() -> intakeSubsystem.setPower(-0.5), intakeSubsystem).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem).repeatedly()),
      new WaitCommand(0.5).deadlineWith(new InstantCommand(armSubsystem::setpointsFromStateMachine, armSubsystem).repeatedly()),
      new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
      new InstantCommand(armSubsystem::updateDeployFalse, armSubsystem)
    );
  }
}
