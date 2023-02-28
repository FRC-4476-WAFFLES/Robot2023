// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutoBalance;

import static frc.robot.RobotContainer.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCubeAndBalance extends SequentialCommandGroup {
  /** Creates a new OnePieceAndBalance. */
  public OneCubeAndBalance() {
    PathPlannerTrajectory path = PathPlanner.loadPath("1 Cube Climb", new PathConstraints(1, 1));

    addCommands(
      new InstantCommand(() -> {
        armSubsystem.updateGamePieceCube();
        armSubsystem.updateHeightHigh();
        armSubsystem.updateSideFront();
      }, armSubsystem),

      new AutoPlaceGamepiece(),

      new InstantCommand(() -> driveSubsystem.resetOdometry(path.getInitialHolonomicPose()), driveSubsystem),

      new PPSwerveControllerCommand(
        path,
        driveSubsystem::getOdometryLocation,
        driveSubsystem.kinematics,
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        new PIDController(0, 0, 0),
        driveSubsystem::setModuleStates,
        true,
        driveSubsystem
      ).until(() -> Math.abs(driveSubsystem.getPitch()) > 5),

      new DriveAutoBalance(),

      new InstantCommand(() -> driveSubsystem.lockWheels(), driveSubsystem).repeatedly()
    );
  }
}
