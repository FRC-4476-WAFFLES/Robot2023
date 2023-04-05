// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTurnToSpecificAngle extends PIDCommand {
  double targetHeading;
  /** Creates a new DriveTurnToSpecificAngle. */
  public DriveTurnToSpecificAngle(double targetHeading) {
    super(
      // The controller that the command will use
      new PIDController(-4, 0, 0),
      // This should return the measurement
      () -> driveSubsystem.getOdometryLocation().getRotation().getRadians(),
      // This should return the setpoint (can also be a constant)
      Math.toRadians(targetHeading),
      // This uses the output
      output -> {
        driveSubsystem.robotDrive(0, 0, output, false);
      }
    );
    this.targetHeading = targetHeading;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getOdometryLocation().getRotation().getDegrees() - targetHeading) < 2.0;
  }
}
