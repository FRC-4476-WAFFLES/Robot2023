// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class DriveAutoBalance extends CommandBase {
  private final PIDController pitchController = new PIDController(1.25, 0, 0.0);
  private final PIDController yawController = new PIDController(-4.0, 0, -0);

  private double pitchRate = 0;
  private double lastPitch = 0;

  /** Creates a new DriveAutoBalance. */
  public DriveAutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchController.setSetpoint(0);
    yawController.setSetpoint(0);

    pitchController.setTolerance(0.04); // This is 2.29 degrees, aka legally level. 

    lastPitch = driveSubsystem.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = driveSubsystem.getPitch();
    pitchRate = (currentPitch - lastPitch) / 0.02;

    double forwardSpeed = pitchController.calculate(Math.toRadians(currentPitch));
    double rotationSpeed = yawController.calculate(driveSubsystem.getOdometryLocation().getRotation().getRadians());

    driveSubsystem.robotDrive(forwardSpeed, 0, rotationSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pitchController.atSetpoint() && Math.abs(pitchRate) < 2.0;
  }
}
