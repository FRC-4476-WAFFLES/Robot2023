// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.disabled;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateLiveAuto extends CommandBase {
  /** Creates a new UpdateLiveAuto. */
  public UpdateLiveAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Check if the command should update the auto path, then do so if it should
    // Path filePath = Path.of("path to file location");

    // Files.writeString(filePath, path from smartdashboard);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
