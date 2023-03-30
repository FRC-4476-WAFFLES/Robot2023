// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.disabled;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateLiveAuto extends CommandBase {
  private boolean autoHasBeenReloaded = false;
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
    // TODO: check names of smartdashboard keys
    boolean reloadAuto = SmartDashboard.getBoolean("Live Auto Robot Reload", false);
    if (reloadAuto && !autoHasBeenReloaded) {
      try {
        String pathName = SmartDashboard.getString("Live Auto Name", "Live Auto");
        Path filePath = new File(Filesystem.getDeployDirectory(), "pathplanner/" + pathName + ".path").toPath();

        String currentAutoPath = Files.readString(filePath);

        Files.writeString(filePath, SmartDashboard.getString("Live Auto Path", currentAutoPath));

        autoHasBeenReloaded = true;

        System.err.println(Files.readString(filePath)); // For debugging, print out the updated auto path
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    if (!reloadAuto && autoHasBeenReloaded) {
      autoHasBeenReloaded = false;
    }

    SmartDashboard.putBoolean("Live Auto Robot Has Reloaded", autoHasBeenReloaded);
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
