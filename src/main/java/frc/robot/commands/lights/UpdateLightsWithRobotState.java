// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.LightSubsystem.LightColours;

import static frc.robot.RobotContainer.*;

public class UpdateLightsWithRobotState extends CommandBase {
  /** Creates a new UpdateLightsWithRobotState. */
  public UpdateLightsWithRobotState() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isAutonomous() && DriverStation.getAlliance() == Alliance.Red) {
      lightSubsystem.setLightColour(LightColours.RED);
    } else if (DriverStation.isAutonomous() && DriverStation.getAlliance() == Alliance.Blue) {
      lightSubsystem.setLightColour(LightColours.BLUE);
    } else if (driveSubsystem.getLockWheels()) {
      lightSubsystem.setLightColour(LightColours.PINK);
    } else if (driveSubsystem.isAutoAiming() && Math.abs(camera.getFilteredHorizontal()) < Constants.DriveConstants.aimToleranceDegrees && camera.getHasTarget()) {
      lightSubsystem.setLightColour(LightColours.GREEN);
    } else if (driveSubsystem.isAutoAiming() && camera.getHasTarget()) {
      lightSubsystem.blinkBetweenColours(LightColours.RED, LightColours.BLACK);
    } else if (armSubsystem.getPiece() == GamePiece.CUBE) {
      lightSubsystem.blinkBetweenColours(LightColours.VIOLET, LightColours.BLACK);
    } else {
      lightSubsystem.blinkBetweenColours(LightColours.YELLOW, LightColours.BLACK);
    }
  }
}
