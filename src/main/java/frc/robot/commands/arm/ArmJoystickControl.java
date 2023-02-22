// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class ArmJoystickControl extends CommandBase {
  private final Supplier<Double> joystick;
  private final Supplier<Double> joystick2;

  /** Creates a new Arm1JoystickControl. */
  public ArmJoystickControl(Supplier<Double> joystick, Supplier<Double> joystick2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick;
    this.joystick2 = joystick2;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.moveArm1WithAnalogStick(joystick.get());
    armSubsystem.moveArm2WithAnalogStick(joystick2.get());
    // armSubsystem.easyRun(joystick2.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
