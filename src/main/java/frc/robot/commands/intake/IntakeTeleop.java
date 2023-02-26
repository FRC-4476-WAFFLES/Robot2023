// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import java.util.function.Supplier;

public class IntakeTeleop extends CommandBase {
  private final Supplier<Double> powerSupplier;
  private boolean hasBeenIntaking = false;

  /** Creates a new IntakeTeleop. */
  public IntakeTeleop(Supplier<Double> powerSupplier) {
    this.powerSupplier = powerSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickValue = MathUtil.applyDeadband(powerSupplier.get(), 0.05);

    if (joystickValue > 0) {
      hasBeenIntaking = true;
    }

    if (joystickValue < 0) {
      hasBeenIntaking = false;
    }

    double power;
    if (hasBeenIntaking && joystickValue == 0) {
      power = 0.1;
    } else {
      power = joystickValue;
    }
 
    intakeSubsystem.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
