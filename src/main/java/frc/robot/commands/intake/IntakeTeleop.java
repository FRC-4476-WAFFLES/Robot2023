// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.RobotContainer.*;

import java.util.function.Supplier;

public class IntakeTeleop extends CommandBase {
  private final Supplier<Double> powerSupplier;

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
    double leftPower = powerSupplier.get();
    double rightPower = powerSupplier.get();

    if (armSubsystem.getPiece() == ArmSubsystem.ArmState.GamePiece.CONE) {
      rightPower *= -1.0;
    }

    if (armSubsystem.getSide() == ArmSubsystem.ArmState.Side.RIGHT) {
      leftPower *= -1.0;
      rightPower *= -1.0;
    }

    intakeSubsystem.setLeftPower(leftPower);
    intakeSubsystem.setRightPower(rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
