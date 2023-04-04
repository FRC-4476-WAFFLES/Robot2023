// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Height;

import static frc.robot.RobotContainer.*;

public class ArmTeleop extends CommandBase {
  private final Supplier<Double> joystickAxis1;
  private final Supplier<Double> joystickAxis2;
  private final Supplier<Double> joystickAxis3;

  /** Creates a new ArmTeleop. */
  public ArmTeleop(Supplier<Double> joystickAxis1, Supplier<Double> joystickAxis2, Supplier<Double> joystickAxis3) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystickAxis1 = joystickAxis1;
    this.joystickAxis2 = joystickAxis2;
    this.joystickAxis3 = joystickAxis3;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armSubsystem.getFudge()) {
      armSubsystem.fudgeShoulderWithAnalogStick(MathUtil.applyDeadband(joystickAxis1.get(), 0.05));
      armSubsystem.fudgeElbowWithAnalogStick(MathUtil.applyDeadband(joystickAxis2.get(), 0.05));
      armSubsystem.fudgeWristWithAnalogStick(MathUtil.applyDeadband(joystickAxis3.get(), 0.05));
    } else {
      armSubsystem.setpointsFromStateMachine();
      if (intakeSubsystem.getPower() > 0.1 && armSubsystem.getHeight() == Height.PICKUP_SHELF) {
        armSubsystem.fudgeElbowSetpoint(40000);
      }
    }
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
