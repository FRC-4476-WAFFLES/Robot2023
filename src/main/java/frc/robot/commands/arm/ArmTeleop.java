// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem.ArmState.Height;

import static frc.robot.RobotContainer.*;

public class ArmTeleop extends CommandBase {
  private final Supplier<Double> joystickAxis1;
  private final Supplier<Double> joystickAxis2;
  private final Supplier<Double> joystickAxis3;

  /** Creates a new Arm1JoystickControl. */
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
      armSubsystem.fudgeArm1WithAnalogStick(MathUtil.applyDeadband(joystickAxis1.get(), 0.05));
      armSubsystem.fudgeArm2WithAnalogStick(MathUtil.applyDeadband(joystickAxis2.get(), 0.05));
      armSubsystem.fudgeIntakeWithAnalogStick(MathUtil.applyDeadband(joystickAxis3.get(), 0.05));
    } else {
      armSubsystem.setpointsFromStateMachine();
      if (intakeSubsystem.getPower() > 0.1 && armSubsystem.getHeight() == Height.HPPICKUP) {
        armSubsystem.fudgeArm2Setpoint(20000);
      }
    }

    // Pose2d currentPose = driveSubsystem.getOdometryLocation();
    // if (currentPose.getX() < 8) {
    //   if (Math.abs(currentPose.getRotation().getDegrees()) < 90) {
    //     armSubsystem.updateSideFront();
    //   } else {
    //     armSubsystem.updateSideBack();
    //   }
    // } else {
    //   if (Math.abs(currentPose.getRotation().getDegrees()) < 90) {
    //     armSubsystem.updateSideBack();
    //   } else {
    //     armSubsystem.updateSideFront();
    //   }
    // }
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
