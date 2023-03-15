// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class ArmTest extends CommandBase {
  private final Timer timer = new Timer();
  private final PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry arm1LeftCurrent = new DoubleLogEntry(log, "Arm 1 Left Current");
  private final DoubleLogEntry arm1RightCurrent = new DoubleLogEntry(log, "Arm 1 Right Current");
  private final DoubleLogEntry arm2LeftCurrent = new DoubleLogEntry(log, "Arm 2 Left Current");
  private final DoubleLogEntry arm2RightCurrent = new DoubleLogEntry(log, "Arm 2 Right Current");
  private final DoubleLogEntry intakePivotLeftCurrent = new DoubleLogEntry(log, "Intake Pivot Left Current");
  private final DoubleLogEntry intakePivotRightCurrent = new DoubleLogEntry(log, "Intake Pivot Right Current");

  /** Creates a new ArmTest. */
  public ArmTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    log.resume();
    armSubsystem.updateFudgeFalse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 3) {
      armSubsystem.updateGamePieceCube();
      armSubsystem.updateHeightHigh();
      armSubsystem.updateDeployTrue();
    } else if (timer.get() < 6) {
      armSubsystem.updateDeployFalse();
    } else if (timer.get() < 9) {
      armSubsystem.updateGamePieceCone();
      armSubsystem.updateHeightMedium();
      armSubsystem.updateDeployTrue();
    } else if (timer.get() < 12) {
      armSubsystem.updateDeployFalse();
    } else if (timer.get() < 15) {
      armSubsystem.updateGamePieceCone();
      armSubsystem.updateHeightLow();
      armSubsystem.updateDeployTrue();
    } else if (timer.get() < 18) {
      armSubsystem.updateDeployFalse();
    } else if (timer.get() < 21) {
      armSubsystem.updateGamePieceCube();
      armSubsystem.updateHeightPickup();
      armSubsystem.updateDeployTrue();
    } else {
      armSubsystem.updateDeployFalse();
    }

    armSubsystem.setpointsFromStateMachine();

    arm1LeftCurrent.append(pdh.getCurrent(Constants.arm1LeftPower));
    arm1RightCurrent.append(pdh.getCurrent(Constants.arm1RightPower));
    arm2LeftCurrent.append(pdh.getCurrent(Constants.arm2LeftPower));
    arm2RightCurrent.append(pdh.getCurrent(Constants.arm2RightPower));
    intakePivotLeftCurrent.append(pdh.getCurrent(Constants.intakePivotLeftPower));
    intakePivotRightCurrent.append(pdh.getCurrent(Constants.intakePivotRightPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
    log.pause();

    arm1LeftCurrent.finish();
    arm1RightCurrent.finish();
    arm2LeftCurrent.finish();
    arm2RightCurrent.finish();
    intakePivotLeftCurrent.finish();
    intakePivotRightCurrent.finish();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 24;
  }
}
