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
  private final DoubleLogEntry shoulderLeftCurrent = new DoubleLogEntry(log, "Shoulder Left Current");
  private final DoubleLogEntry shoulderRightCurrent = new DoubleLogEntry(log, "Shoulder Right Current");
  private final DoubleLogEntry elbowLeftCurrent = new DoubleLogEntry(log, "Elbow Left Current");
  private final DoubleLogEntry elbowRightCurrent = new DoubleLogEntry(log, "Elbow Right Current");
  private final DoubleLogEntry wristLeftCurrent = new DoubleLogEntry(log, "Wrist Left Current");
  private final DoubleLogEntry wristRightCurrent = new DoubleLogEntry(log, "Wrist Right Current");

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
      armSubsystem.updateHeightScoreHigh();
      armSubsystem.updateDeployTrue();
    } else if (timer.get() < 6) {
      armSubsystem.updateDeployFalse();
    } else if (timer.get() < 9) {
      armSubsystem.updateGamePieceCone();
      armSubsystem.updateHeightScoreMedium();
      armSubsystem.updateDeployTrue();
    } else if (timer.get() < 12) {
      armSubsystem.updateDeployFalse();
    } else if (timer.get() < 15) {
      armSubsystem.updateGamePieceCone();
      armSubsystem.updateHeightScoreLow();
      armSubsystem.updateDeployTrue();
    } else if (timer.get() < 18) {
      armSubsystem.updateDeployFalse();
    } else if (timer.get() < 21) {
      armSubsystem.updateGamePieceCube();
      armSubsystem.updateHeightPickupShelf();
      armSubsystem.updateDeployTrue();
    } else {
      armSubsystem.updateDeployFalse();
    }

    armSubsystem.setpointsFromStateMachine();

    shoulderLeftCurrent.append(pdh.getCurrent(Constants.shoulderLeftPower));
    shoulderRightCurrent.append(pdh.getCurrent(Constants.shoulderRightPower));
    elbowLeftCurrent.append(pdh.getCurrent(Constants.elbowLeftPower));
    elbowRightCurrent.append(pdh.getCurrent(Constants.elbowRightPower));
    wristLeftCurrent.append(pdh.getCurrent(Constants.wristLeftPower));
    wristRightCurrent.append(pdh.getCurrent(Constants.wristRightPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
    log.pause();

    shoulderLeftCurrent.finish();
    shoulderRightCurrent.finish();
    elbowLeftCurrent.finish();
    elbowRightCurrent.finish();
    wristLeftCurrent.finish();
    wristRightCurrent.finish();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 24;
  }
}
