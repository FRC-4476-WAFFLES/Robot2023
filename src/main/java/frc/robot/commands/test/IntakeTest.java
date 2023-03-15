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

public class IntakeTest extends CommandBase {
  private final Timer timer = new Timer();
  private final PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry intakeCurrent = new DoubleLogEntry(log, "Intake Current");

  /** Creates a new IntakeTest. */
  public IntakeTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    log.resume();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 2) {
      intakeSubsystem.setPower(1.0);
    } else if (timer.get() < 4) {
      intakeSubsystem.setPower(-1.0);
    } else {
      intakeSubsystem.setPower(0.1);
    }

    intakeCurrent.append(pdh.getCurrent(Constants.intakePower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    log.pause();

    intakeCurrent.finish();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 6;
  }
}
