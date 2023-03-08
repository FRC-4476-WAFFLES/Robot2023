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

public class DriveTest extends CommandBase {
  private final Timer timer = new Timer();
  private final PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry module0DriveCurrent = new DoubleLogEntry(log, "Module 0 Drive Current");
  private final DoubleLogEntry module0AngleCurrent = new DoubleLogEntry(log, "Module 0 Angle Current");
  private final DoubleLogEntry module1DriveCurrent = new DoubleLogEntry(log, "Module 1 Drive Current");
  private final DoubleLogEntry module1AngleCurrent = new DoubleLogEntry(log, "Module 1 Angle Current");
  private final DoubleLogEntry module2DriveCurrent = new DoubleLogEntry(log, "Module 2 Drive Current");
  private final DoubleLogEntry module2AngleCurrent = new DoubleLogEntry(log, "Module 2 Angle Current");
  private final DoubleLogEntry module3DriveCurrent = new DoubleLogEntry(log, "Module 3 Drive Current");
  private final DoubleLogEntry module3AngleCurrent = new DoubleLogEntry(log, "Module 3 Angle Current");
  
  // private double maxCurrent = 0.0;

  /** Creates a new DriveTest. */
  public DriveTest() {
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetSteerEncoders();
    timer.reset();
    log.resume();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 5) {
      driveSubsystem.robotDrive(3, 0, 0, false);
    } else if(timer.get() < 10) {
      driveSubsystem.robotDrive(-3, 0, 0, false);
    } else if(timer.get() < 15) {
      driveSubsystem.robotDrive(2, 2, 0, false);
    } else if(timer.get() < 20) {
      driveSubsystem.robotDrive(-2, -2, 0, false);
    } else if(timer.get() < 25) {
      driveSubsystem.robotDrive(0, 3, 0, false);
    } else if(timer.get() < 30) {
      driveSubsystem.robotDrive(0, -3, 0, false);
    } else if(timer.get() < 35) {
      driveSubsystem.robotDrive(-2, 2, 0, false);
    } else if(timer.get() < 40) {
      driveSubsystem.robotDrive(2, -2, 0, false);
    } else if(timer.get() < 45) {
      driveSubsystem.robotDrive(0, 0, 4, false);
    } else if(timer.get() < 50) {
      driveSubsystem.robotDrive(0, 0, -4, false);
    } else if(timer.get() < 55) {
      double heading = (timer.get() - 50.0) / 5.0 * 2.0 * Math.PI; // "Translate" in a circle over 5 seconds
      driveSubsystem.robotDrive(Math.cos(heading), Math.sin(heading), 0, false);
    }

    module0DriveCurrent.append(pdh.getCurrent(Constants.module0DrivePower));
    module0AngleCurrent.append(pdh.getCurrent(Constants.module0AnglePower));
    module1DriveCurrent.append(pdh.getCurrent(Constants.module1DrivePower));
    module1AngleCurrent.append(pdh.getCurrent(Constants.module1AnglePower));
    module2DriveCurrent.append(pdh.getCurrent(Constants.module2DrivePower));
    module2AngleCurrent.append(pdh.getCurrent(Constants.module2AnglePower));
    module3DriveCurrent.append(pdh.getCurrent(Constants.module3DrivePower));
    module3AngleCurrent.append(pdh.getCurrent(Constants.module3AnglePower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    log.pause();

    module0DriveCurrent.finish();
    module0AngleCurrent.finish();
    module1DriveCurrent.finish();
    module1AngleCurrent.finish();
    module2DriveCurrent.finish();
    module2AngleCurrent.finish();
    module3DriveCurrent.finish();
    module3AngleCurrent.finish();

    // try {
    //   DataLogReader reader = new DataLogReader("null");
    //   DataLogIterator iterator = reader.iterator();
    //   DataLogRecord record = iterator.next();
    //   record.getEntry();
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 55;
  }
}
