// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final XboxController operate = new XboxController(2);

  private final DriveTeleop swerve = new DriveTeleop();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(swerve);
    CommandScheduler.getInstance().registerSubsystem(armSubsystem);
    CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    final var rightToggle = new JoystickButton(rightJoystick, 1);
    final var aButton = new JoystickButton(operate, 1);
    final var bButton = new JoystickButton(operate, 2);
    final var xButton = new JoystickButton(operate, 3);
    final var yButton = new JoystickButton(operate, 4);
    final var oneButton = new JoystickButton(operate, 9);
    final var twoButton = new JoystickButton(operate, 10);
    final var leftBumperButton = new JoystickButton(operate, 5);
    final var rightBumperButton = new JoystickButton(operate, 6);

    rightToggle.onTrue(new InstantCommand(driveSubsystem::resetSteerEncoders, driveSubsystem));
    aButton.onTrue(new InstantCommand(armSubsystem::updateHeightLow, armSubsystem));
    bButton.onTrue(new InstantCommand(armSubsystem::updateHeightMedium, armSubsystem));
    yButton.onTrue(new InstantCommand(armSubsystem::updateHeightHigh, armSubsystem));
    xButton.onTrue(new InstantCommand(armSubsystem::updateHeightPickup, armSubsystem));
    oneButton.onTrue(new InstantCommand(armSubsystem::updateSideLeft, armSubsystem));
    twoButton.onTrue(new InstantCommand(armSubsystem::updateSideRight, armSubsystem));
    leftBumperButton.onTrue(new InstantCommand(armSubsystem::updateGamePieceCone, armSubsystem));
    rightBumperButton.onTrue(new InstantCommand(armSubsystem::updateGamePieceCube, armSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
