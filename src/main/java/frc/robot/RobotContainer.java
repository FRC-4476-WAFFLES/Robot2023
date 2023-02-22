// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmJoystickControl;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.IntakeTeleop;
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
  private final ArmJoystickControl armJoystickControl = new ArmJoystickControl(operate::getLeftX, operate::getLeftY, operate::getRightX);
  private final IntakeTeleop intakeTeleop = new IntakeTeleop(() -> -operate.getLeftTriggerAxis() + operate.getRightTriggerAxis());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(swerve);
    armSubsystem.setDefaultCommand(armJoystickControl);
    intakeSubsystem.setDefaultCommand(intakeTeleop);
    //CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
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
    final var right1 = new JoystickButton(rightJoystick, 1);

    final var aButton = new JoystickButton(operate, XboxController.Button.kA.value);
    final var bButton = new JoystickButton(operate, XboxController.Button.kB.value);
    final var xButton = new JoystickButton(operate, XboxController.Button.kX.value);
    final var yButton = new JoystickButton(operate, XboxController.Button.kY.value);
    final var leftStickButton = new JoystickButton(operate, XboxController.Button.kLeftStick.value);
    final var rightStickButton = new JoystickButton(operate, XboxController.Button.kRightStick.value);
    final var leftBumperButton = new JoystickButton(operate, XboxController.Button.kLeftBumper.value);
    final var rightBumperButton = new JoystickButton(operate, XboxController.Button.kRightBumper.value);
    final var dpadDown = new Trigger(() -> operate.getPOV() == 180);
    final var dpadUp = new Trigger(() -> operate.getPOV() == 0);

    right1.onTrue(new InstantCommand(driveSubsystem::resetSteerEncoders, driveSubsystem));

    aButton.onTrue(new InstantCommand(armSubsystem::updateHeightLow, armSubsystem));
    bButton.onTrue(new InstantCommand(armSubsystem::updateHeightMedium, armSubsystem));
    yButton.onTrue(new InstantCommand(armSubsystem::updateHeightHigh, armSubsystem));
    xButton.onTrue(new InstantCommand(armSubsystem::updateHeightPickup, armSubsystem));
    leftStickButton.onTrue(new InstantCommand(armSubsystem::updateSideLeft, armSubsystem));
    rightStickButton.onTrue(new InstantCommand(armSubsystem::updateSideRight, armSubsystem));
    leftBumperButton.onTrue(new InstantCommand(armSubsystem::updateGamePieceCone, armSubsystem));
    rightBumperButton.onTrue(new InstantCommand(armSubsystem::updateGamePieceCube, armSubsystem));
    dpadDown.onTrue(new InstantCommand(armSubsystem::updateAltPickuplTrue, armSubsystem));
    dpadUp.onTrue(new InstantCommand(() -> {armSubsystem.updateFudgeJoint(true);}, armSubsystem));
    dpadUp.onFalse(new InstantCommand(() -> {armSubsystem.updateFudgeJoint(false);}, armSubsystem));
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
