// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmTeleop;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.Mobility;
import frc.robot.commands.auto.MobilityAndBalance;
import frc.robot.commands.auto.OneConeAndBalance;
import frc.robot.commands.auto.OneConeAndMobilityAndBalance;
import frc.robot.commands.auto.OneConeAndOneCube;
import frc.robot.commands.auto.OneCube;
import frc.robot.commands.auto.OneCubeAndBalance;
import frc.robot.commands.auto.OneCubeAndMobilityAndBalance;
import frc.robot.commands.auto.OneCubeAndPickup;
import frc.robot.commands.auto.OneCubeAndPickupAndBalance;
import frc.robot.commands.auto.PathTest;
import frc.robot.commands.auto.TwoCube;
import frc.robot.commands.auto.TwoCubeAndBalance;
import frc.robot.commands.auto.TwoCubeAndPickup;
import frc.robot.commands.drive.DriveAutoBalance;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.IntakeTeleop;
import frc.robot.commands.lights.UpdateLightsWithRobotState;
import frc.robot.commands.test.MainTest;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

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
  public static final LightSubsystem lightSubsystem = new LightSubsystem();
  public static final Camera camera = new Camera();
  
  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final XboxController operate = new XboxController(2);

  private final DriveTeleop swerve = new DriveTeleop();
  private final ArmTeleop armJoystickControl = new ArmTeleop(operate::getLeftX, operate::getLeftY, operate::getRightX);
  private final IntakeTeleop intakeTeleop = new IntakeTeleop(() -> -operate.getLeftTriggerAxis() + operate.getRightTriggerAxis());
  private final UpdateLightsWithRobotState updateLights = new UpdateLightsWithRobotState();

  /** A map of events and their corresponding commands */
  private final HashMap<String, Command> eventMap = new HashMap<>() {{
    put("intakeRun", new IntakeTeleop(() -> 0.5));
    put("updateArmPieceCube", new InstantCommand(armSubsystem::updateGamePieceCube, armSubsystem));
    put("updateArmPieceCone", new InstantCommand(armSubsystem::updateGamePieceCone, armSubsystem));
    put("updateArmHeightLow", new InstantCommand(armSubsystem::updateHeightLow, armSubsystem));
    put("updateArmHeightMedium", new InstantCommand(armSubsystem::updateHeightMedium, armSubsystem));
    put("updateArmHeightHigh", new InstantCommand(armSubsystem::updateHeightHigh, armSubsystem));
    put("updateArmHeightPickup", new InstantCommand(armSubsystem::updateHeightPickup, armSubsystem));
    put("retractArm", new PrintCommand("Arm Retracted"));
    put("lockWheels", new InstantCommand(driveSubsystem::lockWheels, driveSubsystem));
    put("wait 1 sec", new WaitCommand(1));
    put("reachedPoint", new PrintCommand("reached Point"));
  }};

  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    driveSubsystem::getAdjustedPose, // Pose2d supplier
    driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    new PIDConstants(0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    driveSubsystem::setChassisSpeedsAuto, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
  );

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final Command mobility = new Mobility();
  private final Command balance = new Balance();
  private final Command mobilityAndBalance = new MobilityAndBalance();
  private final Command oneCube = new OneCube();
  private final Command oneCubeAndBalance = new OneCubeAndBalance();
  private final Command oneConeAndBalance = new OneConeAndBalance();
  private final Command oneCubeAndMobilityAndBalance = new OneCubeAndMobilityAndBalance();
  private final Command oneConeAndMobilityAndBalance = new OneConeAndMobilityAndBalance();
  private final Command oneCubeAndPickup = new OneCubeAndPickup();
  private final Command oneCubeAndPickupAndBalance = new OneCubeAndPickupAndBalance();
  private final Command twoCube = new TwoCube();
  private final Command oneConeAndOneCube = new OneConeAndOneCube();
  private final Command twoCubeAndBalance = new TwoCubeAndBalance();
  private final Command twoCubeAndPickup = new TwoCubeAndPickup();

  private final Command testAuto = new PathTest();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(swerve);
    armSubsystem.setDefaultCommand(armJoystickControl);
    intakeSubsystem.setDefaultCommand(intakeTeleop);
    lightSubsystem.setDefaultCommand(updateLights);
    // Configure the trigger bindings
    configureBindings();

    autoChooser.addOption("Mobility", mobility);
    autoChooser.addOption("Balance", balance);
    autoChooser.addOption("Mobility and Balance", mobilityAndBalance);
    autoChooser.addOption("1 Cube", oneCube);
    autoChooser.addOption("1 Cube and Balance", oneCubeAndBalance);
    autoChooser.addOption("1 Cone and Balance", oneConeAndBalance);
    autoChooser.addOption("1 Cube and Mobility and Balance", oneCubeAndMobilityAndBalance);
    autoChooser.addOption("1 Cone and Mobility and Balance", oneConeAndMobilityAndBalance);
    autoChooser.addOption("1 Cube and Pickup", oneCubeAndPickup);
    autoChooser.addOption("1 Cube and Pickup and Balance", oneCubeAndPickupAndBalance);
    autoChooser.addOption("2 Cube", twoCube);
    autoChooser.addOption("1 Cone and 1 Cube", oneConeAndOneCube);
    autoChooser.addOption("2 Cube and Balance", twoCubeAndBalance);
    autoChooser.addOption("2 Cube and Pickup", twoCubeAndPickup);
    autoChooser.addOption("Test Auto", testAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    final var left1 = new JoystickButton(leftJoystick, 1);

    final var aButton = new JoystickButton(operate, XboxController.Button.kA.value);
    final var bButton = new JoystickButton(operate, XboxController.Button.kB.value);
    final var xButton = new JoystickButton(operate, XboxController.Button.kX.value);
    final var yButton = new JoystickButton(operate, XboxController.Button.kY.value);
    // final var leftStickButton = new JoystickButton(operate, XboxController.Button.kLeftStick.value);
    // final var rightStickButton = new JoystickButton(operate, XboxController.Button.kRightStick.value);
    // final var leftBumperButton = new JoystickButton(operate, XboxController.Button.kLeftBumper.value);
    final var rightBumperButton = new JoystickButton(operate, XboxController.Button.kRightBumper.value);
    // final var backButton = new JoystickButton(operate, XboxController.Button.kBack.value);
    // final var startButton = new JoystickButton(operate, XboxController.Button.kStart.value);
    // final var dpadDown = new Trigger(() -> operate.getPOV() == 180);
    final var dpadUp = new Trigger(() -> operate.getPOV() == 0);

    right1.onTrue(new InstantCommand(driveSubsystem::resetSteerEncoders, driveSubsystem).alongWith(new InstantCommand(driveSubsystem::resetGyro)));
    // left1.whileTrue(new DriveToScoreLimelight());
    left1.whileTrue(new DriveAutoBalance());

    aButton.onTrue(new InstantCommand(armSubsystem::updateHeightLow, armSubsystem));
    bButton.onTrue(new InstantCommand(armSubsystem::updateHeightMedium, armSubsystem));
    yButton.onTrue(new InstantCommand(armSubsystem::updateHeightHigh, armSubsystem));
    xButton.onTrue(new InstantCommand(armSubsystem::updateHeightPickup, armSubsystem));
    rightBumperButton.onTrue(new InstantCommand(armSubsystem::togglePiece, armSubsystem));
    dpadUp.onTrue(new InstantCommand(armSubsystem::updateFudgeTrue, armSubsystem));
    dpadUp.onFalse(new InstantCommand(armSubsystem::updateFudgeFalse, armSubsystem));

    aButton.or(bButton).or(xButton).or(yButton).or(armSubsystem::getFudge).onTrue(new InstantCommand(armSubsystem::updateDeployTrue)).onFalse(new InstantCommand(armSubsystem::updateDeployFalse));

    SmartDashboard.putData("Reset Arm Encoders", new InstantCommand(armSubsystem::resetEncoders, armSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getTestCommand() {
    return new MainTest();
  }
}
