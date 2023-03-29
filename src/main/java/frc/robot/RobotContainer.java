// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import frc.robot.commands.drive.DriveToScoreLimelight;
import frc.robot.commands.drive.DriveTurnToNearest90;
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

  private final DriveTeleop driveTeleop = new DriveTeleop();
  private final ArmTeleop armTeleop = new ArmTeleop(operate::getLeftX, operate::getLeftY, operate::getRightX);
  private final IntakeTeleop intakeTeleop = new IntakeTeleop(() -> -operate.getLeftTriggerAxis() + operate.getRightTriggerAxis());
  private final UpdateLightsWithRobotState updateLights = new UpdateLightsWithRobotState();

  /** A map of events and their corresponding commands */
  private final HashMap<String, Command> eventMap = new HashMap<>() {{
    put("armUpdateCubeLow", new InstantCommand(() -> {armSubsystem.updateGamePieceCube(); armSubsystem.updateHeightLow();}, armSubsystem));
    put("armUpdateCubeMedium", new InstantCommand(() -> {armSubsystem.updateGamePieceCube(); armSubsystem.updateHeightMedium();}, armSubsystem));
    put("armUpdateCubeHigh", new InstantCommand(() -> {armSubsystem.updateGamePieceCube(); armSubsystem.updateHeightHigh();}, armSubsystem));
    put("armUpdateConeLow", new InstantCommand(() -> {armSubsystem.updateGamePieceCone(); armSubsystem.updateHeightLow();}, armSubsystem));
    put("armUpdateConeMedium", new InstantCommand(() -> {armSubsystem.updateGamePieceCone(); armSubsystem.updateHeightMedium();}, armSubsystem));
    put("armUpdateConeHigh", new InstantCommand(() -> {armSubsystem.updateGamePieceCone(); armSubsystem.updateHeightHigh();}, armSubsystem));
    put("armUpdateDeployTrue", new InstantCommand(armSubsystem::updateDeployTrue, armSubsystem));
    put("armUpdateDeployFalse", new InstantCommand(armSubsystem::updateDeployFalse, armSubsystem));

    put("driveUpdateLockWheelsTrue", new InstantCommand(driveSubsystem::updateLockWheelsTrue, driveSubsystem));
    put("driveUpdateLockWheelsFalse", new InstantCommand(driveSubsystem::updateLockWheelsFalse, driveSubsystem));
    put("driveTurnToNearest90", new DriveTurnToNearest90(false));
    put("driveAutoBalance", new DriveAutoBalance());

    put("intakeSetPower(-0.3)", new InstantCommand(() -> intakeSubsystem.setPower(-0.3)));
    put("intakeSetPower(0.0)", new InstantCommand(() -> intakeSubsystem.setPower(0.0)));
    put("intakeSetPower(0.1)", new InstantCommand(() -> intakeSubsystem.setPower(0.1)));
    put("intakeSetPower(1.0)", new InstantCommand(() -> intakeSubsystem.setPower(1.0)));

    put("wait(0.1)", new WaitCommand(0.1));
    put("wait(0.5)", new WaitCommand(0.5));
  }};

  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    driveSubsystem::getAdjustedPose, // Pose2d supplier
    driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    new PIDConstants(2, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    driveSubsystem::setChassisSpeedsAuto, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    driveSubsystem, // The drive subsystem. Used to properly set the requirements of path following commands
    intakeSubsystem // Require the intake subsystem to prevent the default command from overriding the auto commands
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
  private final Command fullAutoTest = autoBuilder.fullAuto(PathPlanner.loadPathGroup("Rotation Test", new PathConstraints(3, 2)));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveTeleop);
    armSubsystem.setDefaultCommand(armTeleop);
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
    autoChooser.addOption("Test Full Auto", fullAutoTest);
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
    final var right14 = new JoystickButton(rightJoystick, 14);
    final var left1 = new JoystickButton(leftJoystick, 1);
    final var left2 = new JoystickButton(leftJoystick, 2);

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

    right1.whileTrue(new DriveTurnToNearest90());
    left1.whileTrue(new DriveToScoreLimelight());
    left2.toggleOnTrue(new StartEndCommand(driveSubsystem::updateLockWheelsTrue, driveSubsystem::updateLockWheelsFalse, driveSubsystem));
    right14.onTrue(new InstantCommand(driveSubsystem::resetSteerEncoders, driveSubsystem).alongWith(new InstantCommand(driveSubsystem::resetGyro)));
    // left1.whileTrue(new DriveAutoBalance());

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

  // Automatically unlock the wheels at the start of teleop
  public Command getTeleopInitCommand() {
    return new InstantCommand(driveSubsystem::updateLockWheelsFalse, driveSubsystem);
  }
}
