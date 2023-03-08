// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // CAN bus
  public static final int arm1Left = 9; // Talon FX
  public static final int arm1Right = 11; // Talon FX
  public static final int arm2Left = 10; // Talon FX
  public static final int arm2Right = 12; // Talon FX
  public static final int intakePivotLeft = 14; // Spark MAX
  public static final int intakePivotRight = 13; // Spark MAX
  public static final int intake = 16; // Spark MAX

  public static final int swerveModuleBLAngle = 18; // FX Back Left 2
  public static final int swerveModuleBLDrive = 21; // FX Back Left 1
  public static final int swerveModuleFLAngle = 25; // FX Front Left 2
  public static final int swerveModuleFLDrive = 23; // FX Front Left 1
  public static final int swerveModuleFRAngle = 19; // FX Front Right 1
  public static final int swerveModuleFRDrive = 17; // FX Front Right 2
  public static final int swerveModuleBRAngle = 24; // FX Back Right 1
  public static final int swerveModuleBRDrive = 22; // FX Back Right 2

  // PDH channels
  public static final int module0AnglePower = 0; // FX Front Left 2
  public static final int module0DrivePower = 1; // FX Front Left 1
  public static final int module1AnglePower = 19; // FX Back Left 2
  public static final int module1DrivePower = 18; // FX Back Left 1
  public static final int module2AnglePower = 11; // FX Back Right 1
  public static final int module2DrivePower = 10; // FX Back Right 2
  public static final int module3AnglePower = 9; // FX Front Right 1
  public static final int module3DrivePower = 8; // FX Front Right 2
  public static final int arm1LeftPower = -1;
  public static final int arm1RightPower = -1;
  public static final int arm2LeftPower = -1;
  public static final int arm2RightPower = -1;
  public static final int intakePivotLeftPower = -1;
  public static final int intakePivotRightPower = -1;
  public static final int intakePower = -1;

  public static final class SwerveConstants {
    /** Represents the offset from the centre of the robot, in metres. */
    public final Translation2d position;

    /**
     * Stores the angle offset of this particular swerve module, in degrees. This
     * will be used to compensate for the different "zero" angles of the encoders.
     * 
     * To calibrate this value, manually rotate each module to be facing the same
     * direction. When they are all aligned,
     */
    public final double calibration;

    /** The CAN address of the module's angle motor. */
    public final int angleMotor;
    /** The CAN address of the module's drive motor. */
    public final int driveMotor;

    public final int angleEncoder;

    public final double CPR = 2048; // Encoder ticks per motor rotation
    public final double wheelDiameter = 0.1016; // Wheel diameter in meters
    public final double wheelCircumfrence = wheelDiameter * Math.PI; // Wheel circumfrence in meters

    public final double firstStageRatio = 14.0/50.0;
    public final double secondStageRatio = 28.0/16.0;
    public final double thirdStageRatio = 15.0/60.0;
    public final double driveOverallRatio = 1.0 / (firstStageRatio * secondStageRatio * thirdStageRatio); // Drive gear ratio

    public final double steeringRatio = 12.8;

    public final double steeringToDriveRatio = 1.0 / (firstStageRatio * secondStageRatio * steeringRatio);
    
    public final double metersPerSecondToTicksPer100ms = CPR * driveOverallRatio / wheelCircumfrence / 10.0;

    public static final double maxAttainableSpeedMetersPerSecond = 4.1;
    public static final double maxAttainableRotationRateRadiansPerSecond = 8.0;
    public static final double maxAccelerationMetersPerSecondSquared = 2.9;

    public static final double aimToleranceDegrees = 1.5; // Tolerance of drive aiming, in degrees

    // The number of ticks of the motor's built-in encoder per revolution of the steering module
    public final double ticksPerSteeringRevolution = 26214.4;
    // Convert degrees to motor ticks
    public final double steeringDegreesToTicks = ticksPerSteeringRevolution / 360.0;

    public SwerveConstants(Translation2d position, double calibration, int angleMotor, int driveMotor, int angleEncoder) {
      this.position = position;
      this.calibration = calibration;
      this.angleMotor = angleMotor;
      this.driveMotor = driveMotor;
      this.angleEncoder = angleEncoder;
    }
  }

  public static final class ArmConstants {
    public static final double a1 = 1.0;
    public static final double a2 = 1.0;
    public static final double a3 = 0.1;

    public final double calibration;
    public final int motorID;
    public final double ratio;
    public final int currentLimit;
    public final boolean isInverted;

    public ArmConstants(double calibration, int motorID, double ratio, int currentLimit, boolean isInverted) {
      this.calibration = calibration;
      this.motorID = motorID;
      this.ratio = ratio;
      this.currentLimit = currentLimit;
      this.isInverted = isInverted;
    }
  }

  // public static final SwerveConstants swerveModules[] = new SwerveConstants[] {
  //   // Modules are in the order of Front Left, Back Left, Back Right, Front Right, when intake is front of robot
  //   new SwerveConstants(new Translation2d(0.2921, 0.2921), 48.68, swerveModule1Angle, swerveModule1Drive, swerveModule1Encoder),
  //   new SwerveConstants(new Translation2d(-0.2921, 0.2921), 229.16, swerveModule2Angle, swerveModule2Drive, swerveModule2Encoder),
  //   new SwerveConstants(new Translation2d(-0.2921, -0.2921), 230.14, swerveModule3Angle, swerveModule3Drive, swerveModule3Encoder),
  //   new SwerveConstants(new Translation2d(0.2921, -0.2921), 267.42, swerveModule4Angle, swerveModule4Drive, swerveModule4Encoder),
  // };

  // public static final ArmConstants armMotors[] = new ArmConstants[] {
  //   new ArmConstants(0.0, armShoulder1, (1 / 80) * (15 / 36), 40, false),
  //   new ArmConstants(0.0, armShoulder2, (1 / 80) * (15 / 36), 40, true),
  //   new ArmConstants(0.0, armElbow1, (1 / 80), 40, false),
  //   new ArmConstants(0.0, armElbow2, (1 / 80), 40, true),
  //   new ArmConstants(0.0, armWrist1, (1 / 1) * (16 / 36), 20, false),
  //   new ArmConstants(0.0, armWrist2, (1 / 1) * (16 / 36), 20, true),
  // };
}
