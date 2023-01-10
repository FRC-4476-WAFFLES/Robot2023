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
  // TODO: set these adresses
  public static final int swerveModule1Angle = -1; // FX Front Left 2
  public static final int swerveModule1Drive = -1; // FX Front Left 1
  public static final int swerveModule2Angle = -1; // FX Back Left 2
  public static final int swerveModule2Drive = -1; // FX Back Left 1
  public static final int swerveModule3Angle = -1; // FX Back Right 1
  public static final int swerveModule3Drive = -1; // FX Back Right 2
  public static final int swerveModule4Angle = -1; // FX Front Right 1
  public static final int swerveModule4Drive = -1; // FX Front Right 2
  public static final int swerveModule1Encoder = -1; // CTRE CANcoder
  public static final int swerveModule2Encoder = -1; // CTRE CANcoder
  public static final int swerveModule3Encoder = -1; // CTRE CANcoder
  public static final int swerveModule4Encoder = -1; // CTRE CANcoder

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

public static final SwerveConstants swerveModules[] = new SwerveConstants[] {
    // Modules are in the order of Front Left, Back Left, Back Right, Front Right, when intake is front of robot
    new SwerveConstants(new Translation2d(0.2921, 0.2921), 48.68, swerveModule1Angle, swerveModule1Drive, swerveModule1Encoder),
    new SwerveConstants(new Translation2d(-0.2921, 0.2921), 229.16, swerveModule2Angle, swerveModule2Drive, swerveModule2Encoder),
    new SwerveConstants(new Translation2d(-0.2921, -0.2921), 230.14, swerveModule3Angle, swerveModule3Drive, swerveModule3Encoder),
    new SwerveConstants(new Translation2d(0.2921, -0.2921), 267.42, swerveModule4Angle, swerveModule4Drive, swerveModule4Encoder),
};
}
