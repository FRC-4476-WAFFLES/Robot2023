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
  public static final int climb1Left = 9; // Spark MAX
  public static final int climb1Right = 11; // Spark MAX
  public static final int climb2Left = 10; // Spark MAX
  public static final int climb2Right = 12; // Spark MAX
  public static final int intakePivotLeft = 13; // Spark MAX
  public static final int intakePivotRight = 14; // Spark MAX
  public static final int intakeLeft = 15; // Spark MAX
  public static final int intakeRight = 16; // Spark MAX

  public static final int swerveModuleBLAngle = 18; // FX Back Left 2
  public static final int swerveModuleBLDrive = 21; // FX Back Left 1
  public static final int swerveModuleFLAngle = 25; // FX Front Left 2
  public static final int swerveModuleFLDrive = 23; // FX Front Left 1
  public static final int swerveModuleFRAngle = 19; // FX Front Right 1
  public static final int swerveModuleFRDrive = 17; // FX Front Right 2
  public static final int swerveModuleBRAngle = 24; // FX Back Right 1
  public static final int swerveModuleBRDrive = 22; // FX Back Right 2

  // DIO ports
  public static final int climb1LeftEncoder = 0; // REV Through-Bore Encoder
  public static final int climb1RightEncoder = 1; // REV Through-Bore Encoder
  public static final int climb2LeftEncoder = 5; // REV Through-Bore Encoder
  public static final int climb2RightEncoder = 4; // REV Through-Bore Encoder
  public static final int intakePivotEncoder = 3; // REV Through-Bore Encoder

  // Analog Inputs
  public static final int swerveModule1Encoder = 0; // Thriftybot Analog Encoder
  public static final int swerveModule2Encoder = 1; // Thriftybot Analog Encoder
  public static final int swerveModule3Encoder = 2; // Thriftybot Analog Encoder
  public static final int swerveModule4Encoder = 3; // Thriftybot Analog Encoder

  // Arm Initialization
  public static final ArmConstants climb1LeftConstants = new ArmConstants(-17, climb1Left, (1 / 80) * (15 / 36), -72, 38, 40, false);
  public static final ArmConstants climb1RightConstants = new ArmConstants(284, climb1Right, (1 / 80) * (15 / 36), 229, 339, 40, true);
  public static final ArmConstants climb2LeftConstants = new ArmConstants(-44, climb2Left, (1 / 80), -200, 112, 40, false);
  public static final ArmConstants climb2RightConstants = new ArmConstants(-44, climb2Right, (1 / 80), -200, 112, 40, true);
  public static final ArmConstants intakePivotLeftConstants = new ArmConstants(94, intakePivotLeft, (1 / 80), -86, 274, 20, false);
  public static final ArmConstants intakePivotRightConstants = new ArmConstants(94, intakePivotRight, (1 / 80), -86, 274, 20, true);

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

    public final double steeringRatio = 150/7; //150/7 is on datasheet, 6 degrees of error were observed over 1 full rotation of 360 degrees

    public final double steeringToDriveRatio = 1.0 / (firstStageRatio * secondStageRatio * steeringRatio);
    
    public final double metersPerSecondToTicksPer100ms = CPR * driveOverallRatio / wheelCircumfrence / 10.0;

    public static final double maxAttainableSpeedMetersPerSecond = 4.1;
    public static final double maxAttainableRotationRateRadiansPerSecond = 8.0;
    public static final double maxAccelerationMetersPerSecondSquared = 2.9;

    public static final double aimToleranceDegrees = 1.5; // Tolerance of drive aiming, in degrees

    public static final double wheelbaseWidthM = 0.4763;
    public static final double wheelbaseLengthM = 0.7303;

    // The number of ticks of the motor's built-in encoder per revolution of the steering module
    public final double ticksPerSteeringRevolution = 2048 * steeringRatio;
    // Convert degrees to motor ticks
    public final double ticksPerSteeringDegree = ticksPerSteeringRevolution / 360.0;

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



    // arm 1 left limit 1 : 39 degrees (at back, decreases going to front), zero at -17
    // arm 1 right limit 1 : 229 degrees (at back, increases going to front), zero at 284
    // zero secodn pivot at -44

    // intake 0 94

    public final double calibration;
    public final int motorID;
    public final double ratio;
    public final double minimumLimit;
    public final double maximumLimit;
    public final int currentLimit;
    public final boolean isInverted;

    public ArmConstants(double calibration, int motorID, double ratio, double minimumLimit, double maximumLimit, int currentLimit, boolean isInverted) {
      this.calibration = calibration;
      this.motorID = motorID;
      this.ratio = ratio;
      this.minimumLimit = minimumLimit;
      this.maximumLimit = maximumLimit;
      this.currentLimit = currentLimit;
      this.isInverted = isInverted;
    }
  }

  public static final SwerveConstants swerveModules[] = new SwerveConstants[] {
    // Modules are in the order of Back Left, Front Left, Front Right, Back Right, when intake is front of robot
    new SwerveConstants(new Translation2d(-SwerveConstants.wheelbaseLengthM / 2, SwerveConstants.wheelbaseWidthM / 2), 0, swerveModuleBLAngle, swerveModuleBLDrive, swerveModule1Encoder), // 280
    new SwerveConstants(new Translation2d(SwerveConstants.wheelbaseLengthM / 2, SwerveConstants.wheelbaseWidthM / 2), 0, swerveModuleFLAngle, swerveModuleFLDrive, swerveModule2Encoder), // 170
    new SwerveConstants(new Translation2d(SwerveConstants.wheelbaseLengthM / 2, -SwerveConstants.wheelbaseWidthM / 2), 0, swerveModuleFRAngle, swerveModuleFRDrive, swerveModule3Encoder), // 20
    new SwerveConstants(new Translation2d(-SwerveConstants.wheelbaseLengthM / 2, -SwerveConstants.wheelbaseWidthM / 2), 0, swerveModuleBRAngle, swerveModuleBRDrive, swerveModule4Encoder), // 2
  };
}
