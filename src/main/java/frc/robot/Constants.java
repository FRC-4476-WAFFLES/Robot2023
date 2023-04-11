// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
  public static final int shoulderLeft = 9; // Talon FX
  public static final int shoulderRight = 11; // Talon FX
  public static final int elbowLeft = 10; // Talon FX
  public static final int elbowRight = 12; // Talon FX
  public static final int wristLeft = 13; // Spark MAX
  public static final int wristRight = 16; // Spark MAX
  public static final int intake = 14; // Spark MAX

  public static final int swerveModule0Angle = 18; // FX Back Left 2
  public static final int swerveModule0Drive = 21; // FX Back Left 1
  public static final int swerveModule1Angle = 25; // FX Front Left 2
  public static final int swerveModule1Drive = 23; // FX Front Left 1
  public static final int swerveModule2Angle = 19; // FX Front Right 1
  public static final int swerveModule2Drive = 17; // FX Front Right 2
  public static final int swerveModule3Angle = 24; // FX Back Right 1
  public static final int swerveModule3Drive = 22; // FX Back Right 2

  // DIO ports
  public static final int shoulderLeftEncoder = 0; // REV Through-Bore Encoder
  public static final int shoulderRightEncoder = 1; // REV Through-Bore Encoder
  public static final int elbowLeftEncoder = 5; // REV Through-Bore Encoder
  public static final int elbowRightEncoder = 4; // REV Through-Bore Encoder
  public static final int wristEncoder = 3; // REV Through-Bore Encoder

  // Analog Inputs
  public static final int swerveModule0Encoder = 0; // Thriftybot Analog Encoder
  public static final int swerveModule1Encoder = 1; // Thriftybot Analog Encoder
  public static final int swerveModule2Encoder = 2; // Thriftybot Analog Encoder
  public static final int swerveModule3Encoder = 3; // Thriftybot Analog Encoder

  // PWM outputs
  public static final int lightsBlinkin = 9; // REV Blinkin

  // PDH channels
  public static final int swerveModule0AnglePower = 0; // FX Front Left 2
  public static final int swerveModule0DrivePower = 1; // FX Front Left 1
  public static final int swerveModule1AnglePower = 19; // FX Back Left 2
  public static final int swerveModule1DrivePower = 18; // FX Back Left 1
  public static final int swerveModule2AnglePower = 11; // FX Back Right 1
  public static final int swerveModule2DrivePower = 10; // FX Back Right 2
  public static final int swerveModule3AnglePower = 9; // FX Front Right 1
  public static final int swerveModule3DrivePower = 8; // FX Front Right 2

  public static final int shoulderLeftPower = 2; // TODO: set actual pdh port
  public static final int shoulderRightPower = 3; // TODO: set actual pdh port
  public static final int elbowLeftPower = 4; // TODO: set actual pdh port
  public static final int elbowRightPower = 5; // TODO: set actual pdh port
  public static final int wristLeftPower = 6; // TODO: set actual pdh port
  public static final int wristRightPower = 7; // TODO: set actual pdh port
  public static final int intakePower = 12; // TODO: set actual pdh port

  /** The different heights of the scoring and pickup locations */
  public enum Height {
    SCORE_HIGH,
    SCORE_MEDIUM,
    SCORE_LOW, 
    PICKUP_SHELF,
    PICKUP_CHUTE,
    PICKUP_GROUND
  }

  /** The different game peices */
  public enum GamePiece {
    CUBE,
    CONE
  }

  public static final class DriveConstants {
    public static final double CPR = 2048; // Encoder ticks per motor rotation
    public static final double wheelDiameter = 0.1016; // Wheel diameter in meters
    public static final double wheelCircumfrence = wheelDiameter * Math.PI; // Wheel circumfrence in meters

    public static final double firstStageRatio = 14.0/50.0;
    public static final double secondStageRatio = 25.0/19.0;
    public static final double thirdStageRatio = 15.0/45.0;
    public static final double driveOverallRatio = 1.0 / (firstStageRatio * secondStageRatio * thirdStageRatio); // Drive gear ratio

    public static final double steeringRatio = 150.0/7.0; //150/7 is on datasheet, 6 degrees of error were observed over 1 full rotation of 360 degrees

    public static final double steeringToDriveRatio = 1.0 / (firstStageRatio * secondStageRatio * steeringRatio);
    
    public static final double metersToTicks = CPR * driveOverallRatio / wheelCircumfrence;
    public static final double metersPerSecondToTicksPer100ms = metersToTicks / 10.0;

    public static final double maxAttainableSpeedMetersPerSecond = 4.1;
    public static final double maxAttainableRotationRateRadiansPerSecond = 6.0;
    public static final double maxAccelerationMetersPerSecondSquared = 2.9;

    public static final double aimToleranceDegrees = 1.5; // Tolerance of drive aiming, in degrees

    public static final double wheelbaseWidthM = 0.4763;
    public static final double wheelbaseLengthM = 0.7303;

    // The number of ticks of the motor's built-in encoder per revolution of the steering module
    public static final double ticksPerSteeringRevolution = 2048.0 * steeringRatio;
    // Convert degrees to motor ticks
    public static final double ticksPerSteeringDegree = ticksPerSteeringRevolution / 360.0;

    public static final double cameraHorisontalTolerance = 2;

    public static final HashMap<DriveState, Pose2d> scoringLocations = new HashMap<>() {{
      put(new DriveState(GamePiece.CONE, Alliance.Red, 0), new Pose2d(14.59, 0.53, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Red, 0), new Pose2d(14.59, 1.06, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 1), new Pose2d(14.59, 1.61, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 2), new Pose2d(14.59, 2.18, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Red, 1), new Pose2d(14.59, 2.76, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 3), new Pose2d(14.59, 3.30, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 4), new Pose2d(14.59, 3.86, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Red, 2), new Pose2d(14.59, 4.42, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 5), new Pose2d(14.59, 4.99, new Rotation2d(0.0)));
  
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 0), new Pose2d(1.98, 0.53, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Blue, 0), new Pose2d(1.98, 1.06, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 1), new Pose2d(1.98, 1.61, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 2), new Pose2d(1.98, 2.18, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Blue, 1), new Pose2d(1.98, 2.76, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 3), new Pose2d(1.98, 3.30, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 4), new Pose2d(1.98, 3.86, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Blue, 2), new Pose2d(1.98, 4.42, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 5), new Pose2d(1.98, 4.99, new Rotation2d(0.0)));
    }};

    public static final HashMap<DriveState, Pose2d> pickupLocations = new HashMap<>() {{
      put(new DriveState(GamePiece.CUBE, Alliance.Red, 0), new Pose2d(1.16, 6.11, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 0), new Pose2d(1.16, 6.11, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CUBE, Alliance.Red, 1), new Pose2d(1.16, 7.47, new Rotation2d(0.0)));
      put(new DriveState(GamePiece.CONE, Alliance.Red, 1), new Pose2d(1.16, 7.47, new Rotation2d(0.0)));

      put(new DriveState(GamePiece.CUBE, Alliance.Blue, 0), new Pose2d(15.33, 6.11, new Rotation2d(Math.PI)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 0), new Pose2d(15.33, 6.11, new Rotation2d(Math.PI)));
      put(new DriveState(GamePiece.CUBE, Alliance.Blue, 1), new Pose2d(15.33, 7.47, new Rotation2d(Math.PI)));
      put(new DriveState(GamePiece.CONE, Alliance.Blue, 1), new Pose2d(15.33, 7.47, new Rotation2d(Math.PI)));
    }};

    // Constants for each individual swerve module
    public static final SwerveModuleConstants swerveModules[] = new SwerveModuleConstants[] {
      // Modules are in the order of Back Left (0), Front Left (1), Front Right (2), Back Right (3), when intake is front of robot
      new SwerveModuleConstants(new Translation2d(-wheelbaseLengthM / 2.0, wheelbaseWidthM / 2.0), 80, swerveModule0Angle, swerveModule0Drive, swerveModule0Encoder),
      new SwerveModuleConstants(new Translation2d(wheelbaseLengthM / 2.0, wheelbaseWidthM / 2.0), -123, swerveModule1Angle, swerveModule1Drive, swerveModule1Encoder),
      new SwerveModuleConstants(new Translation2d(wheelbaseLengthM / 2.0, -wheelbaseWidthM / 2.0), 22, swerveModule2Angle, swerveModule2Drive, swerveModule2Encoder),
      new SwerveModuleConstants(new Translation2d(-wheelbaseLengthM / 2.0, -wheelbaseWidthM / 2.0), 47, swerveModule3Angle, swerveModule3Drive, swerveModule3Encoder),
    };

    public static final class SwerveModuleConstants {
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

      public SwerveModuleConstants(Translation2d position, double calibration, int angleMotor, int driveMotor, int angleEncoder) {
        this.position = position;
        this.calibration = calibration;
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
      }
    }

    public static final class DriveState {
      public final int index;
      public final GamePiece piece;
      public final Alliance alliance;
  
      public DriveState(GamePiece gamePiece, Alliance alliance, int index) {
        this.piece = gamePiece;
        this.alliance = alliance;
        this.index = index;
      }
  
      @Override
      public boolean equals(Object obj) {
        if (obj == null) {
          return false;
        }
        
        if (getClass() != obj.getClass()) {
          return false;
        }
  
        final DriveState driveStateObj = (DriveState) obj;
        
        return this.piece.equals(driveStateObj.piece) && this.alliance.equals(driveStateObj.alliance) && this.index == driveStateObj.index;
      }
  
      @Override
      public int hashCode() {
        int temp = 0;
        temp += piece.ordinal();
        temp += alliance.ordinal() * 2;
        temp += index * 4;
        return temp;
      }
    }
  }

  public static final class ArmConstants {
    public static final double a1 = 1.0;
    public static final double a2 = 1.0;
    public static final double a3 = 0.1;

    public static final double shoulderLeftCalibration = 341.0;
    public static final double shoulderRightCalibration = 284.0;
    public static final double shoulderRatio = (1.0 / 80.0) * (15.0 / 36.0);

    public static final double elbowCalibration = 192.0;
    public static final double elbowBaseRatio = 1.0 / 100.0;
    public static final double elbowChainRunRatio = 32.0 / 74.0;

    public static final double wristCalibration = 70.0;
    public static final double wristRatio = 1.0 / 80.0;

    public static final HashMap<ArmState, SetPoint> setPoints = new HashMap<ArmState, SetPoint>() {{
      put(new ArmState(Height.SCORE_HIGH, GamePiece.CUBE, false), new SetPoint(7000, -113000, 10));
      put(new ArmState(Height.SCORE_HIGH, GamePiece.CONE, false), new SetPoint(12000, -152000, 30)); // 14500, -150000, 30
      put(new ArmState(Height.SCORE_MEDIUM, GamePiece.CUBE, false), new SetPoint(0, -90000, 6));     
      put(new ArmState(Height.SCORE_MEDIUM, GamePiece.CONE, false), new SetPoint(-9000, -102000, 24)); 
      put(new ArmState(Height.SCORE_LOW, GamePiece.CUBE, false), new SetPoint(17000, -23000, 9.2)); 
      put(new ArmState(Height.SCORE_LOW, GamePiece.CONE, false), new SetPoint(17000, -23000, 0)); 
      put(new ArmState(Height.PICKUP_SHELF, GamePiece.CUBE, false), new SetPoint(-17000, -136000, 40)); 
      put(new ArmState(Height.PICKUP_SHELF, GamePiece.CONE, false), new SetPoint(-17000, -142000, 40)); 
      put(new ArmState(Height.PICKUP_CHUTE, GamePiece.CUBE, false), new SetPoint(-17000, -47000, 0)); 
      put(new ArmState(Height.PICKUP_CHUTE, GamePiece.CONE, false), new SetPoint(-17000, -47000, 0)); 
      put(new ArmState(Height.PICKUP_GROUND, GamePiece.CUBE, false), new SetPoint(17000, -21000, 9.2)); 
      put(new ArmState(Height.PICKUP_GROUND, GamePiece.CONE, false), new SetPoint(33000, -27000, 0)); 
    }};

    public static class ArmState {
      public Height height;
      public GamePiece piece;
      public boolean fudge;
  
      public ArmState(Height height, GamePiece piece, boolean fudge){
        this.height = height;
        this.piece = piece;
        this.fudge = fudge;
      }
  
      @Override
      public boolean equals(Object obj){
        if (obj == null) {
          return false;
        }
        
        if (getClass() != obj.getClass()) {
          return false;
        }
  
        final ArmState armStateObj = (ArmState) obj;
  
        return this.height.equals(armStateObj.height) && this.piece.equals(armStateObj.piece);
      }
  
      @Override
      public int hashCode(){
        int temp = 0;
        temp += height.ordinal();
        temp += piece.ordinal() * 4;
        return temp;
      }
    }
  
    public static class SetPoint{
      public final int shoulderPos;
      public final int elbowPos;
      public final double wristPos;
  
      public SetPoint(int shoulderPos, int elbowPos, double wristPos) {
        this.shoulderPos = shoulderPos;
        this.elbowPos = elbowPos;
        this.wristPos = wristPos;
      }
    }
  }
}
