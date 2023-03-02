// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    /** The array of swerve modules on the robot. */
    private final SwerveModule[] modules;

    // /** Allows us to calculate the swerve module states from a chassis motion. */
    public final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final AHRS ahrsIMU = new AHRS(SPI.Port.kMXP);

    public DriveSubsystem() {
        ArrayList<Translation2d> positions = new ArrayList<Translation2d>();
        ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();

        // Initialize each swerve module with its constants.
        for (SwerveConstants module : Constants.swerveModules) {
            modules.add(new SwerveModule(module));
            positions.add(module.position);
        }

        // Set up the kinematics and odometry.
        Translation2d[] positionArry = new Translation2d[positions.size()];
        SwerveModule[] moduleArray = new SwerveModule[modules.size()];
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[positions.size()];

        for(int x = 0; x < positions.size(); x++){
            positionArry[x] = positions.get(x);
            moduleArray[x] = modules.get(x);
            modulePositions[x] = modules.get(x).getPosition();
        }

        ahrsIMU.calibrate();

        try{
            Thread.sleep(500);
        } catch (Exception e) {

        }

        resetGyro();

        this.kinematics = new SwerveDriveKinematics(positionArry);
        this.modules = moduleArray;
        this.odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0), modulePositions);
    }

    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        
        for(int x=0; x<modules.length; x++){
            moduleStates[x] = modules[x].getState();
            modulePositions[x] = modules[x].getPosition();
        }

        //HEY YOO WTF IS THIS THING?!?!?!?!?!? \/   --CTRL-f label [jeremyWasHere]
        odometry.update(Rotation2d.fromDegrees(-ahrsIMU.getAngle()), modulePositions);

        SmartDashboard.putNumber("X location Is this changing", getOdometryLocation().getX());
        SmartDashboard.putNumber("Y location", getOdometryLocation().getY());
        SmartDashboard.putNumber("Odometry Heading", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("New Gyro Angle", ahrsIMU.getAngle());
        SmartDashboard.putNumber("New Gyro Rate", ahrsIMU.getRate());
        SmartDashboard.putNumber("Vx", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Vy", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Vomega", getChassisSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("Gyro Pitch", ahrsIMU.getPitch());
        SmartDashboard.putNumber("Gyro Roll", ahrsIMU.getRoll());

        for (int i = 0; i < modules.length; i++) {
            String moduleLabel = "Module " + String.valueOf(i) + " ";
            SmartDashboard.putNumber(moduleLabel + "Speed", moduleStates[i].speedMetersPerSecond);
            SmartDashboard.putNumber(moduleLabel + "Angle", modules[i].getState().angle.getDegrees());
            SmartDashboard.putNumber(moduleLabel + "Encoder", modules[i].getAbsoluteEncoderPosition());

            double encoderAngle = modules[i].getAbsoluteEncoderPosition() * (21.4 * 2048.0 / 360.0);
            double motorAngle = modules[i].getAngleMotorPosition();
            SmartDashboard.putNumber(moduleLabel + "Angle Difference", (motorAngle-encoderAngle) % 2048);

            SmartDashboard.putNumber(moduleLabel + "Builtin encoder pos", modules[i].getAngleMotorPosition());
            SmartDashboard.putNumber(moduleLabel + "Calculated encoder pos", modules[i].getCalculatedEncoderPos());
        }

        //field.setRobotPose(this.getOdometryLocation());

        //SmartDashboard.putNumber("Drive Target Heading (Degrees)", Math.toDegrees(Math.atan2(-getOdometryLocation().getY(), getOdometryLocation().getX())));
    }

    public void robotDrive(double forward, double right, double rotation, boolean fieldCentric){
        ChassisSpeeds chassisSpeeds;

        double robotRotationRate = ahrsIMU.getRate();
        robotRotationRate = (robotRotationRate / 180.0) * Math.PI;

        if (forward != 0 || right != 0) {
            rotation += robotRotationRate;
        }

        if (fieldCentric){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
        }

        SmartDashboard.putNumber("Target X Velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Target Y Velocity", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Target Angular Velocity", chassisSpeeds.omegaRadiansPerSecond);
        
        setChassisSpeeds(chassisSpeeds);
    }

    public void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds) {
        setChassisSpeeds(new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond));
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxAttainableSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates){
        for(int x=0; x<modules.length; x++){
            modules[x].drive(swerveModuleStates[x]);
        }
    }

    public Pose2d getOdometryLocation() {
        return odometry.getPoseMeters();
    }

    public Pose2d getAdjustedPose() {
        // return new Pose2d(
        //     odometry.getPoseMeters().getTranslation(),
        //     odometry.getPoseMeters().getRotation().plus(Rotation2d.fromDegrees(180))
        // );
        return getOdometryLocation();
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for(int x=0; x<modules.length; x++){
            moduleStates[x] = modules[x].getState();
        }
        return kinematics.toChassisSpeeds(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
    }

    public double getPitch() {
        // Because of the orientation of the gyro, the getRoll() function returns the pitch of the robot
        return ahrsIMU.getRoll();
    }

    /** Stop all motors from running. */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void resetGyro() {
        ahrsIMU.reset();
    }

    public void resetOdometry(Pose2d robotPose) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        
        for(int x=0; x<modules.length; x++){
            modulePositions[x] = modules[x].getPosition();
        }
        odometry.resetPosition(Rotation2d.fromDegrees(-ahrsIMU.getAngle()), modulePositions, robotPose);
    }

    public void resetSteerEncoders() {
        for (SwerveModule module : modules) {
            module.resetSteerEncoder();
        }
    }

    public void lockWheels() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].drive(new SwerveModuleState(0, Rotation2d.fromDegrees(i % 2 == 0 ? 45 : -45)));
        }
    }
}
