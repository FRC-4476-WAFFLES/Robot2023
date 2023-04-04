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
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import frc.robot.utils.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    /** The array of swerve modules on the robot. */
    private final SwerveModule[] modules;

    // /** Allows us to calculate the swerve module states from a chassis motion. */
    public final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private boolean lockWheels = false;
    private boolean isAutoAiming = false;

    public DriveSubsystem() {
        ArrayList<Translation2d> positions = new ArrayList<Translation2d>();
        ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();

        // Initialize each swerve module with its constants.
        for (SwerveModuleConstants module : Constants.DriveConstants.swerveModules) {
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

        gyro.calibrate();

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

        odometry.update(gyro.getRotation2d(), modulePositions);

        SmartDashboard.putNumber("X location", getOdometryLocation().getX());
        SmartDashboard.putNumber("Y location", getOdometryLocation().getY());
        SmartDashboard.putNumber("Odometry Heading", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
        SmartDashboard.putNumber("Vx", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Vy", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Vomega", getChassisSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());

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
    }

    public void robotDrive(double forward, double right, double rotation, boolean fieldCentric){
        ChassisSpeeds chassisSpeeds;

        // TODO: find a better way of limiting undesired rotation
        double robotRotationRate = gyro.getRate();
        robotRotationRate = (robotRotationRate / 180.0) * Math.PI;

        if (forward != 0 || right != 0) {
            rotation += robotRotationRate;
        }

        if (fieldCentric){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
        }
        
        setChassisSpeeds(chassisSpeeds);
    }

    public void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds) {
        setChassisSpeeds(new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond));
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.maxAttainableSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates){
        if (lockWheels) {
            for (int i = 0; i < modules.length; i++) {
                modules[i].drive(new SwerveModuleState(0, Rotation2d.fromDegrees(i % 2 == 0 ? -45 : 45)));
            }
        } else {
            for(int i = 0; i < modules.length; i++){
                modules[i].drive(swerveModuleStates[i]);
            }
        }
    }

    public Pose2d getOdometryLocation() {
        return odometry.getPoseMeters();
    }

    public Pose2d getAdjustedPose() {
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
        return gyro.getRoll();
    }

    /** Stop all motors from running. */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetOdometry(Pose2d robotPose) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        
        for(int x=0; x<modules.length; x++){
            modulePositions[x] = modules[x].getPosition();
        }
        odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), modulePositions, robotPose);
    }

    public void resetSteerEncoders() {
        for (SwerveModule module : modules) {
            module.resetSteerEncoder();
        }
    }

    public void updateLockWheelsTrue() {
        lockWheels = true;
    }

    public void updateLockWheelsFalse() {
        lockWheels = false;
    }

    public boolean getLockWheels() {
        return lockWheels;
    }

    public boolean isAutoAiming() {
        return isAutoAiming;
    }

    public void setAutoAiming(boolean isAutoAiming) {
        this.isAutoAiming = isAutoAiming;
    }
}
