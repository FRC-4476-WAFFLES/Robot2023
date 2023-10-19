// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;

public class SwerveModule {
    /** Holds constants like the angle calibration. */
    private final SwerveModuleConstants constants;

    /** The motor controlling the angle of the swerve module. */
    private final LazyTalonFX angleMotor;

    /** The motor controlling the speed of the swerve module. */
    private final LazyTalonFX driveMotor;

    private final AnalogEncoder angleEncoder;
    
    private SwerveModuleState desiredState = new SwerveModuleState();

    private double lastAngle;
    private double lastSpeed = 0;
    private double lastCurrentLimit = DriveConstants.maxDriveCurrentLimit;
    private boolean lastBrakeMode = true;

    public SwerveModule(SwerveModuleConstants constants) {
        this.angleMotor = new LazyTalonFX(constants.angleMotor);
        this.driveMotor = new LazyTalonFX(constants.driveMotor);
        this.angleEncoder = new AnalogEncoder(constants.angleEncoder);
        this.constants = constants;

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        angleMotor.setInverted(true);

        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.maxDriveCurrentLimit, DriveConstants.maxDriveCurrentLimit, 0.03));
        angleMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.03));

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        driveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
        angleMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
        driveMotor.configVelocityMeasurementWindow(4);
        angleMotor.configVelocityMeasurementWindow(4);

        driveMotor.configVoltageCompSaturation(12);
        angleMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);
        angleMotor.enableVoltageCompensation(true);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
        angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);

        angleMotor.config_kP(0, 0.05); // 0.05
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0.0); // 0.2
        angleMotor.configNeutralDeadband(0.02);

        driveMotor.config_kP(0, 0.1); // 0.1
        driveMotor.config_kI(0, 0); // 0
        driveMotor.config_kD(0, 0.0); // 0.0
        driveMotor.config_kF(0, 0.045); // 0.045

        angleEncoder.setDistancePerRotation(360);
        resetSteerEncoder();
    }

    /** Drives the swerve module in a direction at a speed.
     * 
     * @param desired Desired state of the swerve module
     */
    public void drive(SwerveModuleState desired) {
        double currentAngleRaw = angleMotor.getSelectedSensorPosition() / DriveConstants.ticksPerSteeringDegree;
        double currentAngleVelocityRaw = angleMotor.getSelectedSensorVelocity();

        double currentAngle = currentAngleRaw % 360;

        double velocityOffset = -currentAngleVelocityRaw * DriveConstants.steeringToDriveRatio;

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(currentAngle));

        // if (optimizedState.angle.getDegrees() > 180) {
        //     optimizedState.angle = optimizedState.angle.minus(Rotation2d.fromDegrees(360));
        // }
        // if (optimizedState.angle.getDegrees() < -180) {
        //     optimizedState.angle = optimizedState.angle.plus(Rotation2d.fromDegrees(360));
        // }
        
        // The minus function will currently give you an angle from -180 to 180.
        // If future library versions change this, this code may no longer work.
        double targetAngle = currentAngleRaw + optimizedState.angle.minus(Rotation2d.fromDegrees(currentAngleRaw)).getDegrees();

        // if (targetAngle > 180) {
        //     targetAngle -= 360;
        // }
        // if (targetAngle < -180) {
        //     targetAngle += 360;
        // }

        this.desiredState = new SwerveModuleState(desired.speedMetersPerSecond, Rotation2d.fromDegrees(targetAngle));

        // double angle = (Math.abs(desired.speedMetersPerSecond) <= (SwerveConstants.maxAttainableSpeedMetersPerSecond * 0.01)) ? lastAngle : targetAngle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        // lastAngle = angle;

        angleMotor.set(ControlMode.Position, targetAngle * DriveConstants.ticksPerSteeringDegree);
        driveMotor.set(ControlMode.Velocity, optimizedState.speedMetersPerSecond * DriveConstants.metersPerSecondToTicksPer100ms - velocityOffset);
        //driveMotor.set(ControlMode.Velocity, -velocityOffset);
    }

    /**Drives the swerve module in a direction at a speed with active traction control. 
     * Traction control includes wheelspin reduction on power, ABS, and understeer reduction
     * 
     * @param desired Desired state of the swerve module
     * @param robotAccelX X component of robot acceleration (m/s/s)
     * @param robotAccelY Y component of robot acceleration (m/s/s)
     * @param robotAccelTheta Yaw component of robot acceleration (rad/s/s)
     */
    public void driveWithTractionControl(SwerveModuleState desired, double robotAccelX, double robotAccelY, double robotAccelTheta) {
        double currentAngleRaw = angleMotor.getSelectedSensorPosition() / DriveConstants.ticksPerSteeringDegree;
        double currentAngleVelocityRaw = angleMotor.getSelectedSensorVelocity();

        double currentAngle = currentAngleRaw % 360;

        double velocityOffset = -currentAngleVelocityRaw * DriveConstants.steeringToDriveRatio;

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(currentAngle));
        
        // The minus function will currently give you an angle from -180 to 180.
        // If future library versions change this, this code may no longer work.
        double targetAngle = currentAngleRaw + optimizedState.angle.minus(Rotation2d.fromDegrees(currentAngleRaw)).getDegrees();

        desiredState = new SwerveModuleState(desired.speedMetersPerSecond, Rotation2d.fromDegrees(targetAngle));

        /* =========== Run traction control =========== */

        double currentSpeed = driveMotor.getSelectedSensorVelocity() / DriveConstants.metersPerSecondToTicksPer100ms;

        // Longitudenal traction control. Adjust the current limit of the drive motor to reduce wheel spin.
        double robotAccelLong = robotAccelX * Math.cos(Math.toRadians(currentAngle)) + robotAccelY * Math.sin(Math.toRadians(currentAngle));
        double wheelAccelLong = (currentSpeed - lastSpeed) / 0.02;
        lastSpeed = currentSpeed;

        double diffAccelLong = robotAccelLong - wheelAccelLong;

        double maxDriveCurrent = DriveConstants.maxDriveCurrentLimit;
        boolean driveBrakeMode = true;

        // If there is a meaningful difference between longitudenal wheel and robot acceleration, run traction control
        if (Math.abs(diffAccelLong) > 0.1) {
            double outputCurrent = driveMotor.getStatorCurrent();
            if (outputCurrent > 1) {
                // Apply current limiting if on power
                maxDriveCurrent = outputCurrent * 0.9;
            } else {
                // TODO: ABS control here. May be able to use stator current limiting again, as that might reduce braking effectiveness
                // For now, just follow basic ABS and alternately disable and re-enable brake mode
                driveBrakeMode = false;
            }
        }

        // Only update the current limit if it has changed to save on CAN usage
        if (maxDriveCurrent != lastCurrentLimit) {
            driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, maxDriveCurrent, maxDriveCurrent, 0.0));
            lastCurrentLimit = maxDriveCurrent;
        }

        // Only update brake mode if it has changed to save on CAN usage
        if (driveBrakeMode != lastBrakeMode) {
            driveMotor.setNeutralMode((driveBrakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
            lastBrakeMode = driveBrakeMode;
        }

        // Lateral traction control. Modify the target steering angle to reduce sliding
        // TODO: make sure this works when robot is rotating (I am fairly sure the physics works out to having it work)
        // aRad = rad/s * m/s
        double wheelAccelLat = Math.toRadians(currentAngleVelocityRaw / DriveConstants.ticksPerSteeringDegree) * currentSpeed;
        double robotAccelLat = robotAccelX * Math.sin(Math.toRadians(currentAngle)) + robotAccelY * Math.cos(Math.toRadians(currentAngle));
        
        double diffAccelLat = robotAccelLat - wheelAccelLat;

        // If there is a meaningful difference between lateral wheel and robot acceleration, run understeer reduction
        if (Math.abs(diffAccelLat) > 0.1) {
            // Use arctan to adjust the target angle by an angle from -pi/2 to pi/2 based on magnitude of lateral acceleration difference
            targetAngle += Math.atan(diffAccelLat);
        }

        /* =========== End of traction control =========== */

        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        targetAngle = (Math.abs(desired.speedMetersPerSecond) <= (DriveConstants.maxAttainableSpeedMetersPerSecond * 0.01)) ? lastAngle : targetAngle;
        
        lastAngle = targetAngle;
        
        angleMotor.set(ControlMode.Position, targetAngle * DriveConstants.ticksPerSteeringDegree);
        driveMotor.set(ControlMode.Velocity, optimizedState.speedMetersPerSecond * DriveConstants.metersPerSecondToTicksPer100ms - velocityOffset);
    }

    public void resetSteerEncoder() {
        angleMotor.setSelectedSensorPosition((getAbsolutePosition() - constants.calibration) * DriveConstants.ticksPerSteeringDegree);
    }

    /**
     * Get the current state of a swerve module
     * @return SwerveModuleState representing the current speed and angle of the module
     */
    
    public SwerveModuleState getState(){
        double currentVelocity = driveMotor.getSelectedSensorVelocity() / DriveConstants.metersPerSecondToTicksPer100ms;
        //currentVelocity += angleMotor.getSelectedSensorVelocity() * constants.steeringToDriveRatio;

        double currentAngle = (angleMotor.getSelectedSensorPosition() / DriveConstants.ticksPerSteeringDegree) % 360;
        if (currentAngle < -180) {
            currentAngle += 360;
        } else if (currentAngle > 180) {
            currentAngle -= 360;
        }

        return new SwerveModuleState(currentVelocity, Rotation2d.fromDegrees(currentAngle));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() / DriveConstants.metersToTicks, getState().angle);
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public double getAbsoluteEncoderPosition() {
        return getAbsolutePosition();
    }

    public double getAngleMotorPosition() {
        return angleMotor.getSelectedSensorPosition();
    }
    public double getDriveMotorPosition(){
        return driveMotor.getSelectedSensorPosition();
    }

    public double getCalculatedEncoderPos() {
        return (getAbsolutePosition() - constants.calibration) * DriveConstants.ticksPerSteeringDegree;
    }

    /** Stops all motors from running. */
    public void stop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
        this.angleMotor.set(ControlMode.PercentOutput, 0);
    }

    private double getAbsolutePosition() {
        double position = angleEncoder.getDistance();
        if (position > 180) {
            while (position > 180) {
                position -= 360;
            }
        } else if (position < -180) {
            while (position < -180) {
                position += 360;
            }
        }
        return position;
    }
}
