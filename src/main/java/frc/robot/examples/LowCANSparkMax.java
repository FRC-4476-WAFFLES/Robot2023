package frc.robot.examples;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.utils.LazySparkMaxPIDController;

/*
 * This is an example class to demonstrate how to reduce the CAN usage of a SparkMax controller. 
 * Also see the corresponding example for a TalonFX. 
 */
public class LowCANSparkMax {
    private final CANSparkMax sparkMax;
    private final LazySparkMaxPIDController pidController;
    
    public LowCANSparkMax() {
        sparkMax = new CANSparkMax(0, MotorType.kBrushless);
        pidController = new LazySparkMaxPIDController(sparkMax.getPIDController());

        sparkMax.setControlFramePeriodMs(100);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    }
}
