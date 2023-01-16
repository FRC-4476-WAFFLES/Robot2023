package frc.robot.examples;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.utils.LazyTalonFX;

/*
 * This is an example class to demonstrate how to reduce the CAN usage of a TalonFX controller. 
 * Also see the corresponding example for a SparkMax. 
 */
public class LowCANTalonFX {
    private final LazyTalonFX talonFX;

    public LowCANTalonFX() {
        this.talonFX = new LazyTalonFX(0);

        talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
        talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
        talonFX.setControlFramePeriod(ControlFrame.Control_3_General, 100);
    }
}
