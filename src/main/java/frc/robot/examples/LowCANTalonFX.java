package frc.robot.examples;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.utils.LazyTalonFX;

/*
 * This is an example class to demonstrate how to reduce the CAN usage of a TalonFX controller. 
 */
public class LowCANTalonFX {
    private final LazyTalonFX talonFX;

    public LowCANTalonFX() {
        this.talonFX = new LazyTalonFX(0); // Create a "lazy" Talon FX

        talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100); // Increase the time between status frame updates
        talonFX.setControlFramePeriod(ControlFrame.Control_3_General, 100); // Increase the time between control frame updates
    }
}
