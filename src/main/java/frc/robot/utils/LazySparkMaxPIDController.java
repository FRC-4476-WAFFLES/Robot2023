package frc.robot.utils;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class LazySparkMaxPIDController {
    protected final SparkMaxPIDController m_controller;
    protected double m_LastSet = Double.NaN;
    protected ControlType m_LastControlType = null;
    protected REVLibError m_LastError = null;

    public LazySparkMaxPIDController(SparkMaxPIDController controller) {
        this.m_controller = controller;
    }

    public double getLastSet() {
        return m_LastSet;
    }


    public REVLibError setReference(double value, ControlType ctrl) {
        if (value != m_LastSet || ctrl != m_LastControlType) {
            m_LastSet = value;
            m_LastControlType = ctrl;
            m_LastError = m_controller.setReference(value, ctrl);
        }
        return m_LastError;
    }
}
