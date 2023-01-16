package frc.robot.utils;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;

public class LazySparkMaxPIDController {
    protected final SparkMaxPIDController m_controller;
    protected double m_lastSet = Double.NaN;
    protected CANSparkMax.ControlType m_lastControlType = null;
    protected int m_lastPidSlot = 0;
    protected double m_lastArbFeedforward = Double.NaN;
    protected ArbFFUnits m_lastArbFFUnits = null;
    protected REVLibError m_lastError = null;

    public LazySparkMaxPIDController(SparkMaxPIDController controller) {
        this.m_controller = controller;
    }

    public double getLastSet() {
        return m_lastSet;
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl) {
        if (value != m_lastSet || ctrl != m_lastControlType) {
            m_lastSet = value;
            m_lastControlType = ctrl;
            m_lastError = m_controller.setReference(value, ctrl);
        }
        return m_lastError;
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl, int pidSlot) {
        if (value != m_lastSet || ctrl != m_lastControlType || pidSlot != m_lastPidSlot) {
            m_lastSet = value;
            m_lastControlType = ctrl;
            m_lastPidSlot = pidSlot;
            m_lastError = m_controller.setReference(value, ctrl, pidSlot);
        }
        return m_lastError;
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl, int pidSlot, double arbFeedforward) {
        if (
            value != m_lastSet 
            || ctrl != m_lastControlType 
            || pidSlot != m_lastPidSlot 
            || arbFeedforward != m_lastArbFeedforward
        ) {
            m_lastSet = value;
            m_lastControlType = ctrl;
            m_lastPidSlot = pidSlot;
            m_lastArbFeedforward = arbFeedforward;
            m_lastError = m_controller.setReference(value, ctrl, pidSlot, arbFeedforward);
        }
        return m_lastError;
    }

    public REVLibError setReference(
        double value, 
        CANSparkMax.ControlType ctrl, 
        int pidSlot, 
        double arbFeedforward, 
        ArbFFUnits arbFFUnits
    ) {
        if (
            value != m_lastSet 
            || ctrl != m_lastControlType 
            || pidSlot != m_lastPidSlot 
            || arbFeedforward != m_lastArbFeedforward
            || arbFFUnits != m_lastArbFFUnits
        ) {
            m_lastSet = value;
            m_lastControlType = ctrl;
            m_lastPidSlot = pidSlot;
            m_lastArbFeedforward = arbFeedforward;
            m_lastArbFFUnits = arbFFUnits;
            m_lastError = m_controller.setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
        }
        return m_lastError;
    }
}
