package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
    protected double m_LastSet = Double.NaN;
    protected TalonFXControlMode m_LastControlMode = null;

    public LazyTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return m_LastSet;
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if (value != m_LastSet || mode != m_LastControlMode) {
            m_LastSet = value;
            m_LastControlMode = mode;
            super.set(mode, value);
        }
    }
}
