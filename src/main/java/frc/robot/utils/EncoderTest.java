package frc.robot.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class EncoderTest {
    private final DutyCycleEncoder encoder;

    public EncoderTest(int encoderAddress) {
        this.encoder = new DutyCycleEncoder(encoderAddress);
        this.encoder.setDistancePerRotation(360);
    }

    public double getPosition() {
        return encoder.getDistance();
    }
}
