package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Motor {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final Boolean isReversed;

    public Motor(int deviceId, Boolean reverse) {
        motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        isReversed = reverse;
    }

    public double getPositionRotations() {
        double position = encoder.getPosition();
        return isReversed ? -position : position;
    }
    public double getPositionRadians() {
        return DriveModule.rotationsToRadians(getPositionRotations());
    }
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public void set(double relativeSpeed) {
        motor.set(isReversed ? -relativeSpeed : relativeSpeed);
    }
}
