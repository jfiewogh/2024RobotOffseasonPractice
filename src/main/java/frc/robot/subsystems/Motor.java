package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Motor {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;

    public Motor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public double getPositionRotations() {
        return reverseEncoder ? -encoder.getPosition() : encoder.getPosition();
    }
    public double getPositionRadians() {
        return DriveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        encoder.setPosition(reverseEncoder ? -position : position);
    }

    public void set(double relativeSpeed) {
        motor.set(reverseMotor ? -relativeSpeed : relativeSpeed);
    }
}


