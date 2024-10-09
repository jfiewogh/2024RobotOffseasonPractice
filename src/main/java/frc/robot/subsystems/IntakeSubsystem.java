package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.IntakeConstants;

// make sure the intake deploy motor is retracted when starting

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeDeployMotor = new CANSparkMax(MotorConstants.kIntakeDeployMotorDeviceId, MotorType.kBrushless);
    private final RelativeEncoder intakeDeployMotorRelativeEncoder = intakeDeployMotor.getEncoder();
    
    private final CANSparkMax intakeRollerMotor = new CANSparkMax(MotorConstants.kIntakeRollerMotorDeviceId, MotorType.kBrushless);
    private final CANSparkMax intakeIndexMotor = new CANSparkMax(MotorConstants.kIntakeIndexMotorDeviceId, MotorType.kBrushless);

    /*
     * Return the relative encoder position of the intake deploy motor
     * @return position in rotations
     */
    public double getIntakeDeployRelativePosition() {
        return intakeDeployMotorRelativeEncoder.getPosition();
    }

    private double positionToSpeed(double errorPosition) {
        return errorPosition * IntakeConstants.kP;
    }

    public void setIntakeDeployMotor(double desiredPosition) {
        double currentPosition = getIntakeDeployRelativePosition();
        double errorPosition = desiredPosition - currentPosition;
        double speed = positionToSpeed(errorPosition);
        intakeDeployMotor.set(speed);
    }

    public void setIntakeRollerMotor(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setIntakeIndexMotor(double speed) {
        intakeIndexMotor.set(speed);
    }
}
