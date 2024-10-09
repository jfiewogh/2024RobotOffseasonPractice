package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeDeployMotor = new CANSparkMax(MotorConstants.kIntakeDeployMotorDeviceId, MotorType.kBrushless);
    private final CANSparkMax intakeRollerMotor = new CANSparkMax(MotorConstants.kIntakeRollerMotorDeviceId, MotorType.kBrushless);
    private final CANSparkMax intakeIndexMotor = new CANSparkMax(MotorConstants.kIntakeIndexMotorDeviceId, MotorType.kBrushless);

    public void setIntakeDeployMotor(double speed) {
        intakeDeployMotor.set(speed);
    }

    public void setIntakeRollerMotor(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setIntakeIndexMotor(double speed) {
        intakeIndexMotor.set(speed);
    }
}
