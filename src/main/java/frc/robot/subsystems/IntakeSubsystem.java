package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax transitionMotor = new CANSparkMax(15, MotorType.kBrushless);

    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public void runTransitionMotor(double speed) {
        transitionMotor.set(speed);
    }

    public void stopMotors() {
        stopIntakeMotor();
        stopTransitionMotor();
    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    public void stopTransitionMotor() {
        transitionMotor.set(0);
    }
}
