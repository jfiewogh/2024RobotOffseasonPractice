package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterAngleMotor = new CANSparkMax(17, MotorType.kBrushless);

    public void runShooterAngleMotor(double speed) {
        shooterAngleMotor.set(speed);
    }
}
