package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
    // CONSTANTS
    private static final double TAU = Math.PI * 2;
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14);
    // PID Values
    private static final double P = 0.01;
    private static final double D = 0.2;
    // Gear Ratio
    private static final double TURN_MOTOR_GEAR_RATIO = (14.0 / 50.0) * (10.0 / 60.0);

    // Motors and Encoders
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnMotorEncoder;

    private final double turningAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int turnMotorDeviceId, Translation2d location) {
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorDeviceId, MotorType.kBrushless);
        turningAngleRadians = getTurningAngleRadians(location);
        turnMotorEncoder = turnMotor.getEncoder();
    }

    private double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadians(location.getY(), location.getX());
        return normalizeAngleRadians(turningAngleRadians);
    }

    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    public CANSparkMax getTurnMotor() {
        return turnMotor;
    }

    public void setState(SwerveModuleState state) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        setAngle(state.angle);
        driveMotor.set(speedMetersPerSecond / MAX_SPEED_METERS_PER_SECOND);
    }
    public void setState(double driveSpeed, double driveAngleRadians, double turnSpeed) {
        double[] desiredState = getDesiredState(driveAngleRadians, turningAngleRadians, driveSpeed, turnSpeed);
        double desiredAngleRadians = desiredState[0];
        double desiredSpeed = desiredState[1];
        setAngle(Rotation2d.fromRadians(desiredAngleRadians));
        driveMotor.set(desiredSpeed);
    }

    private double[] getDesiredState(double driveAngleRadians, double turnAngleRadians, double driveSpeed, double turnSpeed) {
        // Get x and y components of speeds
        double driveSpeedY = driveSpeed * Math.sin(driveAngleRadians);
        double driveSpeedX = driveSpeed * Math.cos(driveAngleRadians);
        double turnSpeedY = turnSpeed * Math.sin(turnAngleRadians);
        double turnSpeedX = turnSpeed * Math.cos(turnAngleRadians);
        // Get total speeds in x and y directions
        double speedY = driveSpeedY + turnSpeedY;
        double speedX = driveSpeedX + turnSpeedX;
        // Determine and return angle and total speed
        double desiredAngle = Math.atan2(speedY, speedX);
        double speed = Math.hypot(speedX, speedY);
        return new double[] {desiredAngle, speed > 1 ? 1 : speed}; // The speed cannot be over 1
    }

    private double lastTimestamp = 0;
    private double lastError = 0;

    private void setAngle(Rotation2d desiredAngle) {
        double currentAngleRadians = turnMotorEncoder.getPosition() * (2 * Math.PI);
        double errorRadians = desiredAngle.getRadians() - currentAngleRadians; // optimize this not using swervemodulestate
        System.out.println(currentAngleRadians + " " + desiredAngle.getRadians() + " " + errorRadians);

        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        double errorRate = (errorRadians - lastError) / dt;

        double speed = (errorRadians / Math.PI / TURN_MOTOR_GEAR_RATIO * P) + (errorRate * D);
        speed = speed > 1 ? 1 : speed; // speed maximum is 1
        System.out.println(speed);

        turnMotor.set(speed);

        lastTimestamp = Timer.getFPGATimestamp();
    }

    // STATIC METHODS

    public static double getAngleRadians(double y, double x) {
        return normalizeAngleRadians(Math.atan2(y, x));
    }

    public static double normalizeAngleRadians(double angleRadians) {
        while (angleRadians < 0 || angleRadians > TAU) {
            angleRadians += angleRadians < 0 ? TAU : -TAU;
        }
        return angleRadians;
    }
}

