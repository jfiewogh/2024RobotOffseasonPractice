package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;

public class SwerveModule {
    // CONSTANTS
    private static final double TAU = Math.PI * 2; // full circle
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14);
    // PID Values
    private static final double P = 0.05;
    // Gear Ratio
    private static final double ANGLE_MOTOR_GEAR_RATIO = (14.0 / 50.0) * (10.0 / 60.0);

    // Motors and Encoders
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleMotorEncoder;
    private final CANcoder absoluteEncoder;

    private final double turnAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorDeviceId, MotorType.kBrushless);

        turnAngleRadians = getTurningAngleRadians(location); // only used for alternative swerve

        // Encoders
        angleMotorEncoder = angleMotor.getEncoder();
        angleMotorEncoder.setPositionConversionFactor(TAU); // converts rotations to radians

        absoluteEncoder = AbsoluteEncoder.createAbsoluteEncoder(config);
        angleMotorEncoder.setPosition(Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble()));
        absoluteEncoder.close();
    }

    private double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadiansFromComponents(location.getY(), location.getX());
        return normalizeAngleRadians(turningAngleRadians);
    }

    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    public CANSparkMax getAngleMotor() {
        return angleMotor;
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(normalizeSpeed(speed));
    }

    public void setState(SwerveModuleState state) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        setAngle(state.angle);
        setDriveMotorSpeed(speedMetersPerSecond / MAX_SPEED_METERS_PER_SECOND);
    }

    public void setState(double driveSpeed, double driveAngleRadians, double turnSpeed) {
        double[] desiredState = getDesiredState(driveAngleRadians, turnAngleRadians, driveSpeed, turnSpeed);
        double desiredAngleRadians = desiredState[0];
        double desiredSpeed = desiredState[1];
        setAngle(Rotation2d.fromRadians(desiredAngleRadians));
        setDriveMotorSpeed(desiredSpeed);
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
        double speed = normalizeSpeed(Math.hypot(speedX, speedY));
        return new double[] {desiredAngle, speed}; // The speed cannot be over 1
    }

    private double convertRadiansToSpeed(double errorRadians) {
        return normalizeSpeed(errorRadians / Math.PI * P);
    }

    private double normalizeSpeed(double speed) {
        if (speed > 1) {
            return 1;
        } else if (speed < -1) {
            return -1;
        }
        return speed;
    }

    public void setAngle(Rotation2d desiredAngle) {
        double currentWheelAngleRadians = normalizeAngleRadians(angleMotorEncoder.getPosition() * ANGLE_MOTOR_GEAR_RATIO);
        double desiredWheelAngleRadians = normalizeAngleRadians(desiredAngle.getRadians());
        double wheelErrorAngleRadians = normalizeAngleRadians(desiredWheelAngleRadians - currentWheelAngleRadians);
        // Optimizes the angle, goes shortest direction
        if (wheelErrorAngleRadians > Math.PI) {
            wheelErrorAngleRadians = -(TAU - wheelErrorAngleRadians);
        }
        // Convert wheel radians to motor radians
        double motorErrorRadians = wheelErrorAngleRadians / ANGLE_MOTOR_GEAR_RATIO;
        // Convert radians to speed
        double speed = convertRadiansToSpeed(motorErrorRadians);
        angleMotor.set(speed);
    }

    // STATIC METHODS

    public static double getAngleRadiansFromComponents(double y, double x) {
        return normalizeAngleRadians(Math.atan2(y, x));
    }

    public static double normalizeAngleRadians(double angleRadians) {
        while (angleRadians < 0 || angleRadians > TAU) {
            angleRadians += angleRadians < 0 ? TAU : -TAU;
        }
        return angleRadians;
    }

    //
    public double getEncoderValue() {
        return angleMotorEncoder.getPosition();
    }

    public void resetEncoder() { // unused
        angleMotorEncoder.setPosition(0);
    }
}

