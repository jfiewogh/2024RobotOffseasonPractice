package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    // CONSTANTS
    private static final double TAU = Math.PI * 2; // full circle
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14);
    // PID Values
    private static final double P = 0.05;
    // Gear Ratio
    private static final double TURN_MOTOR_GEAR_RATIO = (14.0 / 50.0) * (10.0 / 60.0);

    // Motors and Encoders
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnMotorEncoder;

    private final double turningAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int turnMotorDeviceId, Translation2d location, double encoderOffset) {
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorDeviceId, MotorType.kBrushless);
        turningAngleRadians = getTurningAngleRadians(location);
        turnMotorEncoder = turnMotor.getEncoder();
        turnMotorEncoder.setPosition(encoderOffset);
    }

    private double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadiansFromComponents(location.getY(), location.getX());
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

    public void setAngle(Rotation2d desiredAngle) {
        // almost works
        // errors backwards

        System.out.println("ID: " + this.turnMotor.getDeviceId());

        System.out.println(turnMotorEncoder.getPosition() + " - " + turnMotorEncoder.getPosition() * TAU);

        double currentWheelAngleRadians = normalizeAngleRadians(turnMotorEncoder.getPosition() * TAU * TURN_MOTOR_GEAR_RATIO);
        double desiredWheelAngleRadians = normalizeAngleRadians(desiredAngle.getRadians());

        System.out.println(currentWheelAngleRadians + " " + desiredWheelAngleRadians);

        double currentAngleRadians = currentWheelAngleRadians / TURN_MOTOR_GEAR_RATIO;
        double desiredAngleRadians = desiredWheelAngleRadians / TURN_MOTOR_GEAR_RATIO;

        // optimize this not using swervemodulestate (go shortest direction)
        double errorRadians = desiredAngleRadians - currentAngleRadians;
        
        System.out.println(currentAngleRadians + " " + desiredAngleRadians + " " + errorRadians);

        double speed = errorRadians / Math.PI * P;
        if (speed > 1) {
            speed = 1;
        } else if (speed < -1) {
            speed = -1;
        }
        System.out.println(speed);

        turnMotor.set(speed);
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
        return turnMotorEncoder.getPosition();
    }

    public void resetEncoder() { // unused
        turnMotorEncoder.setPosition(0);
    }
}

