package frc.robot.subsystems;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final Motor driveMotor;
    private final Motor angleMotor;

    private final AbsoluteEncoder wheelAngleAbsoluteEncoder; // rotations of the wheel, not the motor

    private final Boolean flipMotor = true;

    private final double turnAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new Motor(driveMotorDeviceId, true);
        
        angleMotor = new Motor(angleMotorDeviceId, flipMotor);

        // for some reason, clockwise means counterclockwise positive
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        turnAngleRadians = getTurningAngleRadians(location); // only used for alternative swerve

        resetEncoders();
    }

    private static double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadiansFromComponents(location.getY(), location.getX());
        return DriveModule.normalizeAngleRadiansSigned(turningAngleRadians);
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(DriveModule.normalizeSpeed(speed));
    }

    // positive speed is counterclockwise
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(DriveModule.normalizeSpeed(speed));
    }

    /**
     * This sets the SwerveModule to the desired state
     * @param state the desired speed and angle
     */
    public void setState(SwerveModuleState state) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        double driveMotorSpeed = speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;

        double currentWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(DriveModule.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(state.angle.getRadians());
        
        // Optimize Error
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;

        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveModule.normalizeAngleRadiansSigned(wheelErrorRadians + Math.PI);
            driveMotorSpeed = -driveMotorSpeed;
        }

        double motorErrorRadians = DriveModule.angleWheelToMotor(wheelErrorRadians);
        double speed = DriveModule.convertErrorRadiansToSpeed(motorErrorRadians);

        angleMotor.set(speed);

        setDriveMotorSpeed(driveMotorSpeed);
    }

    public void setState(double driveSpeed, double driveAngleRadians, double turnSpeed) {
        double[] desiredState = getDesiredState(driveAngleRadians, turnAngleRadians, driveSpeed, turnSpeed);
        double desiredAngleRadians = desiredState[0];
        double desiredSpeed = desiredState[1];
        setAngle(Rotation2d.fromRadians(desiredAngleRadians));
        setDriveMotorSpeed(desiredSpeed);
    }

    private static double[] getDesiredState(double driveAngleRadians, double turnAngleRadians, double driveSpeed, double turnSpeed) {
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
        double speed = DriveModule.normalizeSpeed(Math.hypot(speedX, speedY));
        return new double[] {desiredAngle, speed};
    }

    /**
     * Turn the motor to the desired wheel angle
     * @param desiredAngle the desired wheel angle
     */
    public void setAngle(Rotation2d desiredAngle) {
        double currentWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(DriveModule.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(desiredAngle.getRadians());
        double wheelErrorRadians = DriveModule.optimizeErrorRadians(DriveModule.normalizeAngleRadiansSigned(desiredWheelAngleRadians - currentWheelAngleRadians));
        double motorErrorRadians = DriveModule.angleWheelToMotor(wheelErrorRadians);
        double speed = DriveModule.convertErrorRadiansToSpeed(motorErrorRadians);
        angleMotor.set(speed);
    }

    // Return the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return DriveModule.normalizeAngleRadiansSigned(Math.atan2(y, x));
    }

    // Set the relative encoder values to default
    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveModule.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double r1 = angleMotor.getPositionRotations();
        double a1 = wheelAngleAbsoluteEncoder.getPositionRotations();
        double a2 = DriveModule.angleWheelToMotor(a1);
        System.out.println("R1 " + String.format("%.3f", r1) + ", A1 " + String.format("%.3f", a1) + ", A2 " + String.format("%.3f", a2));    
    }

    // meters
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            DriveModule.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters,
            Rotation2d.fromRadians(DriveModule.angleMotorToWheel(angleMotor.getPositionRadians()))
        );
    }
}

