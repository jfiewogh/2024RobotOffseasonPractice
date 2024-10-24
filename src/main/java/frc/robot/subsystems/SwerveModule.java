package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    private final RelativeEncoder driveMotorRelativeEncoder; // rotations // unused
    private final RelativeEncoder angleMotorRelativeEncoder; // radians

    private final CANcoder angleWheelAbsoluteEncoder; // rotations of the wheel, not the motor

    private final double kPositionConversionFactor = Constants.kTau;
    private final Boolean flipMotor = true;

    private final double turnAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorDeviceId, MotorType.kBrushless);

        turnAngleRadians = getTurningAngleRadians(location); // only used for alternative swerve

        driveMotorRelativeEncoder = driveMotor.getEncoder();
        angleMotorRelativeEncoder = angleMotor.getEncoder();
        
        // counterclockwise should be positive
        // converts rotations to radians
        angleMotorRelativeEncoder.setPositionConversionFactor(kPositionConversionFactor);

        angleWheelAbsoluteEncoder = AbsoluteEncoder.createAbsoluteEncoder(config);

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
    public void setState(SwerveModuleState state, Boolean printData) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        double driveMotorSpeed = speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;

        double currentWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(DriveModule.motorToWheel(getAngleMotorRelativeEncoderRadians()));
        // 3.14
        double desiredWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(state.angle.getRadians());
        // -3.14
        // -6.28
        
        // Optimize Error
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;

        if (printData && Math.abs(wheelErrorRadians) > 0.1) System.out.println(currentWheelAngleRadians + " " + desiredWheelAngleRadians + " " + wheelErrorRadians);

        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveModule.normalizeAngleRadiansSigned(desiredWheelAngleRadians + Math.PI - currentWheelAngleRadians);
            driveMotorSpeed = -driveMotorSpeed;
        }

        double motorErrorRadians = DriveModule.wheelToMotor(wheelErrorRadians);
        double speed = convertErrorRadiansToSpeed(motorErrorRadians);

        angleMotor.set(flipMotor ? -speed : speed);

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

    // Convert the error radians to speed
    private static double convertErrorRadiansToSpeed(double errorRadians) {
        return DriveModule.normalizeSpeed(errorRadians / Math.PI * SwerveConstants.kP);
    }

    /**
     * Turn the motor to the desired wheel angle
     * @param desiredAngle the desired wheel angle
     */
    public void setAngle(Rotation2d desiredAngle) {
        double currentWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(DriveModule.motorToWheel(getAngleMotorRelativeEncoderRadians()));
        double desiredWheelAngleRadians = DriveModule.normalizeAngleRadiansSigned(desiredAngle.getRadians());
        double wheelErrorRadians = DriveModule.optimizeErrorRadians(DriveModule.normalizeAngleRadiansSigned(desiredWheelAngleRadians - currentWheelAngleRadians));
        double motorErrorRadians = DriveModule.wheelToMotor(wheelErrorRadians);
        double speed = convertErrorRadiansToSpeed(motorErrorRadians);
        angleMotor.set(speed);
    }

    // Return the angle motor relative encoder value in radians
    // flip it so that it is counterclockwise
    public double getAngleMotorRelativeEncoderRadians() {
        double position = angleMotorRelativeEncoder.getPosition();
        return flipMotor ? -position : position;
    }
    // Return the angle motor relative encoder value in rotations
    public double getAngleMotorRelativeEncoderRotations() {
        return DriveModule.radiansToRotations(getAngleMotorRelativeEncoderRadians());
    }

    // Return the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return DriveModule.normalizeAngleRadiansSigned(Math.atan2(y, x));
    }

    // Return the angle wheel absolute encoder value in rotations
    public double getAngleWheelAbsoluteEncoderRotations() {
        return angleWheelAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }
    // Return the angle motor absolute encoder value in rotations
    public double getAngleMotorAbsoluteEncoderRotations() {
        return DriveModule.wheelToMotor(getAngleWheelAbsoluteEncoderRotations());
    }
    // Return the angle motor absolute encoder value in radians with offset
    public double getAngleMotorAbsoluteEncoderRadians() {
        return DriveModule.rotationsToRadians(getAngleMotorAbsoluteEncoderRotations());
    }

    // Set the relative encoder values to default
    public void resetEncoders() {
        driveMotorRelativeEncoder.setPosition(0);
        // set the relative encoder position to the absolute encoder position
        double angleMotorAbsoluteEncoderRadians = getAngleMotorAbsoluteEncoderRadians();
        angleMotorRelativeEncoder.setPosition(flipMotor ? -angleMotorAbsoluteEncoderRadians : angleMotorAbsoluteEncoderRadians);
    }
}

