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
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    private final RelativeEncoder driveMotorRelativeEncoder; // rotations // unused
    private final RelativeEncoder angleMotorRelativeEncoder; // radians

    private final CANcoder angleWheelAbsoluteEncoder; // rotations of the wheel, not the motor

    private final double turnAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorDeviceId, MotorType.kBrushless);

        turnAngleRadians = getTurningAngleRadians(location); // only used for alternative swerve

        driveMotorRelativeEncoder = driveMotor.getEncoder();

        angleMotorRelativeEncoder = angleMotor.getEncoder();
        angleMotorRelativeEncoder.setPositionConversionFactor(Constants.kTau); // converts rotations to radians

        angleWheelAbsoluteEncoder = AbsoluteEncoder.createAbsoluteEncoder(config);

        resetEncoders();
    }

    private static double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadiansFromComponents(location.getY(), location.getX());
        return normalizeAngleRadians(turningAngleRadians);
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(normalizeSpeed(speed));
    }

    /**
     * This sets the SwerveModule to the desired state
     * @param state the desired speed and angle
     */
    public void setState(SwerveModuleState state) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        setAngle(state.angle);
        setDriveMotorSpeed(speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond);
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
        double speed = normalizeSpeed(Math.hypot(speedX, speedY));
        return new double[] {desiredAngle, speed};
    }

    // Converts the error radians to speed
    private static double convertErrorRadiansToSpeed(double errorRadians) {
        return normalizeSpeed(errorRadians / Math.PI * SwerveConstants.kP);
    }

    /**
     * Limits the speed to the range -1 to 1
     * @param speed the unnormalized speed
     * @return the normalized speed
     */
    private static double normalizeSpeed(double speed) {
        if (speed > 1) {
            return 1;
        } else if (speed < -1) {
            return -1;
        }
        return speed;
    }

    /**
     * Optimizes the error so that the motor moves the shorter direction
     * If the error is greater than pi, the other direction is shorter, so flip the angle
     * @param errorRadians the difference from the desired angle and the current angle
     * @return the optimized error
     */
    private static double optimizeErrorRadians(double errorRadians) {
        return errorRadians > Math.PI ? errorRadians = -(Constants.kTau - errorRadians) : errorRadians;
    }

    private static double wheelToMotor(double value) {
        return value / SwerveConstants.kAngleMotorGearRatio;
    }
    private static double motorToWheel(double value) {
        return value * SwerveConstants.kAngleMotorGearRatio;
    }

    private static double rotationsToRadians(double rotations) {
        return rotations * Constants.kTau;
    }
    private static double radiansToRotations(double radians) {
        return radians / Constants.kTau;
    }

    /**
     * Spin the motor to the desired wheel angle
     * @param desiredAngle the desired wheel angle
     */
    public void setAngle(Rotation2d desiredAngle) {
        double currentWheelAngleRadians = normalizeAngleRadians(motorToWheel(getAngleMotorRelativeEncoderRadians()));
        double desiredWheelAngleRadians = normalizeAngleRadians(desiredAngle.getRadians());
        double wheelErrorRadians = optimizeErrorRadians(normalizeAngleRadians(desiredWheelAngleRadians - currentWheelAngleRadians));
        
        double motorErrorRadians = wheelToMotor(wheelErrorRadians);
        double speed = convertErrorRadiansToSpeed(motorErrorRadians);

        angleMotor.set(speed);
    }

    // Returns the angle motor relative encoder value in radians
    public double getAngleMotorRelativeEncoderRadians() {
        return angleMotorRelativeEncoder.getPosition();
    }
    // Returns the angle motor relative encoder value in rotations
    public double getAngleMotorRelativeEncoderRotations() {
        return radiansToRotations(getAngleMotorRelativeEncoderRadians());
    }

    // Returns the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return normalizeAngleRadians(Math.atan2(y, x));
    }

    // Get the coterminal angle between 0 and tau (360 degrees)
    public static double normalizeAngleRadians(double angleRadians) {
        while (angleRadians < 0 || angleRadians > Constants.kTau) {
            angleRadians += angleRadians < 0 ? Constants.kTau : -Constants.kTau;
        }
        return angleRadians;
    }

    // Returns the angle motor absolute encoder value in rotations
    public double getAngleWheelAbsoluteEncoderRotations() {
        return angleWheelAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    // Returns the angle motor absolute encoder value in rotations
    public double getAngleMotorAbsoluteEncoderRotations() {
        return wheelToMotor(getAngleWheelAbsoluteEncoderRotations());
    }
    // Returns the angle motor absolute encoder value in radians with offset
    public double getAngleMotorAbsoluteEncoderRadians() {
        return rotationsToRadians(getAngleMotorAbsoluteEncoderRotations());
    }

    // Sets the relative encoder values to default
    public void resetEncoders() {
        driveMotorRelativeEncoder.setPosition(0);
        // sets the relative encoder position to the absolute encoder position
        angleMotorRelativeEncoder.setPosition(getAngleMotorAbsoluteEncoderRadians());
    }
}

