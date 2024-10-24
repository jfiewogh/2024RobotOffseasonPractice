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
        angleMotorRelativeEncoder.setPositionConversionFactor(kPositionConversionFactor); // converts rotations to radians
        /*
         * FL: -0.07314016054881836 0.171875 3.683035714285714 ﻿
﻿﻿﻿﻿﻿﻿ FR: 0.07730072759304932 -0.162109375 -3.473772321428571 ﻿
﻿﻿﻿﻿﻿﻿ BL: -0.08969909093149418 0.200439453125 4.295131138392857 ﻿
﻿﻿﻿﻿﻿﻿ BR: 0.07231618834600788 0.35888671875 7.6904296875 ﻿

﻿﻿﻿﻿﻿﻿ FL: 0.06399162493340019 0.341552734375 7.318987165178571 ﻿
﻿﻿﻿﻿﻿﻿ FR: 0.0024150638806174117 -0.3232421875 -6.926618303571428 ﻿
﻿﻿﻿﻿﻿﻿ BL: 0.03557970758794426 0.40087890625 8.590262276785714 ﻿
﻿﻿﻿﻿﻿﻿ BR: -6.317562758416651E-5 -0.27880859375 -5.974469866071428 ﻿

         */

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

    // positive speed is counterclockwise
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(normalizeSpeed(speed));
    }

    /**
     * This sets the SwerveModule to the desired state
     * @param state the desired speed and angle
     */
    public void setState(SwerveModuleState state, Boolean printData) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        double driveMotorSpeed = speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;

        double currentWheelAngleRadians = normalizeAngleRadians(motorToWheel(getAngleMotorRelativeEncoderRadians()));
        // 3.14
        double desiredWheelAngleRadians = normalizeAngleRadians(state.angle.getRadians());
        // -3.14
        // -6.28
        
        // Optimize Error
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;

        if (printData && Math.abs(wheelErrorRadians) > 0.1) System.out.println(currentWheelAngleRadians + " " + desiredWheelAngleRadians + " " + wheelErrorRadians);

        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = normalizeAngleRadians(desiredWheelAngleRadians + Math.PI - currentWheelAngleRadians);
            driveMotorSpeed = -driveMotorSpeed;
        }

        double motorErrorRadians = wheelToMotor(wheelErrorRadians);
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
        double speed = normalizeSpeed(Math.hypot(speedX, speedY));
        return new double[] {desiredAngle, speed};
    }

    // Convert the error radians to speed
    private static double convertErrorRadiansToSpeed(double errorRadians) {
        return normalizeSpeed(errorRadians / Math.PI * SwerveConstants.kP);
    }

    /**
     * Limit the speed to the range -maxSpeed to maxSpeed
     * @param speed the unnormalized speed
     * @return the normalized speed
     */
    public static double normalizeSpeed(double speed) {
        if (speed > SwerveConstants.kMaxAngleMotorSpeed) {
            return SwerveConstants.kMaxAngleMotorSpeed;
        } else if (speed < -SwerveConstants.kMaxAngleMotorSpeed) {
            return -SwerveConstants.kMaxAngleMotorSpeed;
        }
        return speed;
    }

    /**
     * Optimize the error so that the motor moves the shorter direction
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
     * Turn the motor to the desired wheel angle
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

    // Return the angle motor relative encoder value in radians
    // flip it so that it is counterclockwise
    public double getAngleMotorRelativeEncoderRadians() {
        double position = angleMotorRelativeEncoder.getPosition();
        return flipMotor ? -position : position;
    }
    // Return the angle motor relative encoder value in rotations
    public double getAngleMotorRelativeEncoderRotations() {
        return radiansToRotations(getAngleMotorRelativeEncoderRadians());
    }

    // Return the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return normalizeAngleRadians(Math.atan2(y, x));
    }

    /**
     * Convert the angle to be between -pi and pi
     * @param angleRadians the unnormalized angle radians
     * @return the normalized angle radians
     */
    public static double normalizeAngleRadians(double angleRadians) {
        while (angleRadians < -Math.PI || angleRadians > Math.PI) {
            angleRadians += angleRadians < -Math.PI ? Constants.kTau : -Constants.kTau;
        }
        return angleRadians;
    }

    // Return the angle wheel absolute encoder value in rotations
    public double getAngleWheelAbsoluteEncoderRotations() {
        return angleWheelAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }
    // Return the angle motor absolute encoder value in rotations
    public double getAngleMotorAbsoluteEncoderRotations() {
        return wheelToMotor(getAngleWheelAbsoluteEncoderRotations());
    }
    // Return the angle motor absolute encoder value in radians with offset
    public double getAngleMotorAbsoluteEncoderRadians() {
        return rotationsToRadians(getAngleMotorAbsoluteEncoderRotations());
    }

    // Set the relative encoder values to default
    public void resetEncoders() {
        driveMotorRelativeEncoder.setPosition(0);
        // set the relative encoder position to the absolute encoder position
        double angleMotorAbsoluteEncoderRadians = getAngleMotorAbsoluteEncoderRadians();
        angleMotorRelativeEncoder.setPosition(flipMotor ? -angleMotorAbsoluteEncoderRadians : angleMotorAbsoluteEncoderRadians);
    }
}

