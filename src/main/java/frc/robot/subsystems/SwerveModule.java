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

    private final double turnAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        // does the type of battery or position affect this?
        driveMotor = new Motor(driveMotorDeviceId, false, false);
        // option 1: true, true
        // option 2: false, false
        angleMotor = new Motor(angleMotorDeviceId, true, true);

        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        turnAngleRadians = getTurningAngleRadians(location); // only used for alternative swerve

        resetEncoders();
    }

    private static double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadiansFromComponents(location.getY(), location.getX());
        return DriveUtils.normalizeAngleRadiansSigned(turningAngleRadians);
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(DriveUtils.normalizeSpeed(speed));
    }
    // positive speed is counterclockwise
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(DriveUtils.normalizeSpeed(speed));
    }

    /**
     * This sets the SwerveModule to the desired state
     * @param state the desired speed and angle
     */
    public void setState(SwerveModuleState state) {
        double driveMotorSpeed = state.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
        // Get and Optimize Error
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(state.angle.getRadians());
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelErrorRadians + Math.PI);
            driveMotorSpeed = -driveMotorSpeed;
        }
        double motorErrorRadians = DriveUtils.angleWheelToMotor(wheelErrorRadians);
        double speed = DriveUtils.convertErrorRadiansToSpeed(motorErrorRadians);
        setAngleMotorSpeed(speed);
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
        double speed = DriveUtils.normalizeSpeed(Math.hypot(speedX, speedY));
        return new double[] {desiredAngle, speed};
    }

    /**
     * Turn the motor to the desired wheel angle
     * @param desiredAngle the desired wheel angle
     */
    public void setAngle(Rotation2d desiredAngle) {
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(desiredAngle.getRadians());
        double wheelErrorRadians = DriveUtils.optimizeErrorRadians(DriveUtils.normalizeAngleRadiansSigned(desiredWheelAngleRadians - currentWheelAngleRadians));
        double motorErrorRadians = DriveUtils.angleWheelToMotor(wheelErrorRadians);
        double speed = DriveUtils.convertErrorRadiansToSpeed(motorErrorRadians);
        setAngleMotorSpeed(speed);
    }

    // Return the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return DriveUtils.normalizeAngleRadiansSigned(Math.atan2(y, x));
    }

    // Set the relative encoder values to default
    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }
/*
 * 
 *  ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 12.513083457946777, A1 0.412841796875, A2 8.846609933035714 ﻿
﻿﻿﻿﻿﻿﻿ ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 13.441595077514648, A1 -0.440185546875, A2 -9.432547433035714 ﻿
﻿﻿﻿﻿﻿﻿ ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 14.349254608154297, A1 -0.294677734375, A2 -6.314522879464286 ﻿
﻿﻿﻿﻿﻿﻿ ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 15.284842491149902, A1 -0.144775390625, A2 -3.102329799107143 ﻿
﻿﻿﻿﻿﻿﻿ ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 16.206466674804688, A1 0.001953125, A2 0.04185267857142857 ﻿
﻿﻿﻿﻿﻿﻿ ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 17.058269500732422, A1 0.135498046875, A2 2.903529575892857 ﻿
﻿﻿﻿﻿﻿﻿ ENCODER POSITIONS ﻿
﻿﻿﻿﻿﻿﻿ FL: R1 17.944984436035156, A1 0.277587890625, A2 5.948311941964286 ﻿
﻿﻿﻿﻿﻿﻿ End drive command ﻿
 */


    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double r2 = angleMotor.getPositionRotations();
        double r1 = DriveUtils.angleMotorToWheel(r2);
        double a1 = wheelAngleAbsoluteEncoder.getPositionRotations();
        double a2 = DriveUtils.angleWheelToMotor(a1);
        System.out.println("R1 " + r1 + ", R2 " + r2 + ", A1 " + a1 + ", A2 " + a2);    
    }

    public void printDriveEncoderValue(String name) {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }

    // meters
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters,
            Rotation2d.fromRadians(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()))
        );
    }

    double sumRelative = 0;
    double sumAbsolute = 0;

    double count = 1;

    double lastRelative = 0;
    double lastAbsolute = 0;

    public void printPositionSlope() {
        double currentRelative = DriveUtils.angleMotorToWheel(angleMotor.getPositionRotations());
        System.out.println(currentRelative - lastRelative + " " + sumRelative/count);
        double currentAbsolute = wheelAngleAbsoluteEncoder.getPositionRotations();
        System.out.println(currentAbsolute - lastAbsolute + " " + sumAbsolute/count);

        sumRelative += currentRelative - lastRelative;
        sumAbsolute += currentAbsolute - lastAbsolute;
        count++;

        lastRelative = currentRelative;
        lastAbsolute = currentAbsolute;
    }
}

