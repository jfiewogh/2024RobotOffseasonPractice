package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;
import frc.robot.Constants;

// problems
// FIXED: when changing the angle, the gear makes a weird noise
// sometimes the wheels are not aligned at the right angle, slightly off

public class DriveSubsystem extends SubsystemBase {
    private static final double width = Units.inchesToMeters(19.75);
    private static final double length = Units.inchesToMeters(19.75);

    private static final Translation2d frontLeftLocation = new Translation2d(width/2, length/2);
    private static final Translation2d frontRightLocation = new Translation2d(width/2, -length/2);
    private static final Translation2d backLeftLocation = new Translation2d(-width/2, length/2);
    private static final Translation2d backRightLocation = new Translation2d(-width/2, -length/2);

    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private static final EncoderConfig frontLeftConfig = EncoderConfig.FRONT_LEFT;
    private static final EncoderConfig frontRightConfig = EncoderConfig.FRONT_RIGHT;
    private static final EncoderConfig backLeftConfig = EncoderConfig.BACK_LEFT;
    private static final EncoderConfig backRightConfig = EncoderConfig.BACK_RIGHT;

    private static final SwerveModule frontLeftSwerveModule = new SwerveModule(1, 2, frontLeftLocation, frontLeftConfig);
    private static final SwerveModule frontRightSwerveModule = new SwerveModule(3, 4, frontRightLocation, frontRightConfig);
    private static final SwerveModule backLeftSwerveModule = new SwerveModule(5, 6, backLeftLocation, backLeftConfig);
    private static final SwerveModule backRightSwerveModule = new SwerveModule(7, 8, backRightLocation, backRightConfig);

    // temporary, should make better organized, or replace with odometry
    private static double longitudinalPosition = 0;
    private static double lateralPosition = 0;
    private static final double kPositionP = 0.1; 
    private static final double kRotationP = 3;

    private static final double kTurnSpeedCoefficient = Math.PI;

    // positive is counterclockwise
    private static final double gyroOffsetDegrees = 0;

    private static final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    public DriveSubsystem() {}
    
    public static void arcadeDrive(double forwardSpeed, double turnSpeed) {
        double leftSpeed = forwardSpeed + turnSpeed;
        double rightSpeed = forwardSpeed - turnSpeed;
        frontLeftSwerveModule.setDriveMotorSpeed(leftSpeed);
        frontRightSwerveModule.setDriveMotorSpeed(leftSpeed);
        backLeftSwerveModule.setDriveMotorSpeed(rightSpeed);
        backRightSwerveModule.setDriveMotorSpeed(rightSpeed);
    }

    //
    public static SwerveModuleState[] getModuleStatesFromChassisSpeeds(ChassisSpeeds speeds) {
        return kinematics.toSwerveModuleStates(speeds);
    }

    // Robot centric
    public static SwerveModuleState[] getRobotCentricModuleStates(double longitudinalSpeedSpeed, double lateralSpeed, double turnSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(longitudinalSpeedSpeed, -lateralSpeed, turnSpeed);
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    // Field centric
    public static SwerveModuleState[] getFieldCentricModuleStates(double longitudinalSpeed, double lateralSpeed, double turnSpeed) {
        // speeds in swerve are positive forward and positive left, so flip lateral speed
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(longitudinalSpeed, -lateralSpeed, turnSpeed, getGyroRotation2d());
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    public static void swerveDrive(double lateralSpeed, double longitudinalSpeed, double turnSpeed) {
        SwerveModuleState[] moduleStates = getFieldCentricModuleStates(longitudinalSpeed, lateralSpeed, turnSpeed * kTurnSpeedCoefficient);
        frontLeftSwerveModule.setState(moduleStates[0], false);
        frontRightSwerveModule.setState(moduleStates[1], false);
        backLeftSwerveModule.setState(moduleStates[2], false);
        backRightSwerveModule.setState(moduleStates[3], false);
    }

    public static void swerveDriveTo(double desiredLongitudinalPosition, double desiredLateralPosition, Rotation2d desiredRotation) {
        double longitudinalError = desiredLongitudinalPosition - longitudinalPosition;
        double lateralError = desiredLateralPosition - lateralPosition;
        double driveAngle = Math.atan2(longitudinalError, lateralError);
        double driveSpeed = DriveModule.normalizeSpeed(Math.hypot(lateralError * kPositionP, longitudinalError * kPositionP));
        double longitudinalSpeed = driveSpeed * Math.sin(driveAngle);
        double lateralSpeed = driveSpeed * Math.cos(driveAngle);

        Rotation2d currentRotation = getGyroRotation2d();
        double desiredRotationRadians = DriveModule.normalizeAngleRadiansSigned(desiredRotation.getRadians());
        double currentRotationRadians = DriveModule.normalizeAngleRadiansSigned(currentRotation.getRadians()); 
        double rotationErrorRadians = DriveModule.optimizeErrorRadians(desiredRotationRadians - currentRotationRadians);
        double rotationSpeed = DriveModule.normalizeSpeed(rotationErrorRadians / Constants.kTau * kRotationP);

        swerveDrive(lateralSpeed, longitudinalSpeed, rotationSpeed);
        
        longitudinalPosition += longitudinalSpeed;
        lateralPosition += lateralSpeed;
    }

    public static void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = SwerveModule.getAngleRadiansFromComponents(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }

    public static void spin() {
        frontLeftSwerveModule.setAngleMotorSpeed(0.1);
        frontRightSwerveModule.setAngleMotorSpeed(0.1);
        backLeftSwerveModule.setAngleMotorSpeed(0.1);
        backRightSwerveModule.setAngleMotorSpeed(0.1);
    }

    public static void getEncoderValues() {
        System.out.println("FL: " + frontLeftSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + frontLeftSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + frontLeftSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("FR: " + frontRightSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + frontRightSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + frontRightSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("BL: " + backLeftSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + backLeftSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + backLeftSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("BR: " + backRightSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + backRightSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + backRightSwerveModule.getAngleMotorAbsoluteEncoderRotations());
    }

    public static Rotation2d getGyroRotation2d() {
        // positive gyro angle is clockwise
        // positive swerve drive angle is counterclockwise
        // so flip the sign
        // rotate origin 90 degrees clockwise
        return Rotation2d.fromDegrees(-gyro.getAngle() - gyroOffsetDegrees);
    }

    public static void printGyroValue() {
        System.out.println(getGyroRotation2d());
    }

    public static void resetGyro() {
        System.out.println("Before: " + getGyroRotation2d().getDegrees());
        gyro.reset();
        System.out.println("After: " + getGyroRotation2d().getDegrees());
        System.out.println("Gyro is reset");
    }
}