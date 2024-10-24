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

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;;

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

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(1, 2, frontLeftLocation, frontLeftConfig);
    private final SwerveModule frontRightSwerveModule = new SwerveModule(3, 4, frontRightLocation, frontRightConfig);
    private final SwerveModule backLeftSwerveModule = new SwerveModule(5, 6, backLeftLocation, backLeftConfig);
    private final SwerveModule backRightSwerveModule = new SwerveModule(7, 8, backRightLocation, backRightConfig);

    // temporary, replace with odometry
    private double longitudinalPosition = 0;
    private double lateralPosition = 0;
    private double kPositionP = 0.1; 

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    public DriveSubsystem() {
        // sets the starting direction of the robot to be 0 degrees
        // doesn't work
        resetGyro();
    }
    
    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        // System.out.println(forwardSpeed + " " + turnSpeed);
        double leftSpeed = forwardSpeed + turnSpeed;
        double rightSpeed = forwardSpeed - turnSpeed;
        frontLeftSwerveModule.setDriveMotorSpeed(leftSpeed);
        frontRightSwerveModule.setDriveMotorSpeed(leftSpeed);
        backLeftSwerveModule.setDriveMotorSpeed(rightSpeed);
        backRightSwerveModule.setDriveMotorSpeed(rightSpeed);
    }

    //
    public SwerveModuleState[] getModuleStatesFromChassisSpeeds(ChassisSpeeds speeds) {
        return kinematics.toSwerveModuleStates(speeds);
    }

    // Robot centric
    public SwerveModuleState[] getRobotCentricModuleStates(double longitudinalSpeedSpeed, double lateralSpeed, double turnSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(longitudinalSpeedSpeed, -lateralSpeed, turnSpeed);
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    // Field centric
    public SwerveModuleState[] getFieldCentricModuleStates(double longitudinalSpeed, double lateralSpeed, double turnSpeed) {
        // speeds in swerve are positive forward and positive left, so flip lateral speed
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(longitudinalSpeed, -lateralSpeed, turnSpeed, getRotation2d());
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    public void swerveDrive(double lateralSpeed, double longitudinalSpeed, double turnSpeed) {
        SwerveModuleState[] moduleStates = getRobotCentricModuleStates(longitudinalSpeed, lateralSpeed, turnSpeed);
        frontLeftSwerveModule.setState(moduleStates[0], false);
        frontRightSwerveModule.setState(moduleStates[1], false);
        backLeftSwerveModule.setState(moduleStates[2], false);
        backRightSwerveModule.setState(moduleStates[3], false);
    }

    public void swerveDriveTo(double desiredLongitudinalPosition, double desiredLateralPosition) {
        double longitudinalError = desiredLongitudinalPosition - longitudinalPosition;
        double lateralError = desiredLateralPosition - lateralPosition;
        double angle = Math.atan2(longitudinalError, lateralError);

        double speed = SwerveModule.normalizeSpeed(Math.hypot(lateralError * kPositionP, longitudinalError * kPositionP));
        double longitudinalSpeed = speed * Math.sin(angle);
        double lateralSpeed = speed * Math.cos(angle);

        swerveDrive(lateralSpeed, longitudinalSpeed, 0);
        
        longitudinalPosition += longitudinalSpeed;
        lateralPosition += lateralSpeed;
    }

    public void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = SwerveModule.getAngleRadiansFromComponents(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }

    public void spin() {
        frontLeftSwerveModule.setAngleMotorSpeed(0.1);
        frontRightSwerveModule.setAngleMotorSpeed(0.1);
        backLeftSwerveModule.setAngleMotorSpeed(0.1);
        backRightSwerveModule.setAngleMotorSpeed(0.1);
    }

    public void getEncoderValues() {
        System.out.println("FL: " + frontLeftSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + frontLeftSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + frontLeftSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("FR: " + frontRightSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + frontRightSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + frontRightSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("BL: " + backLeftSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + backLeftSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + backLeftSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("BR: " + backRightSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + backRightSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + backRightSwerveModule.getAngleMotorAbsoluteEncoderRotations());
    }

    public Rotation2d getRotation2d() {
        // positive gyro angle is clockwise
        // positive swerve drive angle is counterclockwise
        // so flip the sign
        // rotate origin 90 degrees clockwise
        return Rotation2d.fromDegrees(-gyro.getAngle() + 90);
    }

    public void getGyroValue() {
        System.out.println(getRotation2d());
    }

    public void resetGyro() {
        gyro.reset();
    }
}