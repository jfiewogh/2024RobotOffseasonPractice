package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics();

    private static final EncoderConfig frontLeftConfig = EncoderConfig.FRONT_LEFT;
    private static final EncoderConfig frontRightConfig = EncoderConfig.FRONT_RIGHT;
    private static final EncoderConfig backLeftConfig = EncoderConfig.BACK_LEFT;
    private static final EncoderConfig backRightConfig = EncoderConfig.BACK_RIGHT;

    private final SwerveModule frontLeftModule = new SwerveModule(1, 2, frontLeftLocation, frontLeftConfig);
    private final SwerveModule frontRightModule = new SwerveModule(3, 4, frontRightLocation, frontRightConfig);
    private final SwerveModule backLeftModule = new SwerveModule(5, 6, backLeftLocation, backLeftConfig);
    private final SwerveModule backRightModule = new SwerveModule(7, 8, backRightLocation, backRightConfig);

    // temporary, should make better organized, or replace with odometry
    private static double longitudinalPosition = 0;
    private static double lateralPosition = 0;
    private static final double kPositionP = 0.1; 
    private static final double kRotationP = 3;

    private static final double kMaxRotationSpeed = Math.PI; // radians per second
    private static final double kMaxDriveSpeed = 1.5; // meters per second

    // positive is counterclockwise
    private static final double gyroOffsetDegrees = 0;

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, getGyroRotation2d(), getSwerveModulePositions());

    public DriveSubsystem() {}
    
    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        double leftSpeed = forwardSpeed + turnSpeed;
        double rightSpeed = forwardSpeed - turnSpeed;
        frontLeftModule.setDriveMotorSpeed(leftSpeed);
        frontRightModule.setDriveMotorSpeed(leftSpeed);
        backLeftModule.setDriveMotorSpeed(rightSpeed);
        backRightModule.setDriveMotorSpeed(rightSpeed);
    }

    //
    public static SwerveModuleState[] getModuleStatesFromChassisSpeeds(ChassisSpeeds speeds) {
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
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(longitudinalSpeed, -lateralSpeed, turnSpeed, getGyroRotation2d());
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    public void swerveDriveSpeeds(double relativeLateralSpeed, double relativeLongitundalSpeed, double relativeRotationSpeed) {
        double lateralSpeed = relativeLateralSpeed * kMaxDriveSpeed;
        double longitundalSpeed = relativeLongitundalSpeed * kMaxDriveSpeed;
        double rotationSpeed = relativeRotationSpeed * kMaxRotationSpeed;
        SwerveModuleState[] moduleStates = getFieldCentricModuleStates(lateralSpeed, longitundalSpeed, rotationSpeed);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
    }

    public void swerveDrivePosition(double desiredLongitudinalPosition, double desiredLateralPosition, Rotation2d desiredRotation) {
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

        swerveDriveSpeeds(lateralSpeed, longitudinalSpeed, rotationSpeed);
        
        longitudinalPosition += longitudinalSpeed;
        lateralPosition += lateralSpeed;
    }

    public void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = SwerveModule.getAngleRadiansFromComponents(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }

    public void spin() {
        frontLeftModule.setAngleMotorSpeed(0.1);
        frontRightModule.setAngleMotorSpeed(0.1);
        backLeftModule.setAngleMotorSpeed(0.1);
        backRightModule.setAngleMotorSpeed(0.1);
    }

    public void printEncoderValues() {
        frontLeftModule.printEncoderPositions("FL");
        frontRightModule.printEncoderPositions("FR");
        backLeftModule.printEncoderPositions("BL");
        backRightModule.printEncoderPositions("BR");
    }

    public Rotation2d getGyroRotation2d() {
        // positive gyro angle is clockwise
        // positive swerve drive angle is counterclockwise
        // so flip the sign
        // rotate origin 90 degrees clockwise
        return Rotation2d.fromDegrees(-gyro.getAngle() - gyroOffsetDegrees);
    }

    public void printGyroValue() {
        System.out.println(getGyroRotation2d());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetOdometer() {
        odometer.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), odometer.getPoseMeters());
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(), 
            frontRightModule.getPosition(), 
            backLeftModule.getPosition(), 
            backRightModule.getPosition()
        };
    }

    @Override
    public void periodic() {
        odometer.update(getGyroRotation2d(), getSwerveModulePositions());
    }
}