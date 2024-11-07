package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {
    private static final double width = Units.inchesToMeters(19.75);
    private static final double length = Units.inchesToMeters(19.75);

    private static final Translation2d frontLeftLocation = new Translation2d(width/2, length/2);
    private static final Translation2d frontRightLocation = new Translation2d(width/2, -length/2);
    private static final Translation2d backLeftLocation = new Translation2d(-width/2, length/2);
    private static final Translation2d backRightLocation = new Translation2d(-width/2, -length/2);

    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveModule frontLeftModule = new SwerveModule(1, 2, frontLeftLocation, EncoderConfig.FRONT_LEFT);
    private final SwerveModule frontRightModule = new SwerveModule(3, 4, frontRightLocation, EncoderConfig.FRONT_RIGHT);
    private final SwerveModule backLeftModule = new SwerveModule(5, 6, backLeftLocation, EncoderConfig.BACK_LEFT);
    private final SwerveModule backRightModule = new SwerveModule(7, 8, backRightLocation, EncoderConfig.BACK_RIGHT);

    private static final double kPositionP = 0.1; 
    private static final double kRotationP = 2;
    private static final double kAtPositionThreshold = Units.inchesToMeters(12);

    // positive is counterclockwise
    private static final double gyroOffsetDegrees = 0;

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, getGyroRotation(), getSwerveModulePositions());

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
        ChassisSpeeds speeds = new ChassisSpeeds(longitudinalSpeedSpeed, lateralSpeed, turnSpeed);
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyroRotation());
        return getModuleStatesFromChassisSpeeds(robotRelativeSpeeds);
    }

    // Field centric
    public SwerveModuleState[] getFieldCentricModuleStates(double longitudinalSpeed, double lateralSpeed, double turnSpeed) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(longitudinalSpeed, lateralSpeed, turnSpeed, getGyroRotation());
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    public void swerveDriveSpeeds(double relativeLateralSpeed, double relativeLongitundalSpeed, double relativeRotationSpeed) {
        if (relativeLateralSpeed < 0.01) relativeLateralSpeed = 0;
        if (relativeLongitundalSpeed < 0.01) relativeLongitundalSpeed = 0;
        if (relativeRotationSpeed < 0.01) relativeRotationSpeed = 0;

        double lateralSpeed = relativeLateralSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
        double longitundalSpeed = relativeLongitundalSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
        double rotationSpeed = relativeRotationSpeed * SwerveConstants.kMaxRotationSpeed;

        SwerveModuleState[] moduleStates = getFieldCentricModuleStates(lateralSpeed, longitundalSpeed, rotationSpeed);
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
    }

    public void swerveDrivePosition(double desiredLongitudinalPosition, double desiredLateralPosition, Rotation2d desiredRotation) {
        Pose2d currentPose = odometer.getPoseMeters();

        double longitudinalError = desiredLongitudinalPosition - currentPose.getX();
        double lateralError = desiredLateralPosition - currentPose.getY();
        double driveAngle = Math.atan2(longitudinalError, lateralError);
        double driveSpeed = DriveModule.normalizeSpeed(Math.hypot(lateralError * kPositionP, longitudinalError * kPositionP));
        double longitudinalSpeed = driveSpeed * Math.sin(driveAngle);
        double lateralSpeed = driveSpeed * Math.cos(driveAngle);

        double desiredRotationRadians = DriveModule.normalizeAngleRadiansSigned(desiredRotation.getRadians());
        double currentRotationRadians = DriveModule.normalizeAngleRadiansSigned(currentPose.getRotation().getRadians()); 
        double rotationErrorRadians = DriveModule.optimizeErrorRadians(desiredRotationRadians - currentRotationRadians);
        double rotationSpeed = DriveModule.normalizeSpeed(rotationErrorRadians / SwerveConstants.kMaxRotationSpeed * kRotationP);

        swerveDriveSpeeds(lateralSpeed, longitudinalSpeed, rotationSpeed);
    }

    public void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = SwerveModule.getAngleRadiansFromComponents(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }

    public void drive() {
        frontLeftModule.setDriveMotorSpeed(0.1);
        frontRightModule.setDriveMotorSpeed(0.1);
        backLeftModule.setDriveMotorSpeed(0.1);
        backRightModule.setDriveMotorSpeed(0.1);
    }

    public void spin() {
        frontLeftModule.setAngleMotorSpeed(0.1);
        frontRightModule.setAngleMotorSpeed(0.1);
        backLeftModule.setAngleMotorSpeed(0.1);
        backRightModule.setAngleMotorSpeed(0.1);
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
    public Rotation2d getOffsetGyroRotation() {
        return Rotation2d.fromDegrees(-gyro.getAngle() - gyroOffsetDegrees);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // PRINT
    public void printEncoderValues() {
        System.out.println("ENCODER POSITIONS");
        frontLeftModule.printEncoderPositions("FL");
        frontRightModule.printEncoderPositions("FR");
        backLeftModule.printEncoderPositions("BL");
        backRightModule.printEncoderPositions("BR");
    }
    public void printGyroValue() {
        System.out.println("GYRO VALUE");
        System.out.println(getGyroRotation());
    }
    public void printOdometerPose() {
        System.out.println("ODOMETER POSE");
        System.out.println(getPose());
    }

    public void reset() {
        resetGyro();
        resetOdometer();
    }

    public void resetGyro() {
        gyro.reset();
    }

    // not working
    public void resetOdometer() {
        printOdometerPose();
        odometer.resetPosition(getGyroRotation(), getSwerveModulePositions(), getPose());
        printOdometerPose();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};
    }

    public boolean isAtPosition(double longitudinalPosition, double lateralPosition) {
        Pose2d pose = getPose();
        return Math.abs(pose.getX() - longitudinalPosition) < kAtPositionThreshold && Math.abs(pose.getY() - lateralPosition) < kAtPositionThreshold;
    }

    public void updateOdometer() {
        odometer.update(getGyroRotation(), getSwerveModulePositions());
    }
}