package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveSubsystem extends SubsystemBase {
    private final double width = Units.inchesToMeters(19.75);
    private final double length = Units.inchesToMeters(19.75);

    private final Translation2d frontLeftLocation = new Translation2d(width/2, length/2);
    private final Translation2d frontRightLocation = new Translation2d(-width/2, length/2);
    private final Translation2d backLeftLocation = new Translation2d(width/2, -length/2);
    private final Translation2d backRightLocation = new Translation2d(-width/2, -length/2);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(1, 2, frontLeftLocation, -1.7441);
    private final SwerveModule frontRightSwerveModule = new SwerveModule(3, 4, frontRightLocation, 2.0678);
    private final SwerveModule backLeftSwerveModule = new SwerveModule(5, 6, backLeftLocation, -2.0801 + Math.PI);
    private final SwerveModule backRightSwerveModule = new SwerveModule(7, 8, backRightLocation, 2.8041);

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        double leftSpeed = forwardSpeed + turnSpeed;
        double rightSpeed = forwardSpeed - turnSpeed;
        frontLeftSwerveModule.getDriveMotor().set(leftSpeed);
        frontRightSwerveModule.getDriveMotor().set(leftSpeed);
        backLeftSwerveModule.getDriveMotor().set(rightSpeed);
        backRightSwerveModule.getDriveMotor().set(rightSpeed);
    }

    public void swerveDrive(double ySpeed, double xSpeed, double turnSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        frontLeftSwerveModule.setState(moduleStates[0]);
        frontRightSwerveModule.setState(moduleStates[1]);
        backLeftSwerveModule.setState(moduleStates[2]);
        backRightSwerveModule.setState(moduleStates[3]);
    }

    public void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = SwerveModule.getAngleRadiansFromComponents(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightSwerveModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }

    public void getEncoderValues() {
        System.out.println("FL: " + frontLeftSwerveModule.getEncoderValue());
        System.out.println("FR: " + frontRightSwerveModule.getEncoderValue());
        System.out.println("BL: " + backLeftSwerveModule.getEncoderValue());
        System.out.println("BR: " + backRightSwerveModule.getEncoderValue());
    }
}