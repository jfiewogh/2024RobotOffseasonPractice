package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    private final double width = Units.inchesToMeters(19.75);
    private final double length = Units.inchesToMeters(19.75);

    private final Translation2d frontLeftLocation = new Translation2d(width/2, length/2);
    private final Translation2d frontRightLocation = new Translation2d(-width/2, length/2);
    private final Translation2d backLeftLocation = new Translation2d(width/2, -length/2);
    private final Translation2d backRightLocation = new Translation2d(-width/2, -length/2);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveModule frontLeftMotor = new SwerveModule(1, 2, frontLeftLocation);
    private final SwerveModule frontRightMotor = new SwerveModule(3, 4, frontRightLocation);
    private final SwerveModule backLeftMotor = new SwerveModule(5, 6, backLeftLocation);
    private final SwerveModule backRightMotor = new SwerveModule(7, 8, backRightLocation);

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        double leftSpeed = forwardSpeed + turnSpeed;
        double rightSpeed = forwardSpeed - turnSpeed;

        double frontLeftSpeed = leftSpeed; 
        double backLeftSpeed = leftSpeed;
        double frontRightSpeed = rightSpeed;
        double backRightSpeed = rightSpeed;

        frontLeftMotor.getDriveMotor().set(frontLeftSpeed);
        frontRightMotor.getDriveMotor().set(frontRightSpeed);
        backLeftMotor.getDriveMotor().set(backLeftSpeed);
        backRightMotor.getDriveMotor().set(backRightSpeed);
    }

    public void swerveDrive(double ySpeed, double xSpeed, double turnSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        frontLeftMotor.setState(moduleStates[0]);
        frontRightMotor.setState(moduleStates[1]);
        backLeftMotor.setState(moduleStates[2]);
        backRightMotor.setState(moduleStates[3]);
    }

    public void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = SwerveModule.getAngleRadians(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftMotor.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightMotor.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftMotor.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightMotor.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }
}