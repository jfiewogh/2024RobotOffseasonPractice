package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;;

public class DriveSubsystem extends SubsystemBase {
    public Boolean printDesiredStates = false;

    private final double width = Units.inchesToMeters(19.75);
    private final double length = Units.inchesToMeters(19.75);

    private final Translation2d frontLeftLocation = new Translation2d(width/2, length/2);
    private final Translation2d frontRightLocation = new Translation2d(width/2, -length/2);
    private final Translation2d backLeftLocation = new Translation2d(-width/2, length/2);
    private final Translation2d backRightLocation = new Translation2d(-width/2, -length/2);

    private final SwerveDriveKinematics kinematics;

    private final EncoderConfig frontLeftConfig = EncoderConfig.FRONT_LEFT;
    private final EncoderConfig frontRightConfig = EncoderConfig.FRONT_RIGHT;
    private final EncoderConfig backLeftConfig = EncoderConfig.BACK_LEFT;
    private final EncoderConfig backRightConfig = EncoderConfig.BACK_RIGHT;

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(1, 2, frontLeftLocation, frontLeftConfig);
    private final SwerveModule frontRightSwerveModule = new SwerveModule(3, 4, frontRightLocation, frontRightConfig);
    private final SwerveModule backLeftSwerveModule = new SwerveModule(5, 6, backLeftLocation, backLeftConfig);
    private final SwerveModule backRightSwerveModule = new SwerveModule(7, 8, backRightLocation, backRightConfig);

    public DriveSubsystem() {
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        double leftSpeed = forwardSpeed + turnSpeed;
        double rightSpeed = forwardSpeed - turnSpeed;
        frontLeftSwerveModule.setDriveMotorSpeed(leftSpeed);
        frontRightSwerveModule.setDriveMotorSpeed(leftSpeed);
        backLeftSwerveModule.setDriveMotorSpeed(rightSpeed);
        backRightSwerveModule.setDriveMotorSpeed(rightSpeed);
    }

    public SwerveModuleState[] getModuleStates(double ySpeed, double xSpeed, double turnSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        return kinematics.toSwerveModuleStates(speeds);
    }

    public void swerveDrive(double ySpeed, double xSpeed, double turnSpeed) {
        SwerveModuleState[] moduleStates = getModuleStates(ySpeed, xSpeed, turnSpeed);
        for(int i = 0; i < moduleStates.length; i++) {
            moduleStates[i].angle = Rotation2d.fromRadians(-moduleStates[i].angle.getRadians());
        }
        if (printDesiredStates) {
            for (int i = 0; i < moduleStates.length; i++) {
                System.out.print(moduleStates[i].angle.getRadians() + " ");
            }
            System.out.println();
            printDesiredStates = false;
        }
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
        System.out.println("FL: " + frontLeftSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + frontLeftSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + frontLeftSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("FR: " + frontRightSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + frontRightSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + frontRightSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("BL: " + backLeftSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + backLeftSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + backLeftSwerveModule.getAngleMotorAbsoluteEncoderRotations());
        System.out.println("BR: " + backRightSwerveModule.getAngleMotorRelativeEncoderRotations() + " " + backRightSwerveModule.getAngleWheelAbsoluteEncoderRotations() + " " + backRightSwerveModule.getAngleMotorAbsoluteEncoderRotations());
    }
}