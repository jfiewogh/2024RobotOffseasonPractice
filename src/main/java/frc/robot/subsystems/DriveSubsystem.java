package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.AbsoluteEncoder.EncoderConfig;
import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;

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

    private static final double kAtPositionThreshold = Units.inchesToMeters(12);

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0), getSwerveModulePositions());

    private final Timer timer = new Timer();

    // private final FileWriter writer;

    public DriveSubsystem() {
        timer.restart();
        // writer = new FileWriter("output.txt");
    }

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
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyroAngle());
        return getModuleStatesFromChassisSpeeds(robotRelativeSpeeds);
    }

    // Field centric
    public SwerveModuleState[] getFieldCentricModuleStates(double longitudinalSpeed, double lateralSpeed, double turnSpeed) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(longitudinalSpeed, lateralSpeed, turnSpeed, getGyroAngle());
        return getModuleStatesFromChassisSpeeds(speeds);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
    }

    public void swerveDriveSpeeds(double relativeLateralSpeed, double relativeLongitundalSpeed, double relativeRotationSpeed) {
        double lateralSpeed = relativeLateralSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
        double longitundalSpeed = relativeLongitundalSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
        double rotationSpeed = relativeRotationSpeed * SwerveConstants.kMaxRotationSpeed;
        setModuleStates(getFieldCentricModuleStates(longitundalSpeed, lateralSpeed, rotationSpeed));
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

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public Pose2d getPose() {
        System.out.println(odometer.getPoseMeters());
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
    public void printDriveEncoderValues() {
        frontLeftModule.printDriveEncoderValue("FL");
        frontRightModule.printDriveEncoderValue("FR");
        backLeftModule.printDriveEncoderValue("BL");
        backRightModule.printDriveEncoderValue("BR");
    }
    public void printGyroValue() {
        System.out.println("GYRO VALUE");
        System.out.println(getGyroAngle());
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
        odometer.resetPosition(getGyroAngle(), getSwerveModulePositions(), getPose());
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
        odometer.update(getGyroAngle(), getSwerveModulePositions());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }



    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        SwerveConstants.kMaxSpeedMetersPerSecond, 
        AutoSwerveConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.2, 0.1),
            new Translation2d(0.5, 0.15)
        ),
        new Pose2d(1, 0.2, Rotation2d.fromDegrees(25)),
        trajectoryConfig
    );

    HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(0.01, 0, 0), 
        new PIDController(0.01, 0, 0),
        new ProfiledPIDController(0.05, 0, 0, AutoSwerveConstants.kThetaConstraints));

    @Override
    public void periodic() {
        updateOdometer();

        double time = timer.get();

        if (time < 2) {
            System.out.println(time);

            Trajectory.State desiredState = trajectory.sample(time);
            System.out.println(desiredState);

            ChassisSpeeds targetSpeeds = controller.calculate(getPose(), desiredState, new Rotation2d(0));
            System.out.println(targetSpeeds);
        }
    }
}