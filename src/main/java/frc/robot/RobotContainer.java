// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Controller.Button;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.AutoSwerveConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Controller controller = new Controller(OperatorConstants.kDriverControllerPort);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, controller);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, controller));
        configureBindings();
    }

    private void configureBindings() { 
        // Test
        controller.getButton(Button.X).onTrue(new InstantCommand(() -> driveSubsystem.printEncoderValues()));
        controller.getButton(Button.A).onTrue(new InstantCommand(() -> driveSubsystem.printGyroValue()));
        // controller.getButton(Button.B).onTrue(new InstantCommand(() -> System.out.println(intakeSubsystem.getIntakeDeployRelativePosition())));
        controller.getButton(Button.Y).onTrue(new InstantCommand(() -> driveCommand.printJoystickAxes()));
        controller.getButton(Button.B).onTrue(new InstantCommand(() -> driveSubsystem.printOdometerPose()));

        controller.getButton(Button.Start).onTrue(new InstantCommand(() -> driveSubsystem.reset()));

        /* INTAKE */
        controller.getButton(Button.RB).onTrue(new IntakeDeployCommand(intakeSubsystem, true));
        controller.getButton(Button.RT).onTrue(new IntakeDeployCommand(intakeSubsystem, false));
        controller.getButton(Button.LB).onTrue(new IntakeRollerCommand(intakeSubsystem, 0.3, 0.5));
        controller.getButton(Button.LT).onTrue(new IntakeRollerCommand(intakeSubsystem, -0.3, 0.5));

        // new JoystickButton(joystick, Button.B2.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(-1)));
        // new JoystickButton(joystick, Button.B3.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(1)));
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxAccelerationMetersPerSecondSquared);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0.2, 0.1),
                new Translation2d(0.5, 0.15)
            ),
            new Pose2d(1, 0.2, Rotation2d.fromDegrees(25)),
            trajectoryConfig
        );

        // THESE are the problems
        // drive doesn't stop
        // turn doesn't even start (just errors)


        PIDController xController = new PIDController(AutoSwerveConstants.kXP, 0, 0);
        PIDController yController = new PIDController(AutoSwerveConstants.kYP, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoSwerveConstants.kThetaP, 0, 0, AutoSwerveConstants.kThetaConstraints);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            driveSubsystem::getPose, 
            driveSubsystem.getKinematics(), 
            xController,
            yController,
            thetaController,
            driveSubsystem::setModuleStates, 
            driveSubsystem
        );

        return new SequentialCommandGroup(
            // Drive
            swerveControllerCommand,
            // Intake
            new IntakeDeployCommand(intakeSubsystem, true),
            new IntakeRollerCommand(intakeSubsystem, 0.3, 0.5),
            new IntakeDeployCommand(intakeSubsystem, false)
        );
    }
}

