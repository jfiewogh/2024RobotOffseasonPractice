// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
        controller.getButton(Button.LB).onTrue(new IntakeRollerCommand(intakeSubsystem, 0.3));
        controller.getButton(Button.LT).onTrue(new IntakeRollerCommand(intakeSubsystem, -0.3));

        // new JoystickButton(joystick, Button.B2.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(-1)));
        // new JoystickButton(joystick, Button.B3.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(1)));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            // Drive
            new AutonomousDriveCommand(driveSubsystem, 0, 0, 0),
            new AutonomousDriveCommand(driveSubsystem, 0.3, 0, 0),
            new AutonomousDriveCommand(driveSubsystem, 1, 0, 0),
            new AutonomousDriveCommand(driveSubsystem, 1.2, 0, 45),
            // Intake
            new IntakeDeployCommand(intakeSubsystem, true),
            new IntakeRollerCommand(intakeSubsystem, 0.3),
            new IntakeDeployCommand(intakeSubsystem, false)
        );
    }
}

