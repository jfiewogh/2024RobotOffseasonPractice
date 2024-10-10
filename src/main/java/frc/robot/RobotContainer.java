// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, joystick));
        configureBindings();
    }

    private void configureBindings() {
        
        // Test Buttons
        new JoystickButton(joystick, Button.B4.getPort()).onTrue(new InstantCommand(() -> driveSubsystem.getEncoderValues()));
        // new JoystickButton(joystick, Button.RB.getPort()).onTrue(new InstantCommand(() -> driveSubsystem.getGyroValue()));
        new JoystickButton(joystick, Button.B1.getPort()).onTrue(new InstantCommand(() -> System.out.println(intakeSubsystem.getIntakeDeployRelativePosition())));


        /* INTAKE */

        new JoystickButton(joystick, Button.RB.getPort()).onTrue(new IntakeDeployCommand(intakeSubsystem, true));
        new JoystickButton(joystick, Button.RT.getPort()).onTrue(new IntakeDeployCommand(intakeSubsystem, false));

        new JoystickButton(joystick, Button.LB.getPort()).onTrue(new IntakeRollerCommand(intakeSubsystem, 0.3));
        new JoystickButton(joystick, Button.LT.getPort()).onTrue(new IntakeRollerCommand(intakeSubsystem, -0.3));

        new JoystickButton(joystick, Button.B2.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(-1)));
        new JoystickButton(joystick, Button.B3.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(1)));
    }
}

