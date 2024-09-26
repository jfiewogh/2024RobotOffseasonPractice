// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
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
  private final JoystickButton button1 = new JoystickButton(joystick, Button.LB.getPort());
  private final JoystickButton button2 = new JoystickButton(joystick, Button.RB.getPort());

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, joystick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Default command means it will constantly run 
    driveSubsystem.setDefaultCommand(driveCommand);
    button1.whileTrue(new InstantCommand(() -> driveSubsystem.getEncoderValues()));
    button2.onTrue(new InstantCommand(() -> driveSubsystem.printDesiredStates = true));
  }
}

