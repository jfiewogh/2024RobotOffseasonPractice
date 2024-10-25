package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Controller;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private enum DriveType {
        ARCADE, SWERVE, NONE;
    }

    private final DriveSubsystem driveSubsystem;
    private final Controller controller;

    private final DriveType driveType = DriveType.SWERVE;

    public DriveCommand(DriveSubsystem subsystem, Controller controller) {
        driveSubsystem = subsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Begin drive command");
    }

    @Override
    public void execute() {
        switch (driveType) {
            case ARCADE:
                driveSubsystem.arcadeDrive(controller.getLeftStickY() * 0.5, controller.getRightStickX() * 0.5);
                break;
            case SWERVE:
                driveSubsystem.swerveDriveSpeeds(controller.getLeftStickX(), controller.getLeftStickY(), controller.getRightStickX());
                break;
            default:
                driveSubsystem.spin();
                break;
        }
    } 

    @Override
    public void end(boolean interrupted) {
        System.out.println("End drive command");
    }

    public void printJoystickAxes() {
        System.out.println("LX: " + controller.getLeftStickX());
        System.out.println("LY: " + controller.getLeftStickY());
        System.out.println("RX: " + controller.getRightStickX());
    }

    public boolean isFinished() {
        return false;
    }
}


