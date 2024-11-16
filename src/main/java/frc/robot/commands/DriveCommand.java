package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Controller;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private enum DriveType {
        ARCADE, 
        SWERVE, 
        DRIVE, // spins the drive motors // determine direction
        SPIN, // spins the angle motors // determine direction
        TEST;
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
                driveSubsystem.arcadeDrive(getLeftStickY(), getRightStickX());
                break;
            case SWERVE:
                driveSubsystem.swerveDriveSpeeds(getLeftStickX(), getLeftStickY(), getRightStickX());
                break;
            case DRIVE:
                driveSubsystem.drive();
                break;
            case SPIN:
                driveSubsystem.spin();
                break;
            default:
                break;
        }
        driveSubsystem.updateOdometer();
    } 

    public double getLeftStickX() {
        return controller.getLeftStickX();
    }
    public double getLeftStickY() {
        return controller.getLeftStickY();
    }
    public double getRightStickX() {
        return controller.getRightStickX();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("End drive command");
    }

    // PRINT
    public void printJoystickAxes() {
        System.out.println("JOYSTICK AXES");
        System.out.println("LX: " + controller.getLeftStickX());
        System.out.println("LY: " + controller.getLeftStickY());
        System.out.println("RX: " + controller.getRightStickX());
        System.out.println("RY: " + controller.getRightStickY());
    }

    public boolean isFinished() {
        return false;
    }
}


