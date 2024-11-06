package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Controller;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private enum DriveType {
        ARCADE, 
        SWERVE, 
        NONE; // spins the motor
    }

    private final DriveSubsystem driveSubsystem;
    private final Controller controller;

    private final DriveType driveType = DriveType.NONE;

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
                driveSubsystem.arcadeDrive(getLeftStickYSpeed(), controller.getRightStickX());
                break;
            case SWERVE:
                driveSubsystem.swerveDriveSpeeds(getLeftStickXSpeed(), controller.getLeftStickY(), controller.getRightStickX());
                break;
            default:
                driveSubsystem.spin();
                break;
        }
        driveSubsystem.updateOdometer();
    } 

    public double getLeftStickXSpeed() {
        return controller.getLeftStickX() * DriveConstants.kMaxDriveSpeed;
    }
    public double getLeftStickYSpeed() {
        return controller.getLeftStickY() * DriveConstants.kMaxDriveSpeed;
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


