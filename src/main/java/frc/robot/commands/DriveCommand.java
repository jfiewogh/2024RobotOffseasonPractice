package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private enum DriveType {
        ARCADE, SWERVE, NONE;
    }

    private final DriveSubsystem driveSubsystem;
    private final Joystick joystick;

    private final DriveType driveType = DriveType.SWERVE;

    public DriveCommand(DriveSubsystem subsystem, Joystick joystick) {
        driveSubsystem = subsystem;
        this.joystick = joystick;
        addRequirements(driveSubsystem);
    }

    public void initialize() {}

    public void execute() {
        switch (driveType) {
            case ARCADE:
                driveSubsystem.arcadeDrive(getLeftJoystickY() * 0.5, getRightJoystickX() * 0.5);
                break;
            case SWERVE:
                driveSubsystem.swerveDrive(getLeftJoystickX(), getLeftJoystickY(), getRightJoystickX());
                break;
            default:
                break;
        }
    } 

    public void end() {
        driveSubsystem.swerveDrive(0, 0, 0);
    }

    private double getLeftJoystickX() {
        return joystick.getRawAxis(0);
    }
    
    private double getLeftJoystickY() {
        return joystick.getRawAxis(1);
    }
    
    private double getRightJoystickX() {
        return joystick.getRawAxis(2);
    }
    
    @SuppressWarnings("unused")
    private double getRightJoystickY() {
        return joystick.getRawAxis(3);
    }

    public boolean isFinished() {
        return false;
    }
}


