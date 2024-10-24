package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double longitudinalPosition;
    private final double lateralPosition;

    public AutonomousDriveCommand(DriveSubsystem subsystem, double longitudinalPosition, double lateralPosition) {
        driveSubsystem = subsystem;
        this.longitudinalPosition = longitudinalPosition;
        this.lateralPosition = lateralPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.swerveDriveTo(longitudinalPosition, lateralPosition);
    }

    @Override
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
}
