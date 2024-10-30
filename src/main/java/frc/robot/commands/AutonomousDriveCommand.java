package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonomousDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double longitudinalPosition;
    private final double lateralPosition;
    private final Rotation2d angle;

    public AutonomousDriveCommand(DriveSubsystem subsystem, double longitudinalPosition, double lateralPosition, Rotation2d angle) {
        driveSubsystem = subsystem;
        this.longitudinalPosition = longitudinalPosition;
        this.lateralPosition = lateralPosition;
        this.angle = angle;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.swerveDrivePosition(longitudinalPosition, lateralPosition, angle);
    }

    @Override
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return driveSubsystem.isAtPosition(longitudinalPosition, lateralPosition);
    }
}
