package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.DriveState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.spline.CubicHermiteSpline;

public class AutonomousDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    
    private DriveState[] spline;
    private int point = 1;

    public AutonomousDriveCommand(DriveSubsystem subsystem, DriveState[] states) {
        driveSubsystem = subsystem;
        getSmoothStates(states);
        addRequirements(subsystem);
    }

    public void getSmoothStates(DriveState[] states) {
        // CubicHermiteSpline b = new CubicHermiteSpline(null, null, null, null)
        // ControlVector a = new ControlVector(null, null);
        // int index = 0;
        // for (DriveState state : states) {
        //     double lateralPosition = state.getLateralPosition();
        //     double longitundalPosition = state.getLongitudinalPosition();
        //     index += state.getPoints();
        // }
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // driveSubsystem.swerveDrivePosition(longitudinalPosition, lateralPosition, angle);
        point++;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
        // driveSubsystem.isAtPosition(longitudinalPosition, lateralPosition);
    }
}
