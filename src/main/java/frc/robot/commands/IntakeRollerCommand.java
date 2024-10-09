package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public IntakeRollerCommand(IntakeSubsystem subsystem, double speed) {
        intakeSubsystem = subsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeRollerMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
