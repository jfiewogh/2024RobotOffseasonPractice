package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;
        addRequirements(intakeSubsystem);
    }

    public void initialize() {
        intakeSubsystem.runIntakeMotor(0.5);
        intakeSubsystem.runTransitionMotor(0.5);
    }

    public void execute() {
    }

    public void end() {
    }
}
