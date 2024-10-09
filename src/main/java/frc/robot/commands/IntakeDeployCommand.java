package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeDeployCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final boolean isDeploy;

    public IntakeDeployCommand(IntakeSubsystem subsystem, boolean isDeploy) {
        intakeSubsystem = subsystem;
        this.isDeploy = isDeploy;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (isDeploy) {
            intakeSubsystem.setIntakeDeployMotor(IntakeConstants.kDeployPosition);
        } else {
            intakeSubsystem.setIntakeDeployMotor(IntakeConstants.kRetractPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
