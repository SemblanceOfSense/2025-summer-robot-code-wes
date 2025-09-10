package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ClawTestCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public ClawTestCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem; addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //Nothing
    }

    @Override
    public void execute() {
        intakeSubsystem.setClawVel(IntakeConstants.clawMaxAngVel);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setClawVel(0);
    }
}
