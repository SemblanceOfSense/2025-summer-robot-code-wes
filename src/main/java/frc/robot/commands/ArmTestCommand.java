package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmTestCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public ArmTestCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //Nothing
    }

    @Override
    public void execute() {
        for (int i = 0; i < 6; i++) {
            intakeSubsystem.setArmPos(Math.PI);
            Timer.delay(5);
            intakeSubsystem.setArmPos(0);
            Timer.delay(5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setArmPos(0);
    }
}
