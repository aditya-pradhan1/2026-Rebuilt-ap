package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.MaxPower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final double axis;

    public IntakeCommand(IntakeSubsystem intake, double axis) {
        this.intake = intake;
        this.axis = axis;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(MaxPower * this.axis);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
