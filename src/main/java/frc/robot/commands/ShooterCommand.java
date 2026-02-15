package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double axis;

    public ShooterCommand(ShooterSubsystem shooter, double axis) {
        this.shooter = shooter;
        this.axis = axis;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPower(ShooterConstants.MaxPower * this.axis);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
