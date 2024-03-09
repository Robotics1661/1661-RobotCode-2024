package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    private final ClimberSubsystem m_climberSubsystem;

    private final BooleanSupplier m_extendSupplier;
    private final BooleanSupplier m_retractSupplier;

    public ClimberCommand(
        ClimberSubsystem climberSubsystem,
        BooleanSupplier extendSupplier, BooleanSupplier retractSupplier
    ) {
        this.m_climberSubsystem = climberSubsystem;

        this.m_extendSupplier = extendSupplier;
        this.m_retractSupplier = retractSupplier;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        super.execute();

        if (m_extendSupplier.getAsBoolean()) {
            m_climberSubsystem.extend();
        } else if (m_retractSupplier.getAsBoolean()) {
            m_climberSubsystem.retract();
        } else {
            m_climberSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        m_climberSubsystem.stop();
    }
}
