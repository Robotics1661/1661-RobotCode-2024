package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final BooleanSupplier m_runSupplier;
    private final DoubleSupplier m_reverseSupplier;
    private boolean m_previous;

    public IntakeCommand(
        IntakeSubsystem intakeSubsystem,
        BooleanSupplier runSupplier,
        DoubleSupplier reverseSupplier
    ) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_runSupplier = runSupplier;
        this.m_reverseSupplier = reverseSupplier;
        this.m_previous = false;

        addRequirements(m_intakeSubsystem);
    }

    

    @Override
    public void execute() {
        double reverse = m_reverseSupplier.getAsDouble();
        if (reverse > 0.001) {
            m_intakeSubsystem.reverse(reverse);
            return;
        }
        boolean run = m_runSupplier.getAsBoolean();
        if (run != m_previous) {
            m_previous = run;
            if (run) {
                m_intakeSubsystem.run();
            } else {
                m_intakeSubsystem.stop();
            }
        }
    }



    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stop();
    }
}
