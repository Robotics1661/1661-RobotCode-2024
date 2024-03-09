package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final BooleanSupplier m_runSupplier;
    private final DoubleSupplier m_reverseSupplier;
    private boolean m_previous;

    private double reverseEndTime = -1000;

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
        boolean run = m_runSupplier.getAsBoolean();
        if (reverse > 0.001) {
            m_previous = !run;
            m_intakeSubsystem.reverse(reverse);
            return;
        }
        if (run != m_previous) {
            m_previous = run;
            if (run) {
                reverseEndTime = Double.NaN;
                m_intakeSubsystem.run();
            } else {
                m_intakeSubsystem.stop();
            }
        }

        if (!run) {
            if (Double.isNaN(reverseEndTime)) {
                reverseEndTime = Timer.getFPGATimestamp() + 0.25;
            } else if (Timer.getFPGATimestamp() < reverseEndTime) {
                m_intakeSubsystem.reverse(1.0);
            } else if (Timer.getFPGATimestamp() < reverseEndTime + 0.15) {
                m_intakeSubsystem.reverse(0.0);
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
