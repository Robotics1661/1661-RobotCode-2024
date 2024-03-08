package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

public class TestFourBarCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;

    private final DoubleSupplier m_speedSupplier;
    private final BooleanSupplier m_goOriginSupplier;
    private final BooleanSupplier m_goIntakeSupplier;
    private boolean stopped = false;

    public TestFourBarCommand(
        FourBarSubsystem fourBarSubsystem, DoubleSupplier speedSupplier,
        BooleanSupplier goOriginSupplier, BooleanSupplier goIntakeSupplier
        ) {
        this.m_fourBarSubsystem = fourBarSubsystem;
        this.m_speedSupplier = speedSupplier;
        this.m_goOriginSupplier = goOriginSupplier;
        this.m_goIntakeSupplier = goIntakeSupplier;

        addRequirements(m_fourBarSubsystem);
    }

    @Override
    public void execute() {
        double speed = m_speedSupplier.getAsDouble();
        if (Math.abs(speed) > 0.1) {
            m_fourBarSubsystem.setSpeed(speed);
            stopped = false;
        } else {
            if (!stopped) {
                m_fourBarSubsystem.stop();
                stopped = true;
            }

            if (m_goOriginSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.ORIGIN);
            } else if (m_goIntakeSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.INTAKE);
            } else { // safety stop - require button to be held in for motion
                m_fourBarSubsystem.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_fourBarSubsystem.stop();
    }
}
