package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

public class FourBarCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;

    private final DoubleSupplier m_speedSupplier;
    private final BooleanSupplier m_goOriginSupplier;
    private final BooleanSupplier m_goIntakeSupplier;
    private final BooleanSupplier m_goAmpSupplier;
    private final BooleanSupplier m_goSpeakerSupplier;
    private boolean stopped = false;

    public FourBarCommand(
        FourBarSubsystem fourBarSubsystem, DoubleSupplier speedSupplier,
        BooleanSupplier goOriginSupplier, BooleanSupplier goIntakeSupplier,
        BooleanSupplier goAmpSupplier, BooleanSupplier goSpeakerSupplier
        ) {
        this.m_fourBarSubsystem = fourBarSubsystem;
        this.m_speedSupplier = speedSupplier;
        this.m_goOriginSupplier = goOriginSupplier;
        this.m_goIntakeSupplier = goIntakeSupplier;
        this.m_goAmpSupplier = goAmpSupplier;
        this.m_goSpeakerSupplier = goSpeakerSupplier;

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
                m_fourBarSubsystem.activeStop();
                stopped = true;
            }

            if (m_goOriginSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.ORIGIN);
            } else if (m_goIntakeSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.INTAKE);
            } else if (m_goAmpSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.AMP);
            } else if (m_goSpeakerSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.SPEAKER);
            } else { // safety stop - require button to be held in for motion
                m_fourBarSubsystem.activeStop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_fourBarSubsystem.endOfRoutineStop();
    }
}
