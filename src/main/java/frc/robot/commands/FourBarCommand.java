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
    private final BooleanSupplier m_goFarSpeakerSupplier;
    private final BooleanSupplier m_increaseOffsetSupplier;
    private final BooleanSupplier m_decreaseOffsetSupplier;
    private boolean stopped = false;
    private boolean offsetChangePressed = false;

    public FourBarCommand(
        FourBarSubsystem fourBarSubsystem, DoubleSupplier speedSupplier,
        BooleanSupplier goOriginSupplier, BooleanSupplier goIntakeSupplier,
        BooleanSupplier goAmpSupplier, BooleanSupplier goSpeakerSupplier,
        BooleanSupplier goFarSpeakerSupplier,
        BooleanSupplier increaseOffsetSupplier, BooleanSupplier decreaseOffsetSupplier
        ) {
        this.m_fourBarSubsystem = fourBarSubsystem;
        this.m_speedSupplier = speedSupplier;
        this.m_goOriginSupplier = goOriginSupplier;
        this.m_goIntakeSupplier = goIntakeSupplier;
        this.m_goAmpSupplier = goAmpSupplier;
        this.m_goSpeakerSupplier = goSpeakerSupplier;
        this.m_goFarSpeakerSupplier = goFarSpeakerSupplier;
        this.m_increaseOffsetSupplier = increaseOffsetSupplier;
        this.m_decreaseOffsetSupplier = decreaseOffsetSupplier;

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

            if (m_increaseOffsetSupplier.getAsBoolean()) {
                if (!offsetChangePressed) {
                    offsetChangePressed = true;
                    m_fourBarSubsystem.increaseSetPointOffset();
                }
            } else if (m_decreaseOffsetSupplier.getAsBoolean()) {
                if (!offsetChangePressed) {
                    offsetChangePressed = true;
                    m_fourBarSubsystem.decreaseSetPointOffset();
                }
            } else {
                offsetChangePressed = false;
            }

            if (m_goOriginSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.ORIGIN);
            } else if (m_goIntakeSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.INTAKE);
            } else if (m_goAmpSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.AMP);
            } else if (m_goSpeakerSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.SPEAKER);
            } else if (m_goFarSpeakerSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.FAR_SPEAKER_AUTO);
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
