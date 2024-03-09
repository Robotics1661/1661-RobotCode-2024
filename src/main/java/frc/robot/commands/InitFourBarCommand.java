package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;

public class InitFourBarCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;

    private double startTime = 0;

    private boolean finished = false;

    /**
     * MUST be called within a Trigger#whileTrue for safety(tm)
     */
    public InitFourBarCommand(FourBarSubsystem m_fourBarSubsystem) {
        this.m_fourBarSubsystem = m_fourBarSubsystem;

        addRequirements(m_fourBarSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        m_fourBarSubsystem.enterInitMode();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        super.execute();

        if (Utils.isSimulation()) {
            m_fourBarSubsystem.runInitializationProcedure();
            if (Timer.getFPGATimestamp() > startTime + 2) {
                m_fourBarSubsystem.exitInitMode();
                cancel();
            }
        } else if (m_fourBarSubsystem.runInitializationProcedure() // complete!
        && Timer.getFPGATimestamp() > startTime + 0.5) { // give 1/2 second min time
            m_fourBarSubsystem.exitInitMode();
            cancel();
        }
    }

    @Override
    public void cancel() {
        super.cancel();

        finished = true;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        m_fourBarSubsystem.endOfRoutineStop();
    }
}
