package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;

public class InitFourBarCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;

    /**
     * MUST be called within a Trigger#whileTrue
     */
    public InitFourBarCommand(FourBarSubsystem m_fourBarSubsystem) {
        this.m_fourBarSubsystem = m_fourBarSubsystem;

        addRequirements(m_fourBarSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        m_fourBarSubsystem.enterInitMode();
    }

    @Override
    public void execute() {
        super.execute();

        if (m_fourBarSubsystem.runInitializationProcedure()) { // complete!
            m_fourBarSubsystem.exitInitMode();
            cancel();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        m_fourBarSubsystem.stop();
    }
}
