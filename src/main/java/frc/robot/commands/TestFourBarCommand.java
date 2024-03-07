package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

public class TestFourBarCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;

    private final DoubleSupplier m_speedSupplier;
    private final BooleanSupplier m_goForwardSupplier;
    private final BooleanSupplier m_goBackwardSupplier;
    private final BooleanSupplier m_stepEnableSupplier;
    private final BooleanSupplier m_stepForwardSupplier;
    private final BooleanSupplier m_stepBackwardSupplier;
    private boolean stopped = false;
    
    private boolean stepButtonPressed = false;

    public TestFourBarCommand(
        FourBarSubsystem fourBarSubsystem, DoubleSupplier speedSupplier,
        BooleanSupplier goForwardSupplier, BooleanSupplier goBackwardSupplier,
        BooleanSupplier stepEnableSupplier, BooleanSupplier stepForwardSupplier, BooleanSupplier stepBackwardSupplier
        ) {
        this.m_fourBarSubsystem = fourBarSubsystem;
        this.m_speedSupplier = speedSupplier;
        this.m_goForwardSupplier = goForwardSupplier;
        this.m_goBackwardSupplier = goBackwardSupplier;

        this.m_stepEnableSupplier = stepEnableSupplier;
        this.m_stepForwardSupplier = stepForwardSupplier;
        this.m_stepBackwardSupplier = stepBackwardSupplier;

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

            if (m_goForwardSupplier.getAsBoolean()) {
                m_fourBarSubsystem.setTargetPoint(SetPoints.ORIGIN);
            } else if (m_goBackwardSupplier.getAsBoolean()) {
                m_fourBarSubsystem.stop();
                //m_fourBarSubsystem.setTargetPoint(SetPoints.TEST_BACKWARD);
            } else if (m_stepEnableSupplier.getAsBoolean()) {
                if (m_stepForwardSupplier.getAsBoolean()) {
                    if (!stepButtonPressed) {
                        stepButtonPressed = true;
                        m_fourBarSubsystem.moveTargetForward();
                    }
                } else if (m_stepBackwardSupplier.getAsBoolean()) {
                    if (!stepButtonPressed) {
                        stepButtonPressed = true;
                        m_fourBarSubsystem.moveTargetBackward();
                    }
                } else {
                    stepButtonPressed = false;
                }
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
