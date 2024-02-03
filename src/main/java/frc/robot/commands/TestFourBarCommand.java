package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;

public class TestFourBarCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;

    private final DoubleSupplier m_speedSupplier;

    public TestFourBarCommand(FourBarSubsystem fourBarSubsystem, DoubleSupplier speedSupplier) {
        this.m_fourBarSubsystem = fourBarSubsystem;
        this.m_speedSupplier = speedSupplier;

        addRequirements(m_fourBarSubsystem);
    }

    @Override
    public void execute() {
        m_fourBarSubsystem.setSpeed(m_speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_fourBarSubsystem.stop();
    }
}
