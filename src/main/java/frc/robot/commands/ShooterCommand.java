package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MathUtil;

public class ShooterCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;

    private final DoubleSupplier m_speedSupplier;

    private double currentSpeed = 0;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier speedSupplier) {
        this.m_shooterSubsystem = shooterSubsystem;
        this.m_speedSupplier = speedSupplier;

        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        double targetSpeed = m_speedSupplier.getAsDouble();
        currentSpeed = MathUtil.lerp(currentSpeed, targetSpeed, 0.05);
        if (Math.abs(currentSpeed) < 0.01 && Math.abs(targetSpeed) < 0.01) {
            currentSpeed = 0;
        }
        if (Math.abs(currentSpeed) > 0.99) {
            currentSpeed = 1 * Math.signum(currentSpeed);
        }
        //System.out.println("Shooter speed: "+currentSpeed);
        m_shooterSubsystem.setSpeed(currentSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
    }
}
