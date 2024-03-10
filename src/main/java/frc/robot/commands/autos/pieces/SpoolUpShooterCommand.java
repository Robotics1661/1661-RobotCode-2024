package frc.robot.commands.autos.pieces;

import static frc.robot.util.AutonomousDebugger.autoDbg;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SpoolUpShooterCommand extends ShooterCommand {
    private final double m_speed;
    private boolean done = false;

    public SpoolUpShooterCommand(ShooterSubsystem shooterSubsystem, double speed) {
        super(
            shooterSubsystem,
            () -> false,
            () -> false,
            (a, b) -> new InstantCommand()
        );
        this.m_speed = speed;
    }

    @Override
    public void initialize() {
        super.initialize();

        autoDbg("Spooling up shooter");
    }

    @Override
    protected double customTargetSpeed() {
        return m_speed;
    }

    @Override
    protected void onSpeedReached() {
        done = true;
        autoDbg("Shooter spooled up");
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || done;
    }
}
