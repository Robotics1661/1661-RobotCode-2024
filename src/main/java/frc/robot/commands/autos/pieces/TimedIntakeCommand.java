package frc.robot.commands.autos.pieces;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final double m_delaySeconds;
    private final double m_durationSeconds;

    private double startTime;

    public TimedIntakeCommand(IntakeSubsystem intakeSubsystem, double delaySeconds, double durationSeconds) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_delaySeconds = delaySeconds;
        this.m_durationSeconds = durationSeconds;
    }

    @Override
    public void initialize() {
        super.initialize();

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= (startTime + m_delaySeconds + m_durationSeconds);
    }

    @Override
    public void execute() {
        super.execute();

        if (Timer.getFPGATimestamp() >= (startTime + m_delaySeconds)) {
            m_intakeSubsystem.runToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        m_intakeSubsystem.stop();
    }

    @FunctionalInterface
    public static interface TimedIntakeScheduler {
        Command create(double delaySeconds, double durationSeconds);

        default TimedIntakeScheduler afterCommand(Supplier<Runnable> runnable) {
            return (delaySeconds, durationSeconds) -> {
                return create(delaySeconds, durationSeconds).andThen(runnable.get());
            };
        }

        default TimedIntakeScheduler afterSchedule(Supplier<Runnable> runnable) {
            return (delaySeconds, durationSeconds) -> {
                Command cmd = create(delaySeconds, durationSeconds);
                runnable.get().run();
                return cmd;
            };
        }

        default void schedule(double delaySeconds, double durationSeconds) {
            create(delaySeconds, durationSeconds).schedule();
        }
    }

    public static TimedIntakeScheduler makeScheduler(IntakeSubsystem intakeSubsystem) {
        return (delay, duration) -> new TimedIntakeCommand(intakeSubsystem, delay, duration);
    }
}
