package frc.robot.commands.autos.pieces;

import static frc.robot.util.SimulationDebugger.simDbg;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand.TimedIntakeScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MutableObject;

public class AutoShooterCommand extends ShooterCommand {
    public static AutoShooterCommand amp(ShooterSubsystem shooterSubsystem, TimedIntakeScheduler intakeScheduler) {
        MutableObject<Runnable> callback = new MutableObject<Runnable>(() -> {});
        AutoShooterCommand cmd = new AutoShooterCommand(
            shooterSubsystem,
            intakeScheduler
                .afterCommand(callback)
                .afterSchedule(() -> () -> simDbg("Scheduled AMP auto shot")),
            Target.AMP
        );
        callback.set(cmd::scheduleCancel);
        return cmd;
    }

    public static AutoShooterCommand speaker(ShooterSubsystem shooterSubsystem, TimedIntakeScheduler intakeScheduler) {
        MutableObject<Runnable> callback = new MutableObject<Runnable>(() -> {});
        AutoShooterCommand cmd = new AutoShooterCommand(
            shooterSubsystem,
            intakeScheduler
                .afterCommand(callback)
                .afterSchedule(() -> () -> simDbg("Scheduled SPEAKER auto shot")),
            Target.SPEAKER
        );
        callback.set(cmd::scheduleCancel);
        return cmd;
    }

    private double endTime = Double.NaN;

    private AutoShooterCommand(ShooterSubsystem shooterSubsystem, TimedIntakeScheduler intakeScheduler, Target target) {
        super(
            shooterSubsystem,
            () -> {
                return target == Target.AMP;
            },
            () -> {
                return target == Target.SPEAKER;
            },
            () -> false,
            () -> false,
            intakeScheduler
        );
    }

    @Override
    public void initialize() {
        super.initialize();

        simDbg("Started auto-shooter command");
    }

    private void scheduleCancel() {
        endTime = Timer.getFPGATimestamp() + 1.5;
        simDbg("Scheduled auto-shooter end");
    }

    @Override
    public boolean isFinished() {
        if (super.isFinished()) return true;

        return !Double.isNaN(endTime) && Timer.getFPGATimestamp() > endTime;
    }

    private static enum Target {
        AMP,
        SPEAKER
    }
}
