package frc.robot.commands.autos.pieces;

import static frc.robot.util.AutonomousDebugger.autoDbg;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand.TimedIntakeScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.AutoEndException;
import frc.robot.util.AutonomousDebugger;
import frc.robot.util.MutableObject;

public class AutoShooterCommand extends ShooterCommand {
    boolean scheduledEnd = false;
    public static AutoShooterCommand amp(ShooterSubsystem shooterSubsystem, TimedIntakeScheduler intakeScheduler) {
        MutableObject<Runnable> callback = new MutableObject<Runnable>(() -> {});
        AutoShooterCommand cmd = new AutoShooterCommand(
            shooterSubsystem,
            intakeScheduler
                .afterCommand(callback)
                .afterSchedule(() -> () -> autoDbg("Scheduled AMP auto shot")),
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
                .afterSchedule(() -> () -> autoDbg("Scheduled SPEAKER auto shot")),
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
            //() -> false,
            //() -> false,
            intakeScheduler
        );
        intakeDuration = 0.8;
    }

    @Override
    public void initialize() {
        super.initialize();

        autoDbg("Started auto-shooter command");
    }

    private void scheduleCancel() {
        endTime = Timer.getFPGATimestamp() + 1.0;
        autoDbg("Scheduled auto-shooter end");
        scheduledEnd = true;
    }

    @Override
    public boolean isFinished() {
        if (super.isFinished()) return true;

        return !Double.isNaN(endTime) && Timer.getFPGATimestamp() > endTime;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        autoDbg("Auto-shooter complete");
        if (!scheduledEnd) {
            autoDbg("Auto shooter end() called without scheduling...");
            AutonomousDebugger.printAutoEnd();
            throw new AutoEndException();
        }
    }

    private static enum Target {
        AMP,
        SPEAKER
    }
}
