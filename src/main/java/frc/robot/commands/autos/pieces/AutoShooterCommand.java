package frc.robot.commands.autos.pieces;

import static frc.robot.util.AutonomousDebugger.autoDbg;

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
        intakeDuration = 0.75;
    }

    @Override
    public void initialize() {
        super.initialize();

        autoDbg("Started auto-shooter command");
    }

    private void scheduleCancel() {
        endTime = Timer.getFPGATimestamp() + 1.0;
        autoDbg("Scheduled auto-shooter end");
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
    }

    private static enum Target {
        AMP,
        SPEAKER
    }
}
