package frc.robot.commands;

import static frc.robot.util.MathUtil.clamp;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.pieces.TimedIntakeCommand.TimedIntakeScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MathUtil;

public class ShooterCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;

    private final BooleanSupplier m_ampShotSupplier;
    private final BooleanSupplier m_speakerShotSupplier;
    //private final BooleanSupplier m_increaseShooterSpeedSupplier;
    //private final BooleanSupplier m_decreaseShooterSpeedSupplier;

    private final TimedIntakeScheduler m_intakeScheduler;

    private final double AMP_SPEED = 0.54;//0.52;

    private double currentShooterTargetSpeed = 0.85;
    private double currentSpeed = 0;
    private Mode mode;
    private boolean scheduledIntake = false;
    protected double intakeDuration = 3.0;
    //private boolean speedChangePressed = false;

    public ShooterCommand(
        ShooterSubsystem shooterSubsystem,
        BooleanSupplier ampShotSupplier,
        BooleanSupplier speakerShotSupplier,
        //BooleanSupplier increaseShooterSpeedSupplier,
        //BooleanSupplier decreaseShooterSpeedSupplier,
        TimedIntakeScheduler intakeScheduler
    ) {
        this.m_shooterSubsystem = shooterSubsystem;
        this.m_ampShotSupplier = ampShotSupplier;
        this.m_speakerShotSupplier = speakerShotSupplier;
        //this.m_increaseShooterSpeedSupplier = increaseShooterSpeedSupplier;
        //this.m_decreaseShooterSpeedSupplier = decreaseShooterSpeedSupplier;
        this.m_intakeScheduler = intakeScheduler;

        addRequirements(m_shooterSubsystem);
    }

    // Provided for subclasses to override
    protected double customTargetSpeed() {
        return Double.NaN;
    }

    // Provided for subclasses to override
    protected void onSpeedReached() {}

    public ShooterCommand withInitialSpeed(double initialSpeed) {
        this.currentSpeed = initialSpeed;
        return this;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Shooter/Target Speed", currentShooterTargetSpeed);
        /*if (m_increaseShooterSpeedSupplier.getAsBoolean()) {
            if (!speedChangePressed) {
                currentShooterTargetSpeed += 0.05;
                speedChangePressed = true;
            }
        } else if (m_decreaseShooterSpeedSupplier.getAsBoolean()) {
            if (!speedChangePressed) {
                currentShooterTargetSpeed -= 0.05;
                speedChangePressed = true;
            }
        } else {
            speedChangePressed = false;
        }*/
        currentShooterTargetSpeed = clamp(currentShooterTargetSpeed, 0, 1);

        double targetSpeed = 0;
        mode = Mode.OTHER;
        if (m_ampShotSupplier.getAsBoolean()) {
            targetSpeed = AMP_SPEED * -1;
            mode = Mode.AMP;
        } else if (m_speakerShotSupplier.getAsBoolean()) {
            targetSpeed = currentShooterTargetSpeed * -1;
            mode = Mode.SHOOTER;
        }
        {
            double customTargetSpeed = customTargetSpeed();
            if (!Double.isNaN(customTargetSpeed)) {
                targetSpeed = customTargetSpeed;
                mode = Mode.OTHER;
            }
        }

        if (!mode.intakeFollows) {
            scheduledIntake = false;
        }

        currentSpeed = MathUtil.lerp(currentSpeed, targetSpeed, 0.1);
        if (Math.abs(targetSpeed) < 0.03) {
            targetSpeed = 0;
        }
        if (Math.abs(currentSpeed - targetSpeed) < 0.03) {
            currentSpeed = targetSpeed;
            if (mode.intakeFollows && !scheduledIntake) {
                scheduledIntake = true;
                m_intakeScheduler.schedule(0.05, intakeDuration);
            }
            onSpeedReached();
        }
        //System.out.println("Shooter speed: "+currentSpeed);
        m_shooterSubsystem.setSpeed(currentSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
    }

    private static enum Mode {
        SHOOTER(true),
        AMP(true),
        OTHER(false)
        ;
        public final boolean intakeFollows;

        private Mode(boolean intakeFollows) {
            this.intakeFollows = intakeFollows;
        }
    }
}
