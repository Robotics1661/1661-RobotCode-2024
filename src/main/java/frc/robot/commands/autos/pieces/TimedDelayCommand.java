package frc.robot.commands.autos.pieces;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TimedDelayCommand extends Command {
    private final double m_durationSeconds;

    private double startTime;

    public TimedDelayCommand(double durationSeconds) {
        this.m_durationSeconds = durationSeconds;
    }

    @Override
    public void initialize() {
        super.initialize();

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > (startTime + m_durationSeconds);
    }
}
