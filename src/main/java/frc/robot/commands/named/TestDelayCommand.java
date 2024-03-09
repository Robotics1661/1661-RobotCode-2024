package frc.robot.commands.named;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.subsystems.AutonomousInput;

// 2-second delay
public class TestDelayCommand extends Command {
    private double startTime;

    @INamedCommand("test_delay")
    public TestDelayCommand(AutonomousInput ai) {}

    @Override
    public void initialize() {
        super.initialize();

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= (startTime + 2.0);
    }
}
