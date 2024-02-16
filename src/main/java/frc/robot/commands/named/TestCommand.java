package frc.robot.commands.named;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.annotationprocessor.INamedCommand;

public class TestCommand extends InstantCommand {
    private TestCommand() {
        super(() -> {
            System.out.println("\n\n\n\nRunning test_command\n\n\n\n");
            SmartDashboard.putNumber("test_command_times", SmartDashboard.getNumber("test_command_times", 0)+1);
        });
    }

    @INamedCommand("test_command")
    private static Command build(DrivetrainSubsystem drivetrainSubsystem, FourBarSubsystem fourBarSubsystem,
            IntakeSubsystem intakeSubsystem) {
        SmartDashboard.putNumber("test_command_times", 0);
        return new TestCommand();
    }
}
