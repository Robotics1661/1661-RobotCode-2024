package frc.robot.commands.named;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.subsystems.AutonomousInput;

public class TestCommand extends InstantCommand {
    @INamedCommand("test_command")
    public TestCommand(AutonomousInput autonomousInput) {
        super(() -> {
            System.out.println("\n\n\n\nRunning test_command\n\n\n\n");
            SmartDashboard.putNumber("test_command_times", SmartDashboard.getNumber("test_command_times", 0)+1);
        });

        SmartDashboard.putNumber("test_command_times", 0);
    }

    // * Another valid way to register
    @INamedCommand("test_command_method")
    public static TestCommand build(AutonomousInput autonomousInput) {
        SmartDashboard.putNumber("test_command_times", 0);
        return new TestCommand(autonomousInput);
    }
    // */
}
