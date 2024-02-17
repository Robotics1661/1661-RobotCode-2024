package frc.robot.commands.named;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PathfindToCommand  {
    //@INamedCommand("pathfind_to")
    public static Command build(DrivetrainSubsystem drivetrainSubsystem, FourBarSubsystem fourBarSubsystem, IntakeSubsystem intakeSubsystem) {
        return drivetrainSubsystem.pathfindTo(PathPlannerPath.fromPathFile("Non Connected Path"));
    }
}
