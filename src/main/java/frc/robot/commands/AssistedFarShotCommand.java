//*
package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonomousInput;

public class AssistedFarShotCommand {
    public static Command create(AutonomousInput autonomousInput) {
        return new SequentialCommandGroup(
            seedInSim(),
            autonomousInput.drivetrainSubsystem().pathfindTo(new Pose2d(
                new Translation2d(2.94, 5.52),
                Rotation2d.fromDegrees(180)
            ))
        );
    }

    private static Command seedInSim() {
        if (Utils.isSimulation()) {
            return new PathPlannerAuto("SimSeed");
        } else {
            return new InstantCommand();
        }
    }
}
// */