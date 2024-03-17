package frc.robot.commands.autos.pieces;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutonomousMode.CommandBuilder;
import frc.robot.subsystems.AutonomousInput;

public class InitPieces {
    public static enum StartPoint {
        AMP_SIDE("AmpSideInit"),
        FRONT("FrontInit"),
        SOURCE_SIDE("SourceSideInit")
        ;

        public final String m_pathPlannerName;

        StartPoint(String pathPlannerName) {
            this.m_pathPlannerName = pathPlannerName;
        }
    }

    public static CommandBuilder startAt(StartPoint start) {
        return (AutonomousInput autonomousInput) -> {
            return new PathPlannerAuto(start.m_pathPlannerName);
        };
    }

    public static Command createStartAt(StartPoint start) {
        return new PathPlannerAuto(start.m_pathPlannerName);
    }
}
