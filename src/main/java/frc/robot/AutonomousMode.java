package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/* Just some convenient abstraction for autonomous modes */
public enum AutonomousMode {
    /*BALANCE(AutonomousDriveBackCommand::new, "Drive to charge station"),
    FORWARD_AND_BALANCE(AutonomousDriveBackAndForthCommand::new, "Drive to charge station, keep going, then drive back to charge station"),
    FULL_20_POINT(Full20PointAutoCommand::new, "Place cube at top rung, drive to charge station, keep going, then drive back to charge station"),
    PLACE_CUBE_AND_TAXI_LONG(PlaceCubeAndTaxiLongCommand::new, "Place cube at top rung and taxi out of community area"),
    PLACE_CUBE_AND_JUST_BALANCE(PlaceCubeAndJustBalanceCommand::new, "Place cube at top rung and balance on charge station"),*/
    NONE(ds -> new InstantCommand(), "Do nothing"),
    TEST_PATH_PLANNER(ds -> new PathPlannerAuto("Test Auto"), "PathPlanner test")
    ;
    private final SimpleCommandBuilder builder;
    public final String description;

    AutonomousMode(SimpleCommandBuilder builder, String description) {
        //this((ds, as, cs) -> builder.create(ds), description);
        this.builder = builder;
        this.description = description;
    }

    /*AutonomousMode(CommandBuilder builder, String description) {
        this.builder = builder;
        this.description = description;
    }*/

    public Command getCommand(DrivetrainSubsystem drivetrainSubsystem/*, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem*/) {
        return this.builder.create(drivetrainSubsystem/*, armSubsystem, clawSubsystem*/);
    }

    @FunctionalInterface
    interface SimpleCommandBuilder {
        public Command create(DrivetrainSubsystem drivetrainSubsystem);
    }

    /*@FunctionalInterface
    interface CommandBuilder {
        public Command create(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem);
    }*/
}
