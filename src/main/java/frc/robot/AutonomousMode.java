package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autos.SingleSpeakerShotAutoCommand;
import frc.robot.subsystems.AutonomousInput;

/* Just some convenient abstraction for autonomous modes */
public enum AutonomousMode {
    /*BALANCE(AutonomousDriveBackCommand::new, "Drive to charge station"),
    FORWARD_AND_BALANCE(AutonomousDriveBackAndForthCommand::new, "Drive to charge station, keep going, then drive back to charge station"),
    FULL_20_POINT(Full20PointAutoCommand::new, "Place cube at top rung, drive to charge station, keep going, then drive back to charge station"),
    PLACE_CUBE_AND_TAXI_LONG(PlaceCubeAndTaxiLongCommand::new, "Place cube at top rung and taxi out of community area"),
    PLACE_CUBE_AND_JUST_BALANCE(PlaceCubeAndJustBalanceCommand::new, "Place cube at top rung and balance on charge station"),*/
    TEST_PATH_PLANNER("Test Auto", "PathPlanner test"),
    TEST_PATH_PLANNER_REPLAN("Replanning Auto", "PathPlanner replanning test"),
    // does nothing because this should be chained after an InitFourBarCommand
    INITIALIZE_4BAR((ai) -> new InstantCommand(), "Initialize Four Bar"),
    SINGLE_SPEAKER_SHOT(SingleSpeakerShotAutoCommand::create, "Single speaker shot"),
    SPEAKER_SHOT_AND_INTAKE("Speaker Shot + Intake Upper", "Single speaker shot + intake upper note")
    ;
    private final CommandBuilder builder;
    public final String description;

    AutonomousMode(String pathPlannerName, String description) {
        this(ai -> new PathPlannerAuto(pathPlannerName), description);
    }

    AutonomousMode(CommandBuilder builder, String description) {
        //this((ds, as, cs) -> builder.create(ds), description);
        this.builder = builder;
        this.description = description;
    }

    /*AutonomousMode(CommandBuilder builder, String description) {
        this.builder = builder;
        this.description = description;
    }*/

    public Command getCommand(AutonomousInput autonomousInput) {
        return this.builder.create(autonomousInput);
    }

    @FunctionalInterface
    interface CommandBuilder {
        public Command create(AutonomousInput autonomousInput);
    }
}
