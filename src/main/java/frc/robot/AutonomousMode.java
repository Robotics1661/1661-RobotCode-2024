package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AssistedFarShotCommand;
import frc.robot.commands.autos.SingleSpeakerShotAutoCommand;
import frc.robot.commands.autos.SingleSpeakerShotAutoCommand.StartPoint;
import frc.robot.commands.named.MiscPieces;
import frc.robot.subsystems.AutonomousInput;

/* Just some convenient abstraction for autonomous modes */
public enum AutonomousMode {
    /*BALANCE(AutonomousDriveBackCommand::new, "Drive to charge station"),
    FORWARD_AND_BALANCE(AutonomousDriveBackAndForthCommand::new, "Drive to charge station, keep going, then drive back to charge station"),
    FULL_20_POINT(Full20PointAutoCommand::new, "Place cube at top rung, drive to charge station, keep going, then drive back to charge station"),
    PLACE_CUBE_AND_TAXI_LONG(PlaceCubeAndTaxiLongCommand::new, "Place cube at top rung and taxi out of community area"),
    PLACE_CUBE_AND_JUST_BALANCE(PlaceCubeAndJustBalanceCommand::new, "Place cube at top rung and balance on charge station"),*/
    ////TEST_PATH_PLANNER("Test Auto", "PathPlanner test"),
    ////TEST_PATH_PLANNER_REPLAN("Replanning Auto", "PathPlanner replanning test"),

    // does nothing because this should be chained after an InitFourBarCommand
    INITIALIZE_4BAR((ai) -> new InstantCommand(), "Initialize Four Bar"),

    SINGLE_SPEAKER_SHOT_FRONT(SingleSpeakerShotAutoCommand.withStart(StartPoint.FRONT), "Single speaker shot (from front)"),
    SINGLE_SPEAKER_SHOT_AMP(SingleSpeakerShotAutoCommand.withStart(StartPoint.AMP_SIDE), "Single speaker shot (from amp)"),
    SINGLE_SPEAKER_SHOT_SOURCE(SingleSpeakerShotAutoCommand.withStart(StartPoint.SOURCE_SIDE), "Single speaker shot (from source)"),

    SPEAKER_SHOT_AND_INTAKE_UPPER("Speaker Shot + Intake Upper", "Single speaker shot + intake upper note"),
    SPEAKER_SHOT_AND_INTAKE_MIDDLE("Speaker Shot + Intake Middle", "Single speaker shot + intake middle note"),
    SPEAKER_SHOT_AND_INTAKE_MIDDLE_AND_RACE_FORWARD("Speaker Shot + Intake Middle + Race Forward", "Single speaker shot + intake middle note + race back to shooting position"),
    SPEAKER_SHOT_AND_INTAKE_MIDDLE_AND_SHOOT_FAR("Speaker Shot + Intake Middle + Shoot Far", "Single speaker shot + intake middle note + far speaker shot"),

    FAR_SHOT_TEST(MiscPieces::shootSpeakerFarAuto, "Shoot speaker far test"),
    TEST(AssistedFarShotCommand::create, "test"),

    AMP_SIDE_START_FAR_SHOT("Amp-Start Far Shot", "Start on amp side of speaker, move to middle note and shoot"),
    SOURCE_SIDE_START_FAR_SHOT("Source-Start Far Shot", "Start on source side of speaker, move to middle note and shoot")
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
    public interface CommandBuilder {
        public Command create(AutonomousInput autonomousInput);
    }
}
