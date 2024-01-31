package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.GamePieceMode;

/*
 * MUST start robot backwards (eg arm faces drive station)
 */
public class PlaceCubeAndTaxiLongCommand extends CommandBase {
    private double m_startTime;
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private ArmSubsystem m_armSubsystem;
    private ClawSubsystem m_clawSubsystem;
    private boolean finished = false;
    private ExecutionTarget execTarget = ExecutionTarget.FIRST;

    public PlaceCubeAndTaxiLongCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        this.m_startTime = Timer.getFPGATimestamp(); //just as a failsafe
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_armSubsystem = armSubsystem;
        this.m_clawSubsystem = clawSubsystem;

        addRequirements(this.m_drivetrainSubsystem);
        addRequirements(this.m_armSubsystem);
        addRequirements(this.m_clawSubsystem);
    }

    /*
     * Get time this command has been running, in seconds
     */
    private double getRunningTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_drivetrainSubsystem.calibrateGyro180();
        execTarget = ExecutionTarget.FIRST;
        m_clawSubsystem.grabCube();
        initializedOffset = false;
        timeOffset1 = 0;
        if (Constants.GAME_PIECE_MODE == GamePieceMode.CONE)
            throw new RuntimeException("Values not set, will break"); // FIXME remove this once the GamePieceMode values are set
    }

    private double getWrappedGyroDegrees() {
        double degrees = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        while (degrees > 360)
            degrees -= 360;
        while (degrees < 0)
            degrees += 360;
        if (degrees > 180) {
            degrees = degrees - 360; //10 and 350 turn to 10 and -10 respectively
            // 180 and 181 turn to 180 and -179 respectively
        }
        return degrees;
    }

    private void doMovement(double timeOffset) {
        if (false) { // just if you need to disable this
            finished = true;
            return;
        }
        if (isFinished()) return;
        double time = getRunningTime() - timeOffset;
        SmartDashboard.putNumber("shoulderDifference", time);
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if (time < 5.5) { //Drive out of community area
            SmartDashboard.putString("autoTargets", "moving out of community area");
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            .8,
                            0,
                            0,
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
                    
            );
        } else {
            double wrappedDegrees = getWrappedGyroDegrees();
            boolean disableRotation = false;
            if (Math.abs(wrappedDegrees) < 4 || disableRotation) {
                SmartDashboard.putString("autoTargets", "I stopped running ("+wrappedDegrees+")");
                m_drivetrainSubsystem.stop();
                finished = true; // command can stop running now
            } else {
                SmartDashboard.putString("autoTargets", "Spinny spinny spin ("+wrappedDegrees+")");
                m_drivetrainSubsystem.drive(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                0,
                                0,
                                2,
                                m_drivetrainSubsystem.getGyroscopeRotation()
                        )
                );
            }
        }
    }

    private static enum ExecutionTarget {
        EXTEND_ELBOW_PRELIM, // extend arm to starting elbow target, so we don't move it through solid objects later
        EXTEND_SHOULDER, // extend arm to shoulder target
        EXTEND_ELBOW,    // extend arm to elbow target, continue holding shoulder at target
        ALIGN,           // push slightly against driver station to align, keep holding all of arm at target
        DROP,            // release cube
        STOW_SHOULDER,   // move shoulder to stow position
        STOW_ELBOW,      // move elbow to stow position
        BALANCE          // turn over control to balance code
        ;
        private static final ExecutionTarget FIRST = EXTEND_ELBOW_PRELIM;
        public ExecutionTarget next() {
            return values()[Math.min(ordinal() + 1, values().length-1)];
        }
    }

    String tmp = "";

    /* Advance to next execution target */
    private void next() {
        execTarget = execTarget.next();
        sectionReadyForStart = true;
        tmp += ", "+execTarget.name();
    }

    private double sectionTimeOffset = 0.0;
    private boolean sectionReadyForStart = false;
    /* returns true first time it is called after a call to next() */
    private boolean sectionStart() {
        if (sectionReadyForStart) {
            sectionTimeOffset = getRunningTime();
            sectionReadyForStart = false;
            return true;
        }
        return false;
    }

    private double getSectionTime() {
        return getRunningTime() - sectionTimeOffset;
    }

    private boolean initializedOffset = false;
    private double timeOffset1 = 0;

    @Override
    public void execute() {
        if (isFinished()) return;
        double elbowPrelimTarget = Constants.GAME_PIECE_MODE.elbowPrelimDegrees;
        double shoulderExtensionTarget = Constants.GAME_PIECE_MODE.shoulderTargetDegrees;
        double elbowExtensionTarget = Constants.GAME_PIECE_MODE.elbowTargetDegrees;
        double shoulderStowTarget = 103;      // degrees resting position
        double elbowStowTarget = -62;         // degrees resting position
        //SmartDashboard.putString("autoTarget", execTarget.name());
        //SmartDashboard.putString("autoTargets", tmp);
        switch (execTarget) {
            case EXTEND_ELBOW_PRELIM:
                if (m_armSubsystem.elbowApproachDegrees(elbowPrelimTarget)) {
                    m_armSubsystem.stopMovement();
                    next();
                }
                break;
            case EXTEND_SHOULDER:
                if (m_armSubsystem.shoulderApproachDegrees(shoulderExtensionTarget))
                    next();
                break;
            case EXTEND_ELBOW:
                m_armSubsystem.shoulderApproachDegrees(shoulderExtensionTarget);
                if (m_armSubsystem.elbowApproachDegrees(elbowExtensionTarget))
                    next();
                break;
            case ALIGN:
                sectionStart(); // yes, this is safe
                m_armSubsystem.shoulderApproachDegrees(shoulderExtensionTarget);
                m_armSubsystem.elbowApproachDegrees(elbowExtensionTarget);
                if (getSectionTime() < 0.1) {
                    m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        -.1,
                        0,
                        0,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                    ));
                } else {
                    m_drivetrainSubsystem.stop();
                    next();
                }
                break;
            case DROP:
                m_armSubsystem.stopMovement();
                if (sectionStart())
                    m_clawSubsystem.release();
                if (getSectionTime() > 0.5)
                    next();
                break;
            case STOW_SHOULDER:
                if (m_armSubsystem.shoulderApproachDegrees(shoulderStowTarget, 1, false))
                    next();
                break;
            case STOW_ELBOW:
                m_armSubsystem.shoulderApproachDegrees(shoulderStowTarget, 1, false);
                if (m_armSubsystem.elbowApproachDegrees(elbowStowTarget, 1)) {
                    m_armSubsystem.stopMovement();
                    next();
                }
                break;
            case BALANCE:
                if (!initializedOffset) {
                    initializedOffset = true;
                    timeOffset1 = getRunningTime();
                    SmartDashboard.putString("autoTarget", "RT: "+timeOffset1);
                }
                // time offset system so forward and balance timing doesn't get
                // affected by gamepiece placement
                doMovement(timeOffset1);
                break;
            default:
                finished = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.moveElbow(0);
        m_armSubsystem.moveShoulder(0, false);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
/*
 Stages of command-based code:
 init (called 'initialize')
 periodic code - gets called in a loop (callled 'execute')
 cleanup (called 'end')

 extra functions to implement:
 public boolean isFinished() - signals to the CommandScheduler whether the command is done running
 */
}
