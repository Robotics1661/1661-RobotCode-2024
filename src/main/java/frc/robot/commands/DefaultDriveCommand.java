package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_balanceSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier balanceSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_balanceSupplier = balanceSupplier;

        addRequirements(drivetrainSubsystem);
    }

    /*
     * -1 to 1 angle to -1 to 1 acceleration
     */
    //desmos function: y=s\sin^{-1}\left(\frac{x}{a}\right)x^{b}
    protected double balanceCurve(double x) {
        //config parameters
        double s = 0.63;
        double a = 1;
        double b = 2;
        x = Math.max(0, Math.min(x, 1));
        return Math.max(0, Math.min(s * (Math.sinh(x/a)) * Math.pow(x, b), 1));
    }

    protected void doBalance() {
        double chargePitch = m_drivetrainSubsystem.getChargeStationPitch();


//        if (true) //FIXME temporary
//            return;
        
        double targetSpeed = 0;
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                targetSpeed,
                0,
                0,
                m_drivetrainSubsystem.getGyroscopeRotation())
        );
    }

    @Override
    public void execute() {
        if (m_balanceSupplier.getAsBoolean()) {
            doBalance();
            return;
        }
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
        /*m_drivetrainSubsystem.drive(new ChassisSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble()
        ));*/
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
