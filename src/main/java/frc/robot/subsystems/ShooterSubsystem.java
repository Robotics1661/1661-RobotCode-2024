package frc.robot.subsystems;

import static frc.robot.Constants.SHOOTER_BACK_ID;
import static frc.robot.Constants.SHOOTER_FRONT_ID;
import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase {
    /** Front motor is the leader */
    private final TalonFX m_motorFront;
    /** Back motor follows the front motor */
    private final TalonFX m_motorBack;

    private final DutyCycleOut m_request;

    private static final double MAX_VOLTAGE = 8.0; // we've tried up to 8.0

    public ShooterSubsystem() {
        m_request = new DutyCycleOut(0);
        m_motorFront = new TalonFX(SHOOTER_FRONT_ID);
        m_motorBack = new TalonFX(SHOOTER_BACK_ID);

        // Setup PID and max voltage for front motor
        TalonFXConfigurator config = m_motorFront.getConfigurator();
        config.apply(new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(-MAX_VOLTAGE)
        );
        /*config.apply(new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKA(0.00384)
            .withKD(0.27497)
            .withKP(15.437)
            .withKV(0.11009)
        );*/

        // setup following behavior
        m_motorBack.setControl(new Follower(m_motorFront.getDeviceID(), false));

        m_motorFront.setControl(m_request);

        stop();
    }

    public void setSpeed(double percent) {
        percent = clamp(percent, -1, 1);
        m_motorFront.setControl(m_request.withOutput(percent));
        //System.out.print("SETTING SHOOTER SPEED TO: ");
        //System.out.println(percent);
    }

    public void stop() {
        setSpeed(0);
        m_motorFront.setControl(new StaticBrake());
    }

    /** Get angular velocity of MOTOR */
    private Measure<Velocity<Angle>> getAnglularVelocity() {
        double rawRotPerSecond = m_motorFront.getVelocity().getValueAsDouble();
        return Units.RotationsPerSecond.of(rawRotPerSecond);
    }

    /** Get angle of MOTOR */
    private Measure<Angle> getAngle() {
        double raw = m_motorFront.getPosition().getValueAsDouble();
        return Units.Radians.of(raw * 2 * Math.PI);
    }

    private void sysIdVelocityDrive(Measure<Voltage> voltage) {
        m_motorFront.setControl(new VoltageOut(voltage.baseUnitMagnitude()));
    }

    private void sysIdLogger(SysIdRoutineLog log) {
        var frontLogger = log.motor("front");
        frontLogger.voltage(Units.Volts.of(m_motorFront.getMotorVoltage().refresh().getValueAsDouble()));
        frontLogger.angularVelocity(getAnglularVelocity());
        frontLogger.angularPosition(getAngle());
        //rightLogger.angularPosition(Units.Meters.ofBaseUnits(m_motorRight.getPosition().refresh().getValueAsDouble()));
    }

    private SysIdRoutine createSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::sysIdVelocityDrive, this::sysIdLogger, this)
        );
    }

    /**
     * @deprecated Should only be used for testing
     */
    @Deprecated
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return createSysIdRoutine().quasistatic(direction);
    }

    /**
     * @deprecated Should only be used for testing
     */
    @Deprecated
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return createSysIdRoutine().dynamic(direction);
    }
}
