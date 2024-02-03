package frc.robot.subsystems;

import static frc.robot.Constants.FOUR_BAR_LEFT_ID;
import static frc.robot.Constants.FOUR_BAR_RIGHT_ID;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class FourBarSubsystem extends SubsystemBase {
    // TODO: set angle points

    /** Right motor is the leader */
    private final TalonFX m_motorRight;
    /** Left motor follows the right motor */
    private final TalonFX m_motorLeft;

    private final DutyCycleOut m_request;

    private static final double MAX_VOLTAGE = 2.0;

    public FourBarSubsystem() {
        m_request = new DutyCycleOut(0);
        m_motorRight = new TalonFX(FOUR_BAR_RIGHT_ID);
        m_motorLeft = new TalonFX(FOUR_BAR_LEFT_ID);

        // Setup PID and max voltage for right motor
        TalonFXConfigurator config = m_motorRight.getConfigurator();
        config.apply(new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(-MAX_VOLTAGE)
        );
        config.apply(new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKA(0.00384)
            .withKD(0.27497)
            .withKP(15.437)
            .withKV(0.11009)
        );

        // setup following behavior
        m_motorLeft.setControl(new Follower(m_motorRight.getDeviceID(), true));

        m_motorRight.setControl(m_request);

        stop();
    }

    public void setSpeed(double percent) {
        percent = clamp(percent, -1, 1);
        m_motorRight.setControl(m_request.withOutput(percent));
    }

    public void stop() {
        setSpeed(0);
        m_motorRight.setControl(new StaticBrake());
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }

    private Measure<Velocity<Angle>> getAnglularVelocity() {
        double rawRotPerSecond = m_motorRight.getVelocity().getValueAsDouble();
        return Units.RotationsPerSecond.of(rawRotPerSecond);
    }

    private Measure<Angle> getAngle() {
        double raw = m_motorRight.getPosition().getValueAsDouble();
        return Units.Radians.of(raw * 2 * Math.PI);
    }

    private void sysIdVelocityDrive(Measure<Voltage> voltage) {
        m_motorRight.setControl(new VoltageOut(voltage.baseUnitMagnitude()));
    }

    private void sysIdLogger(SysIdRoutineLog log) {
        var rightLogger = log.motor("right");
        rightLogger.voltage(Units.Volts.of(m_motorRight.getMotorVoltage().refresh().getValueAsDouble()));
        rightLogger.angularVelocity(getAnglularVelocity());
        rightLogger.angularPosition(getAngle());
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
