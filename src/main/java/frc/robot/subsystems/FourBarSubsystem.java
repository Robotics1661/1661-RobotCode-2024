package frc.robot.subsystems;

import static frc.robot.Constants.FOUR_BAR_LEFT_ID;
import static frc.robot.Constants.FOUR_BAR_RIGHT_ID;
import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final DutyCycleOut m_rightDutyCycleRequest;
    private final DutyCycleOut m_leftDutyCycleRequest;
    private final StaticBrake m_brake = new StaticBrake();
    private final PositionVoltage m_rightPositionVoltageRequest;
    private final PositionVoltage m_leftPositionVoltageRequest;

    // WARN: Follower control mode breaks position value - DO NOT USE

    private boolean wasFollowingSetPoint = false;

    private static final double MAX_VOLTAGE = 2.0;
    private static final double GEAR_RATIO = 95; // TODO exact

    public FourBarSubsystem() {
        m_rightDutyCycleRequest = new DutyCycleOut(0);
        m_leftDutyCycleRequest = new DutyCycleOut(0);
        m_motorRight = new TalonFX(FOUR_BAR_RIGHT_ID);
        m_motorLeft = new TalonFX(FOUR_BAR_LEFT_ID);

        // Setup PID and max voltage for motors
        final VoltageConfigs voltageConf = new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(-MAX_VOLTAGE);
        final Slot0Configs slot0Conf = new Slot0Configs()
            /*.withKS(0.11787)
            .withKV(0.10395)
            .withKA(0.0048531)*/
            // FOR VELOCITY CONTROL
            ////.withKP(134.47)
            .withKP(0.1)
            /////.withKD(62.22)

            // FOR POSITION CONTROL
            //.withKP(5517)
            //.withKD(62.22)
        ;
        final MotorOutputConfigs outputConfig = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator configRight = m_motorRight.getConfigurator();
        configRight.apply(voltageConf);
        configRight.apply(slot0Conf);
        configRight.apply(outputConfig);

        TalonFXConfigurator configLeft = m_motorLeft.getConfigurator();
        configLeft.apply(voltageConf);
        configLeft.apply(slot0Conf);
        configLeft.apply(outputConfig);

        // setup following behavior
        // SEE NOTE ABOVE m_follower = new Follower(m_motorRight.getDeviceID(), true);
        m_rightPositionVoltageRequest = new PositionVoltage(SetPoints.TEST_FORWARD.rightPosition(), 0.0, false, 0.0, 0, true, false, false);
        m_leftPositionVoltageRequest = new PositionVoltage(SetPoints.TEST_FORWARD.leftPosition(), 0.0, false, 0.0, 0, true, false, false);
        
        //m_motorLeft.setControl(m_follower);
        m_motorLeft.setControl(m_leftDutyCycleRequest);
        m_motorRight.setControl(m_rightDutyCycleRequest);

        stop();
    }

    public void setSpeed(double percent) {
        wasFollowingSetPoint = false;
        percent = clamp(percent, -1, 1) * 0.2;
        if (Math.abs(percent) < 0.001) {
            //System.out.println("Stopping four bar");
            m_motorRight.setControl(m_brake);
            m_motorLeft.setControl(m_brake);
            SmartDashboard.putString("FourBar Mode", "Brake");
        } else {
            m_motorLeft.setControl(m_leftDutyCycleRequest.withOutput(-percent));
            m_motorRight.setControl(m_rightDutyCycleRequest.withOutput(percent));
            SmartDashboard.putString("FourBar Mode", "Percent output: "+percent);
        }
    }

    public void stop() {
        setSpeed(0);
        //m_motorRight.setControl(new StaticBrake());
    }

    public void setTargetPoint(SetPoints target) {
        if (!wasFollowingSetPoint) {
            wasFollowingSetPoint = true;
            m_motorLeft.setControl(m_brake);
            m_motorRight.setControl(m_brake);
        }
        m_motorLeft.setControl(m_leftPositionVoltageRequest.withPosition(target.leftPosition()));
        m_motorRight.setControl(m_rightPositionVoltageRequest.withPosition(target.rightPosition()));
        SmartDashboard.putString("FourBar Mode", "SetPoint: "+target.name());
    }

    /** Get angular velocity of MOTOR */
    private Measure<Velocity<Angle>> getAnglularVelocity() {
        double rawRotPerSecond = m_motorRight.getVelocity().getValueAsDouble();
        return Units.RotationsPerSecond.of(rawRotPerSecond);
    }

    /** Get angle of MOTOR */
    private Measure<Angle> getAngle() {
        double raw = m_motorRight.getPosition().getValueAsDouble();
        return Units.Radians.of(raw * 2 * Math.PI);
    }

    /** Get angular velocity of ARM */
    public Measure<Velocity<Angle>> getArmAngularVelocity() {
        return getAnglularVelocity().divide(GEAR_RATIO);
    }

    /** Get angle of ARM */
    public Measure<Angle> getArmAngle() {
        return getAngle().divide(GEAR_RATIO);
    }

    private void sysIdVelocityDrive(Measure<Voltage> voltage) {
        m_motorRight.setControl(new VoltageOut(voltage.baseUnitMagnitude()));
        m_motorLeft.setControl(new VoltageOut(-voltage.baseUnitMagnitude()));
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
            new SysIdRoutine.Config(
                null, null, Units.Seconds.of(4)
            ),
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

    private static double rightPositionToLeftPosition(double rightPosition) {
        // apply transform and make negative
        double position = rightPosition - 0.2;
        return -position;
    }

    @Override
    public void periodic() {
        super.periodic();

        double rightPos = m_motorRight.getPosition().refresh().getValueAsDouble();
        double leftPos = m_motorLeft.getPosition().refresh().getValueAsDouble();
        SmartDashboard.putNumber("FourBar Right position", rightPos);
        SmartDashboard.putNumber("FourBar Left position", leftPos);
        SmartDashboard.putNumber("FourBar right-left difference", rightPos - (leftPos * -1));
    }

    public static enum SetPoints {
        TEST_FORWARD(-38.2353515625),
        TEST_BACKWARD(-94.96337890625),
        AMP(12.32373046875)
        ;
        private final double rightPosition;
        private SetPoints(double rightPosition) {
            this.rightPosition = rightPosition;
        }

        public double rightPosition() {
            return rightPosition;
        }

        public double leftPosition() {
            return rightPositionToLeftPosition(rightPosition);
        }
    }
}
