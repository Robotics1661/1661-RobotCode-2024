package frc.robot.subsystems;

import static frc.robot.Constants.FOUR_BAR_ENCODER_LEFT_ID;
import static frc.robot.Constants.FOUR_BAR_ENCODER_RIGHT_ID;
import static frc.robot.Constants.FOUR_BAR_LEFT_ID;
import static frc.robot.Constants.FOUR_BAR_RIGHT_ID;
import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    /** Right motor is the leader */
    private final TalonFX m_motorRight;
    /** Left motor follows the right motor */
    private final TalonFX m_motorLeft;

    @SuppressWarnings("unused")
    private final CANcoder m_encoderRight;
    @SuppressWarnings("unused")
    private final CANcoder m_encoderLeft;

    private final DutyCycleOut m_rightDutyCycleRequest;
    private final DutyCycleOut m_leftDutyCycleRequest;
    private final StaticBrake m_brake = new StaticBrake();
    private final PositionVoltage m_rightPositionVoltageRequest;
    private final PositionVoltage m_leftPositionVoltageRequest;

    // WARN: Follower control mode breaks position value - DO NOT USE

    private boolean wasFollowingSetPoint = false;
    private double lastRightTarget = SetPoints.ORIGIN.rightPosition();

    private static final double MAX_VOLTAGE = 2.0;
    private static final double GEAR_RATIO = 95; // TO-DO exact

    public FourBarSubsystem() {
        // TODO: use encoders for homing
        m_rightDutyCycleRequest = new DutyCycleOut(0);
        m_leftDutyCycleRequest = new DutyCycleOut(0);
        m_motorRight = new TalonFX(FOUR_BAR_RIGHT_ID);
        m_motorLeft = new TalonFX(FOUR_BAR_LEFT_ID);
        m_encoderRight = new CANcoder(FOUR_BAR_ENCODER_RIGHT_ID);
        m_encoderLeft = new CANcoder(FOUR_BAR_ENCODER_LEFT_ID);

        // Setup PID and max voltage for motors
        final VoltageConfigs voltageConf = new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(-MAX_VOLTAGE);
        final Slot0Configs slot0Conf = new Slot0Configs() // normal configs
            .withKP(0.1)
        ;
        final Slot1Configs slot1Conf = new Slot1Configs() // homing configs
            .withKP(9.5)
        ;
        final MotorOutputConfigs outputConfig = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs rightFeedbackConf = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(FOUR_BAR_ENCODER_RIGHT_ID)
            .withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        ;
        FeedbackConfigs leftFeedbackConf = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(FOUR_BAR_ENCODER_LEFT_ID)
            .withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        ;

        TalonFXConfigurator configRight = m_motorRight.getConfigurator();
        configRight.apply(voltageConf);
        configRight.apply(slot0Conf);
        configRight.apply(outputConfig);
        configRight.apply(rightFeedbackConf);

        TalonFXConfigurator configLeft = m_motorLeft.getConfigurator();
        configLeft.apply(voltageConf);
        configLeft.apply(slot0Conf);
        configLeft.apply(outputConfig);
        configLeft.apply(leftFeedbackConf);

        // setup following behavior
        // SEE NOTE ABOVE m_follower = new Follower(m_motorRight.getDeviceID(), true);
        m_rightPositionVoltageRequest = new PositionVoltage(SetPoints.ORIGIN.rightPosition(), 0.0, false, 0.0, 0, true, false, false);
        m_leftPositionVoltageRequest = new PositionVoltage(SetPoints.ORIGIN.leftPosition(), 0.0, false, 0.0, 0, true, false, false);
        
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
        setTargetPointInternal(target.leftPosition(), target.rightPosition());
        SmartDashboard.putString("FourBar Mode", "SetPoint: "+target.name());
    }

    public void moveTargetForward() {
        setTargetPointInternal(lastRightTarget + 0.01);
        SmartDashboard.putString("FourBar Mode", "Moving to: "+lastRightTarget);
    }

    public void moveTargetBackward() {
        setTargetPointInternal(lastRightTarget - 0.01);
        SmartDashboard.putString("FourBar Mode", "Moving to: "+lastRightTarget);
    }

    /**
     * 
     * @deprecated for internal - and CAREFUL - use only. Using setTargetPoint is safer because it only allows tested-to-be-safe points
     */
    @Deprecated
    private void setTargetPointInternal(double rightTarget) {
        setTargetPointInternal(rightEncoderPositionToLeftEncoderPosition(rightTarget), rightTarget);
    }

    /**
     * 
     * @deprecated for internal - and CAREFUL - use only. Using setTargetPoint is safer because it only allows tested-to-be-safe points
     */
    @Deprecated
    private void setTargetPointInternal(double leftTarget, double rightTarget) {
        if (!wasFollowingSetPoint) {
            wasFollowingSetPoint = true;
            m_motorLeft.setControl(m_brake);
            m_motorRight.setControl(m_brake);
        }
        lastRightTarget = rightTarget;
        m_motorLeft.setControl(m_leftPositionVoltageRequest.withPosition(leftTarget));
        m_motorRight.setControl(m_rightPositionVoltageRequest.withPosition(rightTarget));
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

    private static double rightEncoderPositionToLeftEncoderPosition(double rightPosition) {
        // apply transform
        // equivalent positions: <--- these are a bit skewed
        // right position: 0.202148
        // left position: 0.312256

        // other equivalent positions:
        // right position: -0.333740
        // left position: -0.165283

        final double rightCal = -0.333740;
        final double leftCal = -0.165283;

        double position = rightPosition - rightCal + leftCal;
        return position;
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
        //TEST_FORWARD(-38.2353515625),
        //TEST_BACKWARD(-94.96337890625),
        //AMP(12.32373046875)
        ORIGIN(-0.112305), // left equiv: -0.388916
        ;
        private final double rightPosition;
        private SetPoints(double rightPosition) {
            this.rightPosition = rightPosition;
        }

        public double rightPosition() {
            return rightPosition;
        }

        public double leftPosition() {
            return rightEncoderPositionToLeftEncoderPosition(rightPosition);
        }
    }
}
