package frc.robot.subsystems;

import static frc.robot.Constants.FOUR_BAR_ENCODER_LEFT_ID;
import static frc.robot.Constants.FOUR_BAR_ENCODER_RIGHT_ID;
import static frc.robot.Constants.FOUR_BAR_LEFT_ID;
import static frc.robot.Constants.FOUR_BAR_POSITION_HOLD_ASSIST;
import static frc.robot.Constants.FOUR_BAR_RIGHT_ID;
import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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
import frc.robot.util.TimeUtil;

public class FourBarSubsystem extends SubsystemBase {

    /** Right motor is the leader */
    private final TalonFX m_motorRight;
    /** Left motor follows the right motor */
    private final TalonFX m_motorLeft;

    @SuppressWarnings("unused")
    private final CANcoder m_encoderRight;
    @SuppressWarnings("unused")
    private final CANcoder m_encoderLeft;

    //private final Orchestra m_orchestra;

    private final DutyCycleOut m_rightDutyCycleRequest;
    private final DutyCycleOut m_leftDutyCycleRequest;
    private final StaticBrake m_brake = new StaticBrake();
    private final PositionVoltage m_rightPositionVoltageRequest;
    private final PositionVoltage m_leftPositionVoltageRequest;
    private final MotionMagicVoltage m_rightMotionMagicVoltageRequest;
    private final MotionMagicVoltage m_leftMotionMagicVoltageRequest;

    // WARN: Follower control mode breaks position value - DO NOT USE

    private boolean initialized = false;

    private double activeBrakeRightTarget = Double.NaN;
    private boolean wasFollowingSetPoint = false;
    @SuppressWarnings("unused")
    private SetPoints lastSetPoint = SetPoints.ORIGIN;
    private double lastRightTarget = SetPoints.ORIGIN.rightPosition();
    private boolean goFaster = false;

    private static final double MAX_VOLTAGE = 4.0;
    private static final double GEAR_RATIO = 95; // TO-DO exact

    private final double RIGHT_HOMING_TARGET = -0.098876953125;
    private final double LEFT_HOMING_TARGET = rightEncoderPositionToLeftEncoderPosition(RIGHT_HOMING_TARGET);

    private final double FORWARD_RIGHT_MOTOR_LIMIT = 88.17041015625; // TODO: apply SP_OFFSET
    private final double BACKWARD_RIGHT_MOTOR_LIMIT = -77.0;

    // assigning -BR to FR and -FR to BL is intentional, it needs to be swapped due to negative
    private final double FORWARD_LEFT_MOTOR_LIMIT = -BACKWARD_RIGHT_MOTOR_LIMIT;
    private final double BACKWARD_LEFT_MOTOR_LIMIT = -FORWARD_RIGHT_MOTOR_LIMIT;
    private final boolean ENABLE_SOFTWARE_LIMITS = true;

    private final FeedbackConfigs
        rightInitFeedbackConf = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(FOUR_BAR_ENCODER_RIGHT_ID)
            .withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder),
        leftInitFeedbackConf = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(FOUR_BAR_ENCODER_LEFT_ID)
            .withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        ;
    private final SoftwareLimitSwitchConfigs initLimitSwitchConf =
        new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(false)
        .withReverseSoftLimitEnable(false)
        .withForwardSoftLimitThreshold(0.0)
        .withReverseSoftLimitThreshold(0.0)
    ;
    
    private final FeedbackConfigs
        rightNormalFeedbackConf = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(0)
            .withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor),
        leftNormalFeedbackConf = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(0)
            .withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        ;
    private final SoftwareLimitSwitchConfigs
        rightNormalLimitSwitchConf = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(ENABLE_SOFTWARE_LIMITS)
            .withReverseSoftLimitEnable(ENABLE_SOFTWARE_LIMITS)
            .withForwardSoftLimitThreshold(FORWARD_RIGHT_MOTOR_LIMIT)
            .withReverseSoftLimitThreshold(BACKWARD_RIGHT_MOTOR_LIMIT),
        leftNormalLimitSwitchConf = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(ENABLE_SOFTWARE_LIMITS)
            .withReverseSoftLimitEnable(ENABLE_SOFTWARE_LIMITS)
            .withForwardSoftLimitThreshold(FORWARD_LEFT_MOTOR_LIMIT)
            .withReverseSoftLimitThreshold(BACKWARD_LEFT_MOTOR_LIMIT)
    ;

    public FourBarSubsystem() {
        m_rightDutyCycleRequest = new DutyCycleOut(0);
        m_leftDutyCycleRequest = new DutyCycleOut(0);
        m_motorRight = new TalonFX(FOUR_BAR_RIGHT_ID);
        m_motorLeft = new TalonFX(FOUR_BAR_LEFT_ID);
        m_encoderRight = new CANcoder(FOUR_BAR_ENCODER_RIGHT_ID);
        m_encoderLeft = new CANcoder(FOUR_BAR_ENCODER_LEFT_ID);

        /*m_orchestra = new Orchestra();
        m_orchestra.addInstrument(m_motorRight);
        m_orchestra.addInstrument(m_motorLeft);
        m_orchestra.loadMusic("song1.chrp");*/

        // Setup PID and max voltage for motors
        {
            final VoltageConfigs voltageConf = new VoltageConfigs()
                .withPeakForwardVoltage(MAX_VOLTAGE)
                .withPeakReverseVoltage(-MAX_VOLTAGE);
            final Slot0Configs slot0Conf = new Slot0Configs() // normal configs
                .withKP(16.0)
            ;
            final Slot1Configs slot1Conf = new Slot1Configs() // homing configs
                .withKP(55.0) // was 35.0
            ;
            final Slot2Configs slot2Conf = new Slot2Configs() // normal (faster) configs
                .withKP(20.0)
            ;
            final MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
            final MotionMagicConfigs motionMagicConf = new MotionMagicConfigs()
                .withMotionMagicAcceleration(100)
                .withMotionMagicCruiseVelocity(300.0)
            ;


            TalonFXConfigurator configRight = m_motorRight.getConfigurator();
            configRight.apply(voltageConf);
            configRight.apply(slot0Conf);
            configRight.apply(outputConfig);
            configRight.apply(slot1Conf);
            configRight.apply(initLimitSwitchConf);
            configRight.apply(slot2Conf);
            configRight.apply(motionMagicConf);

            TalonFXConfigurator configLeft = m_motorLeft.getConfigurator();
            configLeft.apply(voltageConf);
            configLeft.apply(slot0Conf);
            configLeft.apply(outputConfig);
            configLeft.apply(slot1Conf);
            configLeft.apply(initLimitSwitchConf);
            configLeft.apply(slot2Conf);
            configLeft.apply(motionMagicConf);
        }

        // configure encoders
        {
            System.out.println("[4bar] Configuring CANCoders");
            MagnetSensorConfigs magnetConf = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf); // was signed +- 1/2

            CANcoderConfigurator configRight = m_encoderRight.getConfigurator();
            configRight.apply(magnetConf);

            CANcoderConfigurator configLeft = m_encoderLeft.getConfigurator();
            configLeft.apply(magnetConf);

            /*
            busySleep(500);
            System.out.println("[4bar] Setting CANCoder positions");
            // init can coders to absolute
            m_encoderRight.setPosition(m_encoderRight
                .getAbsolutePosition().refresh().getValueAsDouble());
            m_encoderLeft.setPosition(m_encoderLeft
                .getAbsolutePosition().refresh().getValueAsDouble());
            // */
            System.out.println("[4bar] Done with CANCoder setup");
        }

        // setup following behavior
        // SEE NOTE ABOVE m_follower = new Follower(m_motorRight.getDeviceID(), true);
        m_rightPositionVoltageRequest = new PositionVoltage(SetPoints.ORIGIN.rightPosition(), 0.0, false, 0.0, 0, true, false, false);
        m_leftPositionVoltageRequest = new PositionVoltage(SetPoints.ORIGIN.leftPosition(), 0.0, false, 0.0, 0, true, false, false);
        m_rightMotionMagicVoltageRequest = new MotionMagicVoltage(SetPoints.ORIGIN.rightPosition(), false, 0.0, 0, true, false, false);
        m_leftMotionMagicVoltageRequest = new MotionMagicVoltage(SetPoints.ORIGIN.leftPosition(), false, 0.0, 0, true, false, false);
        
        //m_motorLeft.setControl(m_follower);
        m_motorLeft.setControl(m_leftDutyCycleRequest);
        m_motorRight.setControl(m_rightDutyCycleRequest);

        endOfRoutineStop();

        SmartDashboard.putNumber("FourBar/Right Homing Target", RIGHT_HOMING_TARGET);
        SmartDashboard.putNumber("FourBar/Left Homing Target", LEFT_HOMING_TARGET);
    }

    public void setSpeed(double percent) {
        setSpeed(percent, false);
    }

    public void setSpeed(double percent, boolean forceStaticBrake) {
        wasFollowingSetPoint = false;
        percent = clamp(percent, -1, 1) * 0.2;
        if (Math.abs(percent) < 0.001) {
            //System.out.println("Stopping four bar");
            if (FOUR_BAR_POSITION_HOLD_ASSIST && initialized && !forceStaticBrake) {
                activePositionHold();
                SmartDashboard.putString("FourBar/Mode", "ActiveBrake");
            } else {
                m_motorRight.setControl(m_brake);
                m_motorLeft.setControl(m_brake);
                activeBrakeRightTarget = Double.NaN;
                SmartDashboard.putString("FourBar/Mode", "StaticBrake");
            }
        } else {
            m_motorLeft.setControl(m_leftDutyCycleRequest.withOutput(-percent));
            m_motorRight.setControl(m_rightDutyCycleRequest.withOutput(percent));
            activeBrakeRightTarget = Double.NaN;
            SmartDashboard.putString("FourBar/Mode", "Percent output: "+percent);
        }
    }

    public void activeStop() {
        goFaster = false;
        setSpeed(0);
    }

    public void endOfRoutineStop() {
        goFaster = false;
        setSpeed(0, true);
    }

    public void setTargetPoint(SetPoints target) {
        setTargetPoint(target, false);
    }

    public void setTargetPoint(SetPoints target, boolean faster) {
        goFaster = faster;
        lastSetPoint = target;
        setTargetPointInternal(target.leftPosition(), target.rightPosition(), true);
        SmartDashboard.putString("FourBar/Mode", "SetPoint: "+target.name());
        activeBrakeRightTarget = Double.NaN;
    }

    public void moveTargetForward() {
        setTargetPointInternal(lastRightTarget + 4, false);
        SmartDashboard.putString("FourBar/Mode", "Moving to: "+lastRightTarget);
        activeBrakeRightTarget = Double.NaN;
    }

    public void moveTargetBackward() {
        setTargetPointInternal(lastRightTarget - 4, false);
        SmartDashboard.putString("FourBar/Mode", "Moving to: "+lastRightTarget);
        activeBrakeRightTarget = Double.NaN;
    }

    public void enterInitMode() {
        if (initialized) {
            System.out.println("[4bar] Re-initializing, is this a bug?");
        }
        //System.out.println("[4bar] playing");
        //m_orchestra.play();
        //TimeUtil.busySleep(5000);
        //System.out.println("[4bar] done");
        endOfRoutineStop();
        initialized = false;

        TalonFXConfigurator rightConf = m_motorRight.getConfigurator();
        rightConf.apply(rightInitFeedbackConf);
        rightConf.apply(initLimitSwitchConf);

        TalonFXConfigurator leftConf = m_motorLeft.getConfigurator();
        leftConf.apply(leftInitFeedbackConf);
        leftConf.apply(initLimitSwitchConf);
        
        m_rightPositionVoltageRequest.withSlot(1).withPosition(RIGHT_HOMING_TARGET);
        m_leftPositionVoltageRequest.withSlot(1).withPosition(LEFT_HOMING_TARGET);

        System.out.println("Sleeping");
        TimeUtil.busySleep(50);
        System.out.println("WAKEY WAKEY");
    }

    private double lastWorstError = 0.0;

    private double getManualErrorRight() {
        return Math.abs(m_motorRight.getPosition().getValueAsDouble() - lastRightTarget);
    }

    private double getManualErrorLeft() {
        return Math.abs(m_motorLeft.getPosition().getValueAsDouble() - (-lastRightTarget));
    }

    public boolean isClosedLoopErrorWithin(double buffer) {
        double rightError = Math.abs(m_motorRight.getClosedLoopError().getValueAsDouble());
        double leftError = Math.abs(m_motorLeft.getClosedLoopError().getValueAsDouble());
        double rightErrorM = getManualErrorRight();
        double leftErrorM = getManualErrorLeft();
        double worstError;
        if (initialized) {
            worstError = Math.max(
                Math.max(rightError, leftError),
                Math.max(rightErrorM, leftErrorM)
            );
        } else {
            worstError = Math.max(rightError, leftError);
        }
        if (worstError < buffer && worstError < lastWorstError) {
            lastWorstError = worstError;
            return true;
        }
        lastWorstError = worstError;
        return false;
    }

    /**
     * 
     * @return completion
     */
    public boolean runInitializationProcedure() {
        if (initialized) {
            System.err.println("[4bar] cannot run initialization procedure while already initialized");
            return true;
        }

        m_motorRight.setControl(m_rightPositionVoltageRequest.withSlot(1).withPosition(RIGHT_HOMING_TARGET));
        m_motorLeft.setControl(m_leftPositionVoltageRequest.withSlot(1).withPosition(LEFT_HOMING_TARGET));

        if (isClosedLoopErrorWithin(0.0045)) {
            System.out.println("[4bar] initialization complete!!!");
            return true;
        }

        return false;
    }

    /**
     * exits init mode and zeroes motors
     */
    public void exitInitMode() {
        if (initialized) {
            System.err.println("[4bar] Exit init mode called while already initialized");
            return;
        }
        System.out.println("[4bar] exiting init mode");
        endOfRoutineStop();

        TalonFXConfigurator rightConf = m_motorRight.getConfigurator();
        rightConf.apply(rightNormalFeedbackConf);
        rightConf.apply(rightNormalLimitSwitchConf);

        TalonFXConfigurator leftConf = m_motorLeft.getConfigurator();
        leftConf.apply(leftNormalFeedbackConf);
        leftConf.apply(leftNormalLimitSwitchConf);
        
        m_motorRight.setPosition(0.0);
        m_motorLeft.setPosition(0.0);
        
        m_rightPositionVoltageRequest.withSlot(0).withPosition(SetPoints.ORIGIN.rightPosition());
        m_leftPositionVoltageRequest.withSlot(0).withPosition(SetPoints.ORIGIN.leftPosition());
        initialized = true;
        //m_orchestra.stop();
        System.out.println("[4bar] Zeroed motors to home position!");
    }

    public boolean needsInitialization() {
        return !initialized;
    }

    public void activePositionHold() {
        if (Double.isNaN(activeBrakeRightTarget)) {
            activeBrakeRightTarget = m_motorRight.getPosition().getValueAsDouble();
        }
        setTargetPointInternal(activeBrakeRightTarget, false);
    }

    /**
     * 
     * @deprecated for internal - and CAREFUL - use only. Using setTargetPoint is safer because it only allows tested-to-be-safe points
     */
    @Deprecated
    private void setTargetPointInternal(double rightTarget, boolean useMotionMagic) {
        setTargetPointInternal(-rightTarget, rightTarget, useMotionMagic);
    }

    /**
     * 
     * @deprecated for internal - and CAREFUL - use only. Using setTargetPoint is safer because it only allows tested-to-be-safe points
     */
    @Deprecated
    private void setTargetPointInternal(double leftTarget, double rightTarget, boolean useMotionMagic) {
        if (!initialized) {
            System.err.println("[4bar] Target set without initialization, skipping");
            endOfRoutineStop();
            return;
        }
        if (!wasFollowingSetPoint) {
            wasFollowingSetPoint = true;
            m_motorLeft.setControl(m_brake);
            m_motorRight.setControl(m_brake);
        }
        //boolean slowMode = m_motorRight.getPosition().getValueAsDouble() > 70
        //    && (lastSetPoint != SetPoints.AMP && lastSetPoint != SetPoints.SPEAKER && lastSetPoint != SetPoints.SPEAKER_AUTO && lastSetPoint != SetPoints.FAR_SPEAKER_AUTO);
        //SmartDashboard.putBoolean("FourBar/Slow Mode", slowMode);
        SmartDashboard.putBoolean("FourBar/Faster", goFaster);

        SmartDashboard.putBoolean("FourBar/MotionMagic", useMotionMagic);

        lastRightTarget = rightTarget;

        if (useMotionMagic) {
            m_motorLeft.setControl(m_leftMotionMagicVoltageRequest
                .withPosition(leftTarget).withSlot(goFaster ? 2 : 0));
            m_motorRight.setControl(m_rightMotionMagicVoltageRequest
                .withPosition(rightTarget).withSlot(goFaster ? 2 : 0));
        } else {
            m_motorLeft.setControl(m_leftPositionVoltageRequest
                .withPosition(leftTarget).withSlot(goFaster ? 2 : 0));
            m_motorRight.setControl(m_rightPositionVoltageRequest
                .withPosition(rightTarget).withSlot(goFaster ? 2 : 0));
        }
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
        // apply negated transform (left varies negatively proportional to right)

        boolean n = false;
        final double leftCal = n ? -0.340088 : -0.448242;
        final double rightCal = n ? -0.189697 : -0.064697;

        double normalizedPosition = rightPosition - rightCal;
        double offset = -normalizedPosition;

        double position = offset + leftCal;

        // DO NOT convert to plus/minus 1/2
        /*
        while (position > 0.5) {
            position -= 1.0;
        }
        while (position < -0.5) {
            position += 1.0;
        } // */
        return position;
    }

    @Override
    public void periodic() {
        super.periodic();

        double rightPos = m_encoderRight.getPosition().refresh().getValueAsDouble();
        double leftPos = m_encoderLeft.getPosition().refresh().getValueAsDouble();
        SmartDashboard.putNumber("FourBar/Right position", rightPos); // for graphs
        SmartDashboard.putNumber("FourBar/Left position", leftPos); // for graphs
        SmartDashboard.putNumber("FourBar/Right Position", rightPos); // for copy-paste
        SmartDashboard.putNumber("FourBar/Left Position", leftPos); // for copy-paste
        SmartDashboard.putNumber("FourBar/right-left difference", rightPos - (leftPos * -1));

        double expectedLeft = rightEncoderPositionToLeftEncoderPosition(rightPos);
        double error = expectedLeft - leftPos;
        SmartDashboard.putNumber("FourBar/Left prediction error", error);
        SmartDashboard.putNumber("FourBar/Left prediction", expectedLeft);

        double rightMotorPos = m_motorRight.getPosition().getValueAsDouble();
        double leftMotorPos = m_motorLeft.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("FourBar/Right Motor Encoder", rightMotorPos);
        SmartDashboard.putNumber("FourBar/Left Motor Encoder", leftMotorPos);

        double rightMotorError = m_motorRight.getClosedLoopError().getValueAsDouble();
        double leftMotorError = m_motorLeft.getClosedLoopError().getValueAsDouble();
        double worstError = Math.max(Math.abs(rightMotorError), Math.abs(leftMotorError));
        SmartDashboard.putNumber("FourBar/Closed Loop Error (Worst)", worstError);
        SmartDashboard.putNumber("FourBar/MM Running", m_motorRight.getMotionMagicIsRunning().getValueAsDouble());

        SmartDashboard.putBoolean("FourBar/Initialized", initialized);
    }

    private static double SP_OFFSET = -1.2; // -1.2

    public static enum SetPoints {
        ORIGIN(0.0),
        INTAKE(-69.25 + SP_OFFSET),
        INTAKE_AUTO_HALFWAY(-60.5 + SP_OFFSET),
        AMP(82.74),//81.36),//85.0),
        SPEAKER(85.5 + SP_OFFSET),
        SPEAKER_AUTO(85.5 + SP_OFFSET),
        FAR_SPEAKER_AUTO(79.10)
        ;
        private final double rightPosition;
        private SetPoints(double rightPosition) {
            this.rightPosition = rightPosition;
        }

        public double rightPosition() {
            return rightPosition;
        }

        public double leftPosition() {
            return -rightPosition;
        }
    }
}
