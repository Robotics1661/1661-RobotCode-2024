package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SensorConverter;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX elbowMotor;
    private final TalonFX shoulderMotor;

    public ArmSubsystem() {
        elbowMotor = new TalonFX(ELBOW_MOTOR);
        shoulderMotor = new TalonFX(SHOULDER_MOTOR);
    }

    public void dashboardInfo() {
        SmartDashboard.putNumber("elbowPos", elbowMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("shoulderPos", shoulderMotor.getSelectedSensorPosition());
        if (SmartDashboard.getBoolean("zeroDashboard", false))
            zeroMotorSensors();
        SmartDashboard.putBoolean("zeroDashboard", false);
        postArmAngles(
            shoulderMotor.getSelectedSensorPosition(),
            elbowMotor.getSelectedSensorPosition()
        );
    }

    public void zeroMotorSensors() {
        elbowMotor.setSelectedSensorPosition(0); //max: 186977.000000
        shoulderMotor.setSelectedSensorPosition(0);
    }

    private static final double approachSpeed = 0.3;

    /*
     * USE WITH CAUTION. This will approach a different real-world position
     * dependent on real-world mechanical things with the shoulder position
     * 
     * returns true when target reached
     */
    public boolean elbowApproachDegrees(double target) {
        return elbowApproachDegrees(target, 0.5);
    }

    /*
     * USE WITH CAUTION. This will approach a different real-world position
     * dependent on real-world mechanical things with the shoulder position
     * 
     * returns true when target reached, freedom is # of degrees error that is acceptable
     */
    public boolean elbowApproachDegrees(double target, double freedom) {
        double elbowDegrees = SensorConverter.getElbowDegrees(elbowMotor.getSelectedSensorPosition());
        double difference = target - elbowDegrees;
        double speed = 0;
        boolean reached = true;
        if (Math.abs(difference) > freedom) {
            speed = Math.copySign(approachSpeed, difference);
            reached = false;
        }
        moveElbow(speed);
        return reached;
    }

    /*
     * Returns true when target reached
     */
    public boolean shoulderApproachDegrees(double target) {
        return shoulderApproachDegrees(target, 0.5);
    }

    /*
     * Returns true when target reached, freedom is # of degrees error that is acceptable
     */
    public boolean shoulderApproachDegrees(double target, double freedom) {
        return shoulderApproachDegrees(target, freedom, false);
    }

    /*
     * Returns true when target reached, freedom is # of degrees error that is acceptable
     */
    public boolean shoulderApproachDegrees(double target, double freedom, boolean doEscape) {
        double shoulderDegrees = SensorConverter.getShoulderDegrees(shoulderMotor.getSelectedSensorPosition());
        double difference = target - shoulderDegrees;
        double speed = 0;
        boolean reached = true;
        SmartDashboard.putNumber("shoudlerTarget", target);
        SmartDashboard.putNumber("shoulderDegreesRealReal", shoulderDegrees);
        if (Math.abs(difference) > freedom) {
            SmartDashboard.putNumber("shoulderDifference", difference);
            speed = -Math.copySign(approachSpeed, difference);
            reached = false;
        } else {
            SmartDashboard.putString("shoulderDifference", "not an issue");
        }
        moveShoulder(speed, doEscape);
        return reached;
    }

    public void moveElbow(double speed) {
        speed = boundSpeed(speed);
        elbowMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    private boolean gettingToSafety = false;

    public void moveShoulder(double speed, boolean doEscape) {
        int hardLimit = 50; // must always be > 30
        int escapeZone = 5; // and extra 10 -> won't control smaller than 40
        //will escape to 40 if it gets to 30
        double shoulderDegrees = SensorConverter.getShoulderDegrees(shoulderMotor.getSelectedSensorPosition());
        int softLimit = hardLimit + escapeZone;

        
        
          int backHardLimit = 115;
          int backEscapeZone = 5;
          int backSoftLimit = backHardLimit - backEscapeZone;
          
         

        if (shoulderDegrees < softLimit && doEscape) {
            speed = 0;
        }
        if (doEscape) {
            boolean currentlySafeSafe = shoulderDegrees > softLimit+10;
            boolean currentlyEndanger = shoulderDegrees <= softLimit+10;
            boolean mustEscape = false;
            if (gettingToSafety) {
                if (currentlySafeSafe) {
                    gettingToSafety = false;
                } else {
                    mustEscape = true;
                }
            } else if (currentlyEndanger) {
                gettingToSafety = true;
                mustEscape = true;
            }
            if (mustEscape) {
                speed = -0.15;
            }
            SmartDashboard.putBoolean("currentlySafeSafe", currentlySafeSafe);
            SmartDashboard.putBoolean("currentlyEndanger", currentlyEndanger);
            SmartDashboard.putBoolean("mustEscape", mustEscape);
        }
            
         // this will be the code for the shoulder not going to far back and 
              //damaging the metal
            if(shoulderDegrees > backSoftLimit && doEscape){
                speed = 0;
            }
            if(doEscape){
                boolean currentlyBackSafe = shoulderDegrees < backSoftLimit - 10;
                boolean currentlyBackDanger = shoulderDegrees >= backSoftLimit - 10;
                boolean mustEscape = false;
                if (gettingToSafety) {
                    if (currentlyBackSafe) {
                        gettingToSafety = false;
                    } else {
                        mustEscape = true;
                    }
                } else if(currentlyBackDanger){
                    gettingToSafety = true;
                    mustEscape = true;
                }
                if(mustEscape){
                    speed = +0.15;
                }
            }
            //SmartDashboard.putBoolean("currentlyBackSafe", currentlyBackSafe);
            //SmartDashboard.putBoolean("currentlyBackDanger", );
            

        speed = boundSpeed(speed);
        shoulderMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    private double boundSpeed(double speed) {
        return Math.min(Math.max(-1, speed), 1);
    }

    private void postArmAngles(double shoulderSensorValue, double elbowSensorValue) {
        double shoulderDegrees = SensorConverter.getShoulderDegrees(shoulderSensorValue);
        double elbowDegreesRaw = SensorConverter.getElbowDegrees(elbowSensorValue);

//        double elbowDegrees = elbowDegreesRaw + shoulderDegrees; // no simple code solution to find real elbow degrees. Sad.

        SmartDashboard.putNumber("shoulderDegrees", shoulderDegrees);
        SmartDashboard.putNumber("elbowDegreesRaw", elbowDegreesRaw);
//        SmartDashboard.putNumber("elbowDegrees", elbowDegrees);
    }

    public void stopMovement() {
        moveElbow(0);
        moveShoulder(0, false);
    }

    /*private static int sensor_90 = 29467;
    private static int sensor_135 = 123362;
    private static int _45_degrees_as_sens = sensor_135 - sensor_90;

    private static double toSensorUnits(double degrees) {
        return degrees * _45_degrees_as_sens / 45.0d;
    }

    private static double toDegrees(double sensorUnits) {
        return sensorUnits * 45.0d / _45_degrees_as_sens;
    }

    private static double realDegreesFromLeft(double reportedSensorUnits) {
//        double sensor_90_as_degrees = toDegrees(sensor_90);
//        double reported_as_degrees = toDegrees(reportedSensorUnits);
//        double real_deg = reported_as_degrees - sensor_90_as_degrees + 90;
//        return real_deg;
        return toDegrees(reportedSensorUnits - sensor_90) + 90;
    }*/
}
