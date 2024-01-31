package frc.robot.util;

// TODO: do we actually need this?
public class SensorConverter {
    /*private double sensor_small = 29467;
    private double degrees_small = 90;
    private double sensor_big = 123362;
    private double degrees_big = 135;
    private double difference_sens = sensor_big - sensor_small;
    private double difference_degrees = degrees_big - degrees_small;*/
    private double sensor_small;
    private double degrees_small;
    private double sensor_big;
    private double degrees_big;
    private double difference_sens;
    private double difference_degrees;
    private boolean sensorIsBackwards;
    private static final SensorConverter SHOULDER_CONVERTER = makeShoulder();
    private static final SensorConverter ELBOW_CONVERTER = makeElbow();

    public static double getShoulderDegrees(double reportedSensorUnits) {
        return SHOULDER_CONVERTER.toRealDegrees(reportedSensorUnits);
    }

    public static double getElbowDegrees(double reportedSensorUnits) {
        return ELBOW_CONVERTER.toRealDegrees(reportedSensorUnits);
    }

    public static SensorConverter makeShoulder() {
        return new SensorConverter(29467, 90,
        123362, 135, true);
    }

    public static SensorConverter makeElbow() {
        return new SensorConverter(63832, 0,
        156386, 90, false);
    }

    public SensorConverter(double sensor_small, double degrees_small,
                           double sensor_big, double degrees_big,
                           boolean sensorIsBackwards) {
        this.sensor_small = sensor_small;
        this.degrees_small = degrees_small;
        
        this.sensor_big = sensor_big;
        this.degrees_big = degrees_big;

        this.sensorIsBackwards = sensorIsBackwards;

        this.difference_sens = this.sensor_big - this.sensor_small;
        this.difference_degrees = this.degrees_big - this.degrees_small;
    }

    @SuppressWarnings("unused")
    private double toSensorUnits(double degrees) {
        return degrees * difference_sens / difference_degrees;
    }

    private double toDegrees(double sensorUnits) {
        return sensorUnits * difference_degrees / difference_sens;
    }

    private double realDegreesFromLeft(double reportedSensorUnits) {
//        double sensor_90_as_degrees = toDegrees(sensor_90);
//        double reported_as_degrees = toDegrees(reportedSensorUnits);
//        double real_deg = reported_as_degrees - sensor_90_as_degrees + 90;
//        return real_deg;
        return toDegrees(reportedSensorUnits - sensor_small) + degrees_small;
    }

    public double toRealDegrees(double reportedSensorUnits) {
        if (sensorIsBackwards) {
            return 180 - realDegreesFromLeft(reportedSensorUnits);
        } else {
            return realDegreesFromLeft(reportedSensorUnits);
        }
    }
}