package frc.robot.subsystems;

// TODO: implement & use

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.limelight.LimelightHelpers.PoseEstimate;

public class VisionSubsystem {
    public final String m_name;
    private PoseEstimate estimate;

    public VisionSubsystem(String name) {
        this.m_name = name;
    }

    public VisionSubsystem() {
        this("");
    }

    public boolean areAnyTargetsValid() {
        return estimate.tagCount > 0;
    }

    public int getNumberOfTargetsVisible() {
        return estimate.tagCount;
    }

    public double getBestTargetArea() {
        return estimate.avgTagArea;
    }

    public PoseLatency getPoseLatency() {
        estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        return new PoseLatency(estimate.timestampSeconds, estimate.pose);
    }

    public static record PoseLatency(double timestampSeconds, Pose2d pose2d) {}
}
