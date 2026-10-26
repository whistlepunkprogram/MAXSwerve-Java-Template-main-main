package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;

/**
 * Vision subsystem for a Limelight 4 camera.
 *
 * <p>Reads common targeting values and the botpose from NetworkTables.
 */
public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;

  public VisionSubsystem() {
    m_limelightTable =
        NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.kLimelightTableName);
  }

  /** Returns true if the Limelight reports a valid target. */
  public boolean hasTarget() {
    return m_limelightTable.getEntry("tv").getDouble(0.0) >= 1.0;
  }

  /** Horizontal offset in degrees from crosshair to target. */
  public double getTx() {
    return m_limelightTable.getEntry("tx").getDouble(0.0);
  }

  /** Vertical offset in degrees from crosshair to target. */
  public double getTy() {
    return m_limelightTable.getEntry("ty").getDouble(0.0);
  }

  /** Target area (percentage of image). */
  public double getTa() {
    return m_limelightTable.getEntry("ta").getDouble(0.0);
  }

  /** Active Limelight target AprilTag ID, or -1 when unavailable. */
  public int getTargetId() {
    return (int) m_limelightTable.getEntry("tid").getDouble(-1.0);
  }

  /** True when Limelight has a valid target and that target is one of the configured hub tags. */
  public boolean hasHubTarget() {
    if (!hasTarget()) {
      return false;
    }

    int targetId = getTargetId();
    for (int hubTagId : Constants.VisionConstants.kHubAprilTagIds) {
      if (targetId == hubTagId) {
        return true;
      }
    }

    return false;
  }

  /** Pipeline latency in milliseconds. */
  public double getPipelineLatencyMs() {
    return m_limelightTable.getEntry("tl").getDouble(0.0);
  }

  /** Capture latency in milliseconds. */
  public double getCaptureLatencyMs() {
    return m_limelightTable.getEntry("cl").getDouble(0.0);
  }

  /** Total vision latency in milliseconds. */
  public double getTotalLatencyMs() {
    return getPipelineLatencyMs() + getCaptureLatencyMs();
  }

  /**
   * Returns the robot pose in WPILib blue-alliance field coordinates from Limelight.
   *
   * <p>Uses the `botpose_wpiblue` entry where index 0=x (m), 1=y (m), and 5=yaw (deg).
   */
  public Optional<Pose2d> getBotPoseBlue() {
    double[] poseArray = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    if (poseArray.length < 6) {
      return Optional.empty();
    }

    double xMeters = poseArray[0];
    double yMeters = poseArray[1];
    double yawDegrees = poseArray[5];
    return Optional.of(new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(yawDegrees)));
  }

  /**
   * Timestamp (FPGA time, seconds) corresponding to the latest Limelight measurement.
   */
  public double getMeasurementTimestampSeconds() {
    return Timer.getFPGATimestamp() - (getTotalLatencyMs() / 1000.0);
  }

  /** Set active Limelight pipeline. */
  public void setPipeline(int pipelineIndex) {
    m_limelightTable.getEntry("pipeline").setNumber(pipelineIndex);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    SmartDashboard.putNumber("Vision/tx", getTx());
    SmartDashboard.putNumber("Vision/ty", getTy());
    SmartDashboard.putNumber("Vision/ta", getTa());
  SmartDashboard.putNumber("Vision/TargetId", getTargetId());
  SmartDashboard.putBoolean("Vision/HasHubTarget", hasHubTarget());
    SmartDashboard.putNumber("Vision/TotalLatencyMs", getTotalLatencyMs());

    getBotPoseBlue()
        .ifPresent(
            pose -> {
              SmartDashboard.putNumber("Vision/PoseX", pose.getX());
              SmartDashboard.putNumber("Vision/PoseY", pose.getY());
              SmartDashboard.putNumber("Vision/PoseHeadingDeg", pose.getRotation().getDegrees());
            });
  }
}
