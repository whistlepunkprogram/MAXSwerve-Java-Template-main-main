package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight4Subsystem extends SubsystemBase {
  private static final String DEFAULT_TABLE_NAME = "limelight";

  private final NetworkTable limelightTable;
  private final NetworkTableEntry tv;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry tid;
  private final NetworkTableEntry botpose;

  public Limelight4Subsystem() {
    this(DEFAULT_TABLE_NAME);
  }

  public Limelight4Subsystem(String tableName) {
    limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
    tv = limelightTable.getEntry("tv");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tid = limelightTable.getEntry("tid");
    botpose = limelightTable.getEntry("botpose_wpiblue");
  }

  public boolean hasTarget() {
    return tv.getDouble(0.0) > 0.5;
  }

  public double getTargetX() {
    return tx.getDouble(0.0);
  }

  public double getTargetY() {
    return ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  public int getAprilTagId() {
    return (int) tid.getDouble(-1.0);
  }

  public Pose2d getBotPoseBlue() {
    double[] pose = botpose.getDoubleArray(new double[6]);
    if (pose.length < 6) {
      return new Pose2d();
    }
  return new Pose2d(
    pose[0],
    pose[1],
    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(pose[5]));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LL4/HasTarget", hasTarget());
    SmartDashboard.putNumber("LL4/Tx", getTargetX());
    SmartDashboard.putNumber("LL4/Ty", getTargetY());
    SmartDashboard.putNumber("LL4/Area", getTargetArea());
    SmartDashboard.putNumber("LL4/TagId", getAprilTagId());
  }
}
