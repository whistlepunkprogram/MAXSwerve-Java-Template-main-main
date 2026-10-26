package frc.robot.tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

/**
 * Centralized tuning source for LB auto-aim shot behavior.
 *
 * <p>This class owns:
 * <ul>
 *   <li>all tunable entries (keys/defaults/clamping)</li>
 *   <li>the Shuffleboard Tuning tab layout</li>
 *   <li>live diagnostic entries used by students while tuning</li>
 * </ul>
 */
public final class AutoAimTuning {
  private static boolean s_initialized = false;

  private static GenericEntry s_turnP;
  private static GenericEntry s_maxRot;

  private static GenericEntry s_area0;
  private static GenericEntry s_area1;
  private static GenericEntry s_area2;
  private static GenericEntry s_area3;

  private static GenericEntry s_rpm0;
  private static GenericEntry s_rpm1;
  private static GenericEntry s_rpm2;
  private static GenericEntry s_rpm3;

  private static GenericEntry s_cap0;
  private static GenericEntry s_cap1;
  private static GenericEntry s_cap2;
  private static GenericEntry s_cap3;

  private static GenericEntry s_spinupSec;
  private static GenericEntry s_aimEnterToleranceDeg;
  private static GenericEntry s_aimExitToleranceDeg;
  private static GenericEntry s_rpmEnterTolerance;
  private static GenericEntry s_rpmExitTolerance;
  private static GenericEntry s_readyDebounceSec;
  private static GenericEntry s_allowNoTargetFeed;

  // Live readouts/diagnostics
  private static GenericEntry s_visionTx;
  private static GenericEntry s_visionTa;
  private static GenericEntry s_hasHubTarget;
  private static GenericEntry s_nearFraction;
  private static GenericEntry s_targetRpm;
  private static GenericEntry s_measuredRpm;
  private static GenericEntry s_rpmError;
  private static GenericEntry s_aimReady;
  private static GenericEntry s_rpmReady;
  private static GenericEntry s_feedReadyRaw;
  private static GenericEntry s_feedReady;

  private AutoAimTuning() {}

  /** Creates (once) a student-friendly Shuffleboard tab layout for tuning and live diagnostics. */
  public static synchronized void initializeDashboardLayout() {
    if (s_initialized) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Tuning");

    ShuffleboardLayout aim =
        tab.getLayout("AutoAim - Aim", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5);
    s_turnP = aim.add("Turn P", Constants.VisionConstants.kAutoAimP).getEntry();
    s_maxRot = aim.add("Max Rot", Constants.VisionConstants.kAutoAimMaxRot).getEntry();
    s_aimEnterToleranceDeg =
        aim.add("Aim Enter Tol (deg)", Constants.VisionConstants.kAutoAimAimEnterToleranceDeg).getEntry();
    s_aimExitToleranceDeg =
        aim.add("Aim Exit Tol (deg)", Constants.VisionConstants.kAutoAimAimExitToleranceDeg).getEntry();

    ShuffleboardLayout map =
        tab.getLayout("AutoAim - Shooter Map", BuiltInLayouts.kList)
            .withPosition(2, 0)
            .withSize(3, 8);
    s_area0 = map.add("Area 0", Constants.VisionConstants.kAutoAimAreaMapDefaults[0]).getEntry();
    s_rpm0 = map.add("RPM 0", Constants.VisionConstants.kAutoAimRpmMapDefaults[0]).getEntry();
    s_cap0 = map.add("Cap 0", Constants.VisionConstants.kAutoAimOutputCapMapDefaults[0]).getEntry();
    s_area1 = map.add("Area 1", Constants.VisionConstants.kAutoAimAreaMapDefaults[1]).getEntry();
    s_rpm1 = map.add("RPM 1", Constants.VisionConstants.kAutoAimRpmMapDefaults[1]).getEntry();
    s_cap1 = map.add("Cap 1", Constants.VisionConstants.kAutoAimOutputCapMapDefaults[1]).getEntry();
    s_area2 = map.add("Area 2", Constants.VisionConstants.kAutoAimAreaMapDefaults[2]).getEntry();
    s_rpm2 = map.add("RPM 2", Constants.VisionConstants.kAutoAimRpmMapDefaults[2]).getEntry();
    s_cap2 = map.add("Cap 2", Constants.VisionConstants.kAutoAimOutputCapMapDefaults[2]).getEntry();
    s_area3 = map.add("Area 3", Constants.VisionConstants.kAutoAimAreaMapDefaults[3]).getEntry();
    s_rpm3 = map.add("RPM 3", Constants.VisionConstants.kAutoAimRpmMapDefaults[3]).getEntry();
    s_cap3 = map.add("Cap 3", Constants.VisionConstants.kAutoAimOutputCapMapDefaults[3]).getEntry();

    ShuffleboardLayout feed =
        tab.getLayout("AutoAim - Feed Timing", BuiltInLayouts.kList)
            .withPosition(5, 0)
            .withSize(2, 6);
    s_spinupSec = feed.add("Spinup Sec", 0.8).getEntry();
    s_rpmEnterTolerance =
        feed.add("RPM Enter Tol", Constants.VisionConstants.kAutoAimRpmEnterTolerance).getEntry();
    s_rpmExitTolerance =
        feed.add("RPM Exit Tol", Constants.VisionConstants.kAutoAimRpmExitTolerance).getEntry();
    s_readyDebounceSec =
        feed.add("Ready Debounce Sec", Constants.VisionConstants.kAutoAimReadyDebounceSec).getEntry();
    s_allowNoTargetFeed = feed.add("Allow Feed Without Tag", false).getEntry();

  ShuffleboardLayout vision =
    tab.getLayout("AutoAim - Vision Health", BuiltInLayouts.kList)
      .withPosition(7, 0)
      .withSize(2, 4);
  s_visionTx = vision.add("Vision tx (deg)", 0.0).getEntry();
  s_visionTa = vision.add("Vision ta", 0.0).getEntry();
  s_hasHubTarget = vision.add("Has Hub Target", false).getEntry();

  ShuffleboardLayout status =
        tab.getLayout("AutoAim - Readiness", BuiltInLayouts.kList)
      .withPosition(9, 0)
            .withSize(2, 8);
    s_nearFraction = status.add("Near Fraction", 0.0).getEntry();
    s_targetRpm = status.add("Target RPM", 0.0).getEntry();
    s_measuredRpm = status.add("Measured RPM", 0.0).getEntry();
    s_rpmError = status.add("RPM Error", 0.0).getEntry();
    s_aimReady = status.add("Aim Ready", false).getEntry();
    s_rpmReady = status.add("RPM Ready", false).getEntry();
    s_feedReadyRaw = status.add("Feed Ready Raw", false).getEntry();
    s_feedReady = status.add("Feed Ready", false).getEntry();

  ShuffleboardLayout utilities =
    tab.getLayout("AutoAim - Utilities", BuiltInLayouts.kList)
      .withPosition(11, 0)
      .withSize(2, 3);
  utilities.add("Reset Tunables to Defaults", Commands.runOnce(AutoAimTuning::resetTunablesToDefaults));

    s_initialized = true;
  }

  private static void ensureInitialized() {
    if (!s_initialized) {
      initializeDashboardLayout();
    }
  }

  public static double getTurnP() {
    ensureInitialized();
    return s_turnP.getDouble(Constants.VisionConstants.kAutoAimP);
  }

  public static double getMaxRot() {
    ensureInitialized();
    return Math.abs(s_maxRot.getDouble(Constants.VisionConstants.kAutoAimMaxRot));
  }

  public static double[] getAreaMap() {
    ensureInitialized();
    return new double[] {
      s_area0.getDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[0]),
      s_area1.getDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[1]),
      s_area2.getDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[2]),
      s_area3.getDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[3])
    };
  }

  public static double[] getRpmMap() {
    ensureInitialized();
    return new double[] {
      s_rpm0.getDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[0]),
      s_rpm1.getDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[1]),
      s_rpm2.getDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[2]),
      s_rpm3.getDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[3])
    };
  }

  public static double[] getCapMap() {
    ensureInitialized();
    return new double[] {
      MathUtil.clamp(s_cap0.getDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[0]), 0.0, 1.0),
      MathUtil.clamp(s_cap1.getDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[1]), 0.0, 1.0),
      MathUtil.clamp(s_cap2.getDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[2]), 0.0, 1.0),
      MathUtil.clamp(s_cap3.getDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[3]), 0.0, 1.0)
    };
  }

  public static double getSpinupSeconds() {
    ensureInitialized();
    return Math.max(0.0, s_spinupSec.getDouble(0.8));
  }

  public static double getAimEnterToleranceDeg() {
    ensureInitialized();
    return Math.abs(s_aimEnterToleranceDeg.getDouble(Constants.VisionConstants.kAutoAimAimEnterToleranceDeg));
  }

  public static double getAimExitToleranceDeg() {
    ensureInitialized();
    return Math.abs(s_aimExitToleranceDeg.getDouble(Constants.VisionConstants.kAutoAimAimExitToleranceDeg));
  }

  public static double getRpmEnterTolerance() {
    ensureInitialized();
    return Math.abs(s_rpmEnterTolerance.getDouble(Constants.VisionConstants.kAutoAimRpmEnterTolerance));
  }

  public static double getRpmExitTolerance() {
    ensureInitialized();
    return Math.abs(s_rpmExitTolerance.getDouble(Constants.VisionConstants.kAutoAimRpmExitTolerance));
  }

  public static double getReadyDebounceSec() {
    ensureInitialized();
    return Math.max(0.0, s_readyDebounceSec.getDouble(Constants.VisionConstants.kAutoAimReadyDebounceSec));
  }

  public static boolean allowNoTargetFeed() {
    ensureInitialized();
    return s_allowNoTargetFeed.getBoolean(false);
  }

  public static boolean getHasHubTarget() {
    ensureInitialized();
    return s_hasHubTarget.getBoolean(false);
  }

  public static boolean getFeedReady() {
    ensureInitialized();
    return s_feedReady.getBoolean(false);
  }

  public static void setHasHubTarget(boolean value) {
    ensureInitialized();
    s_hasHubTarget.setBoolean(value);
  }

  public static void setVisionTx(double value) {
    ensureInitialized();
    s_visionTx.setDouble(value);
  }

  public static void setVisionTa(double value) {
    ensureInitialized();
    s_visionTa.setDouble(value);
  }

  public static void setNearFraction(double value) {
    ensureInitialized();
    s_nearFraction.setDouble(value);
  }

  public static void setTargetRpm(double value) {
    ensureInitialized();
    s_targetRpm.setDouble(value);
  }

  public static void setMeasuredRpm(double value) {
    ensureInitialized();
    s_measuredRpm.setDouble(value);
  }

  public static void setRpmError(double value) {
    ensureInitialized();
    s_rpmError.setDouble(value);
  }

  public static void setAimReady(boolean value) {
    ensureInitialized();
    s_aimReady.setBoolean(value);
  }

  public static void setRpmReady(boolean value) {
    ensureInitialized();
    s_rpmReady.setBoolean(value);
  }

  public static void setFeedReadyRaw(boolean value) {
    ensureInitialized();
    s_feedReadyRaw.setBoolean(value);
  }

  public static void setFeedReady(boolean value) {
    ensureInitialized();
    s_feedReady.setBoolean(value);
  }

  /** Restores all auto-aim tunable controls back to their default values. */
  public static void resetTunablesToDefaults() {
    ensureInitialized();

    s_turnP.setDouble(Constants.VisionConstants.kAutoAimP);
    s_maxRot.setDouble(Constants.VisionConstants.kAutoAimMaxRot);

    s_area0.setDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[0]);
    s_area1.setDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[1]);
    s_area2.setDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[2]);
    s_area3.setDouble(Constants.VisionConstants.kAutoAimAreaMapDefaults[3]);

    s_rpm0.setDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[0]);
    s_rpm1.setDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[1]);
    s_rpm2.setDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[2]);
    s_rpm3.setDouble(Constants.VisionConstants.kAutoAimRpmMapDefaults[3]);

    s_cap0.setDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[0]);
    s_cap1.setDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[1]);
    s_cap2.setDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[2]);
    s_cap3.setDouble(Constants.VisionConstants.kAutoAimOutputCapMapDefaults[3]);

    s_spinupSec.setDouble(0.8);
    s_aimEnterToleranceDeg.setDouble(Constants.VisionConstants.kAutoAimAimEnterToleranceDeg);
    s_aimExitToleranceDeg.setDouble(Constants.VisionConstants.kAutoAimAimExitToleranceDeg);
    s_rpmEnterTolerance.setDouble(Constants.VisionConstants.kAutoAimRpmEnterTolerance);
    s_rpmExitTolerance.setDouble(Constants.VisionConstants.kAutoAimRpmExitTolerance);
    s_readyDebounceSec.setDouble(Constants.VisionConstants.kAutoAimReadyDebounceSec);
    s_allowNoTargetFeed.setBoolean(false);

    resetLiveStatus();
  }

  public static void resetLiveStatus() {
    ensureInitialized();
    setHasHubTarget(false);
  setVisionTx(0.0);
  setVisionTa(0.0);
    setNearFraction(0.0);
    setTargetRpm(0.0);
    setMeasuredRpm(0.0);
    setRpmError(0.0);
    setAimReady(false);
    setRpmReady(false);
    setFeedReadyRaw(false);
    setFeedReady(false);
  }
}
