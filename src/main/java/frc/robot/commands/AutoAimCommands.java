package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.JustShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.tuning.AutoAimTuning;
import java.util.function.DoubleSupplier;

/**
 * Reusable auto-aim command factory for hub AprilTag assisted driver shooting.
 *
 * <p>Student note: this class owns the "math + logic" for the left-bumper assisted shot.
 * `RobotContainer` only connects controller inputs to this behavior.
 */
public final class AutoAimCommands {
  private AutoAimCommands() {}

  /**
   * Seeds SmartDashboard with default tuning values so students can tune live without redeploying.
   */
  public static void initializeTuningEntries() {
    AutoAimTuning.initializeDashboardLayout();
  }

  /**
   * Creates the driver LB auto-aim command.
   *
   * <p>Behavior summary while held:
   *
   * <ul>
   *   <li>Drive translation still comes from left stick.
   *   <li>Rotation is auto-corrected toward the hub AprilTag when visible.
   *   <li>Shooter RPM is interpolated between high/low presets based on target area.
   *   <li>Feeder waits for spin-up and readiness checks before feeding.
   * </ul>
   */
  public static Command createDriverHubAutoAimShootCommand(
      DriveSubsystem drive,
      IntakeShooterSubsystem intake,
      JustShooterSubsystem shooter,
      FeederSubsystem feeder,
      VisionSubsystem vision,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier,
      DoubleSupplier rightXSupplier,
      double driveDeadband) {
  // Small mutable holders used by lambdas to preserve state across execute() cycles.
  final double[] startTimeSec = new double[] {0.0};
  final double[] rawReadyStartTimeSec = new double[] {-1.0};
  final boolean[] aimReadyLatched = new boolean[] {false};
  final boolean[] rpmReadyLatched = new boolean[] {false};

    return Commands.runEnd(
            () -> {
              double xSpeed = -MathUtil.applyDeadband(leftYSupplier.getAsDouble(), driveDeadband);
              double ySpeed = -MathUtil.applyDeadband(leftXSupplier.getAsDouble(), driveDeadband);
              double rotSpeed = -MathUtil.applyDeadband(rightXSupplier.getAsDouble(), driveDeadband);

              double aimP = AutoAimTuning.getTurnP();
              double maxRot = AutoAimTuning.getMaxRot();
              double[] areaMap = AutoAimTuning.getAreaMap();
              double[] rpmMap = AutoAimTuning.getRpmMap();
              double[] capMap = AutoAimTuning.getCapMap();
              sortMapByArea(areaMap, rpmMap, capMap);

              double spinupSeconds = AutoAimTuning.getSpinupSeconds();
              double aimEnterToleranceDeg = AutoAimTuning.getAimEnterToleranceDeg();
              double aimExitToleranceDeg = AutoAimTuning.getAimExitToleranceDeg();
              double rpmEnterTolerance = AutoAimTuning.getRpmEnterTolerance();
              double rpmExitTolerance = AutoAimTuning.getRpmExitTolerance();
              double readyDebounceSec = AutoAimTuning.getReadyDebounceSec();
              boolean allowNoTargetFeed = AutoAimTuning.allowNoTargetFeed();

              boolean hasHubTarget = vision.hasHubTarget();
              double tx = vision.getTx();
              double ta = vision.getTa();
              if (hasHubTarget) {
                // P-only turn assist toward center (tx -> 0).
                rotSpeed = MathUtil.clamp(-tx * aimP, -maxRot, maxRot);
              }

              double targetRpm = interpolateByArea(ta, areaMap, rpmMap);
              double targetOutputCap = MathUtil.clamp(interpolateByArea(ta, areaMap, capMap), 0.0, 1.0);
              double nearFraction = normalizeByRange(ta, areaMap[0], areaMap[areaMap.length - 1]);

              if (!hasHubTarget) {
                // Without target lock, fall back to farthest-map values.
                targetRpm = rpmMap[0];
                targetOutputCap = MathUtil.clamp(capMap[0], 0.0, 1.0);
              }

              drive.drive(xSpeed, ySpeed, rotSpeed, true);
              shooter.startOrUpdatePIDControl(targetRpm, targetOutputCap);
              intake.setIntakeShooterSpeed(Constants.ManipulatorConstants.kIntakeShootOutput);

              double measuredRpm = shooter.getMeasuredRPM();
              double rpmError = Math.abs(targetRpm - measuredRpm);
              if (hasHubTarget) {
                if (!aimReadyLatched[0] && Math.abs(tx) <= aimEnterToleranceDeg) {
                  aimReadyLatched[0] = true;
                } else if (aimReadyLatched[0] && Math.abs(tx) > aimExitToleranceDeg) {
                  aimReadyLatched[0] = false;
                }
              } else {
                aimReadyLatched[0] = false;
              }

              if (!rpmReadyLatched[0] && rpmError <= rpmEnterTolerance) {
                rpmReadyLatched[0] = true;
              } else if (rpmReadyLatched[0] && rpmError > rpmExitTolerance) {
                rpmReadyLatched[0] = false;
              }

              boolean aimReady = aimReadyLatched[0];
              boolean rpmReady = rpmReadyLatched[0];
              boolean spinupReady = Timer.getFPGATimestamp() - startTimeSec[0] >= spinupSeconds;
              boolean feedReadyRaw =
                  spinupReady && rpmReady && (aimReady || (allowNoTargetFeed && !hasHubTarget));

              double nowSec = Timer.getFPGATimestamp();
              if (feedReadyRaw) {
                if (rawReadyStartTimeSec[0] < 0.0) {
                  rawReadyStartTimeSec[0] = nowSec;
                }
              } else {
                rawReadyStartTimeSec[0] = -1.0;
              }

              boolean feedReady =
                  feedReadyRaw
                      && rawReadyStartTimeSec[0] >= 0.0
                      && (nowSec - rawReadyStartTimeSec[0]) >= readyDebounceSec;

              AutoAimTuning.setHasHubTarget(hasHubTarget);
              AutoAimTuning.setVisionTx(tx);
              AutoAimTuning.setVisionTa(ta);
              AutoAimTuning.setNearFraction(nearFraction);
              AutoAimTuning.setTargetRpm(targetRpm);
              AutoAimTuning.setMeasuredRpm(measuredRpm);
              AutoAimTuning.setRpmError(rpmError);
              AutoAimTuning.setAimReady(aimReady);
              AutoAimTuning.setRpmReady(rpmReady);
              AutoAimTuning.setFeedReadyRaw(feedReadyRaw);
              AutoAimTuning.setFeedReady(feedReady);

              if (feedReady) {
                feeder.setFeederSpeed(Constants.ManipulatorConstants.kFeederShootOutput);
              } else {
                // If not ready, keep feeder off to avoid weak/early shots.
                feeder.stopFeeder();
              }
            },
            () -> {
              aimReadyLatched[0] = false;
              rpmReadyLatched[0] = false;
              rawReadyStartTimeSec[0] = -1.0;
              AutoAimTuning.resetLiveStatus();
              intake.stopIntakeShooter();
              shooter.stopPIDControlAndShooter();
              feeder.stopFeeder();
            },
            drive,
            intake,
            shooter,
            feeder)
        .beforeStarting(
            () -> {
              startTimeSec[0] = Timer.getFPGATimestamp();
              rawReadyStartTimeSec[0] = -1.0;
              aimReadyLatched[0] = false;
              rpmReadyLatched[0] = false;
              vision.setPipeline(Constants.VisionConstants.kHubTagPipeline);
              AutoAimTuning.resetLiveStatus();
            });
  }

  private static void sortMapByArea(double[] areaMap, double[] rpmMap, double[] capMap) {
    for (int i = 0; i < areaMap.length - 1; i++) {
      for (int j = i + 1; j < areaMap.length; j++) {
        if (areaMap[j] < areaMap[i]) {
          double a = areaMap[i];
          areaMap[i] = areaMap[j];
          areaMap[j] = a;

          double r = rpmMap[i];
          rpmMap[i] = rpmMap[j];
          rpmMap[j] = r;

          double c = capMap[i];
          capMap[i] = capMap[j];
          capMap[j] = c;
        }
      }
    }
  }

  private static double interpolateByArea(double area, double[] areaMap, double[] outputMap) {
    if (area <= areaMap[0]) {
      return outputMap[0];
    }

    for (int i = 0; i < areaMap.length - 1; i++) {
      double leftArea = areaMap[i];
      double rightArea = areaMap[i + 1];
      if (area <= rightArea) {
        double fraction = normalizeByRange(area, leftArea, rightArea);
        return MathUtil.interpolate(outputMap[i], outputMap[i + 1], fraction);
      }
    }

    return outputMap[outputMap.length - 1];
  }

  private static double normalizeByRange(double value, double min, double max) {
    if (max - min <= 1e-6) {
      return 0.0;
    }
    return MathUtil.clamp((value - min) / (max - min), 0.0, 1.0);
  }
}
