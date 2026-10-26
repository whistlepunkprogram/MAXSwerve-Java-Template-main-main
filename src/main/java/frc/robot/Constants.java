// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.3;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kTrackWidth = Units.inchesToMeters(24);
    public static final double kWheelBase = Units.inchesToMeters(25);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    // Slew limits (units per second) used by DriveSubsystem
    public static final double kDriveTranslationRateLimit = 3.0;
    public static final double kDriveRotationRateLimit = 4.0;

  // Xbox controller deadband and button/axis choices are used for driver/operator
  // If you need to support a joystick later, add mappings here.
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // PathPlanner HolonomicDriveController PID constants
    public static final double kPPTranslationP = 1.0;
    public static final double kPPRotationP = 1.0;

    // PathPlanner robot config
    public static final double kRobotMassKg = Units.lbsToKilograms(65); // converted to kg
    public static final double kRobotMOI = 6.883;
    public static final double kWheelRadiusMeters = ModuleConstants.kWheelDiameterMeters / 2.0;
    public static final double kMaxDriveSpeedMetersPerSec = DriveConstants.kMaxSpeedMetersPerSecond;
    public static final double kDriveCurrentLimitAmps = 50.0;
    public static final double kWheelCOF = 1.2;

    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
      new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
      new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
      new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
      new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)
    };

    public static final RobotConfig kPPConfig = new RobotConfig(
      kRobotMassKg,
      kRobotMOI,
      new ModuleConfig(
        kWheelRadiusMeters,
        kMaxDriveSpeedMetersPerSec,
        kWheelCOF,
        DCMotor.getNEO(1).withReduction(ModuleConstants.kDrivingMotorReduction),
        kDriveCurrentLimitAmps,
        1),
      kModuleTranslations);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    public static final String kLimelightTableName = "limelight";
    public static final int kHubTagPipeline = 0;

    // 2026 Rebuilt Hub AprilTag IDs (update if your field/event uses different IDs).
    public static final int[] kHubAprilTagIds = {11, 12, 13, 14, 15, 16};

    public static final double kAutoAimP = 0.02;
    public static final double kAutoAimMaxRot = 0.6;

    // Dynamic shooter settings for hub targeting.
    // Bounded between existing driver/operator shot presets:
    // low power (right trigger): -3000 RPM @ 0.68 cap
    // high power (right bumper): -3750 RPM @ 0.80 cap
    public static final double kAutoAimLowShotRPM = -3000.0;
    public static final double kAutoAimHighShotRPM = -3750.0;
    public static final double kAutoAimLowShotOutputCap = 0.68;
    public static final double kAutoAimHighShotOutputCap = 0.80;

    // Limelight target area calibration range for interpolation
    // (far target -> lower area, near target -> higher area).
    public static final double kAutoAimFarTargetArea = 0.35;
    public static final double kAutoAimNearTargetArea = 3.50;

    // Piecewise area -> shooter map defaults used for LB auto-aim shot tuning.
    // Area points should increase from far -> near. RPM is negative due to motor direction.
    public static final double[] kAutoAimAreaMapDefaults = {0.35, 0.90, 1.80, 3.50};
    public static final double[] kAutoAimRpmMapDefaults = {-3750.0, -3500.0, -3250.0, -3000.0};
    public static final double[] kAutoAimOutputCapMapDefaults = {0.80, 0.76, 0.72, 0.68};

    // Hysteresis thresholds for "ready" state.
    // Enter threshold is tighter; exit threshold is looser to reduce chatter.
    public static final double kAutoAimAimEnterToleranceDeg = 1.2;
    public static final double kAutoAimAimExitToleranceDeg = 1.8;
    public static final double kAutoAimRpmEnterTolerance = 140.0;
    public static final double kAutoAimRpmExitTolerance = 220.0;

    // Debounce to require stable readiness before feeding.
    public static final double kAutoAimReadyDebounceSec = 0.20;

    // Pose estimator standard deviations (x, y in meters, heading in radians)
    public static final Vector<N3> kStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
    public static final Vector<N3> kVisionStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
  }

  public static final class ManipulatorConstants {
    // Common intake/feeder outputs used by command factories.
    public static final double kIntakeShootOutput = 0.4;
    public static final double kIntakeAutoOutput = 0.35;
    public static final double kFeederShootOutput = -0.6;
    public static final double kFeederIntakeOutput = 0.6;

    // LED state helper threshold: intake considered active when commanded below this output.
    public static final double kIntakeActiveOutputThreshold = -0.1;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
