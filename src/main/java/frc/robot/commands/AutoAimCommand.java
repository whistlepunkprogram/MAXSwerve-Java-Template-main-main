package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight4Subsystem;

public class AutoAimCommand extends Command {
  private static final double kTargetTimeoutSeconds = 0.25;
  private static final double kTurnSlewRate = 3.0;

  private final DriveSubsystem drive;
  private final Limelight4Subsystem limelight;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Set<Subsystem> requirements;
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(kTurnSlewRate);
  private double lastTurn = 0.0;
  private double lastTargetTime = -1.0;

  public AutoAimCommand(
      DriveSubsystem drive,
      Limelight4Subsystem limelight,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.drive = drive;
    this.limelight = limelight;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.requirements = Set.of(drive);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }

  @Override
  public void initialize() {
    lastTurn = 0.0;
    lastTargetTime = -1.0;
    turnLimiter.reset(0.0);
  }

  @Override
  public void execute() {
  double turn = 0.0;
    boolean hasTarget = limelight.hasTarget();
  double now = Timer.getFPGATimestamp();

    SmartDashboard.putBoolean("AutoAim/HasTarget", hasTarget);
    SmartDashboard.putNumber("AutoAim/Tx", limelight.getTargetX());
    SmartDashboard.putNumber("AutoAim/Ty", limelight.getTargetY());

    if (hasTarget) {
      turn = MathUtil.clamp(
          -limelight.getTargetX() * VisionConstants.kAutoAimP,
          -VisionConstants.kAutoAimMaxRot,
          VisionConstants.kAutoAimMaxRot);
      lastTurn = turn;
      lastTargetTime = now;
      drive.addVisionMeasurement(
          limelight.getBotPoseBlue(),
          limelight.getBotPoseBlueTimestampSeconds());
    }

    boolean withinTimeout = lastTargetTime > 0.0
        && (now - lastTargetTime) <= kTargetTimeoutSeconds;
    if (!hasTarget && withinTimeout) {
      turn = lastTurn;
    }
    if (!hasTarget && !withinTimeout) {
      turn = 0.0;
    }

    double smoothedTurn = turnLimiter.calculate(turn);
    SmartDashboard.putNumber("AutoAim/TurnCommand", smoothedTurn);

    drive.drive(
        -MathUtil.applyDeadband(ySupplier.getAsDouble(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(xSupplier.getAsDouble(), OIConstants.kDriveDeadband),
    smoothedTurn,
        true);
  }

  @Override
  public void end(boolean interrupted) {
    // No-op
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
