package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Reusable command factories for drivetrain operator controls.
 *
 * <p>Student note: these commands are small and focused, which makes button mapping easier to
 * read in {@code RobotContainer}.
 */
public final class DriveCommands {
  private DriveCommands() {}

  /**
   * Default teleop drive command:
   *
   * <ul>
   *   <li>Reads stick values from suppliers.
   *   <li>Applies deadband so tiny stick noise is ignored.
   *   <li>Drives field-relative or robot-relative based on flag.
   * </ul>
   */
  public static Command createDefaultDriveCommand(
      DriveSubsystem drive,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier,
      DoubleSupplier rightXSupplier,
      double deadband,
      boolean fieldRelative) {
    return Commands.run(
        () ->
            drive.drive(
                -MathUtil.applyDeadband(leftYSupplier.getAsDouble(), deadband),
                -MathUtil.applyDeadband(leftXSupplier.getAsDouble(), deadband),
                -MathUtil.applyDeadband(rightXSupplier.getAsDouble(), deadband),
                fieldRelative),
        drive);
  }

  /** Hold wheels in an X pattern to resist pushing/rolling. */
  public static Command createHoldXCommand(DriveSubsystem drive) {
    return Commands.run(drive::setX, drive);
  }

  /** One-shot command to zero the gyro heading. */
  public static Command createZeroHeadingCommand(DriveSubsystem drive) {
    return Commands.runOnce(drive::zeroHeading, drive);
  }
}