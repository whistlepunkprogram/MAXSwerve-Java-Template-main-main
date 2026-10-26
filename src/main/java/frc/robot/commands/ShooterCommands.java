package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.JustShooterSubsystem;

/**
 * Reusable command factories for shooter/intake workflows.
 *
 * <p>Student note: this class does not talk to joystick buttons directly. It only builds
 * {@link Command} objects. Button wiring happens in {@code RobotContainer}. This separation keeps
 * behavior reusable (teleop + auto) and easier to debug.
 */
public final class ShooterCommands {
  private ShooterCommands() {}

  /**
   * Builds a "shoot while held" flow:
   *
   * <ol>
   *   <li>Run the intake assist motor.
   *   <li>Run shooter PID at the requested RPM and output cap.
   *   <li>After a delay, start feeding the game piece.
   * </ol>
   *
   * <p>Used for trigger/bumper shooting where we want wheel spin-up before feeding.
   */
  public static Command createShootWhileHeldCommand(
      IntakeShooterSubsystem intake,
      JustShooterSubsystem shooter,
      FeederSubsystem feeder,
      double targetRpm,
      double outputCap,
      double feedDelaySeconds) {
    return new ParallelCommandGroup(
        intake.runIntakeShooterCommand(),
        shooter.runHighRPMJustShooterCommand(targetRpm, outputCap),
        Commands.waitSeconds(feedDelaySeconds).andThen(feeder.reverseFeederCommand()));
  }

  /**
   * Builds the shutdown sequence for shooting:
   *
   * <ul>
   *   <li>Stop feeder immediately.
   *   <li>After a short delay, stop intake + shooter.
   * </ul>
   *
   * <p>The delay helps avoid abrupt transitions as the last game piece clears.
   */
  public static Command createStopShotCommand(
      IntakeShooterSubsystem intake,
      JustShooterSubsystem shooter,
      FeederSubsystem feeder,
      double stopDelaySeconds) {
    return Commands.parallel(
        feeder.stopFeederCommand(),
        Commands.waitSeconds(stopDelaySeconds)
            .andThen(intake.stopIntakeShooterCommand(), shooter.stopJustShooterCommand()));
  }

  /** Reverse intake direction and run feeder to back a piece out. */
  public static Command createReverseIntakeFeedCommand(
      IntakeShooterSubsystem intake,
      FeederSubsystem feeder) {
    return new ParallelCommandGroup(intake.reverseIntakeShooterCommand(), feeder.runFeederCommand());
  }

  /** Stop both intake and feeder after reverse-intake operations. */
  public static Command createStopIntakeFeedCommand(
      IntakeShooterSubsystem intake,
      FeederSubsystem feeder) {
    return Commands.parallel(intake.stopIntakeShooterCommand(), feeder.stopFeederCommand());
  }

  /**
   * Slow intake assist + feeder reverse for controlled indexing.
   *
   * <p>This is bound to driver {@code Y} as a lighter-speed handling mode.
   */
  public static Command createSlowIntakeFeedCommand(
      IntakeShooterSubsystem intake,
      FeederSubsystem feeder) {
    return new ParallelCommandGroup(intake.runSlowIntakeCommand(), feeder.reverseFeederCommand());
  }

  /**
   * Unjams by reversing shooter while running intake/feeder to clear stuck game pieces.
   */
  public static Command createUnjamCommand(
      IntakeShooterSubsystem intake,
      JustShooterSubsystem shooter,
      FeederSubsystem feeder) {
    return new ParallelCommandGroup(
        intake.runUnjamShooterCommand(), shooter.reverseJustShooterCommand(), feeder.runFeederCommand());
  }

  /** Stop all motors used during unjam. */
  public static Command createStopUnjamCommand(
      IntakeShooterSubsystem intake,
      JustShooterSubsystem shooter,
      FeederSubsystem feeder) {
    return Commands.parallel(
        intake.stopIntakeShooterCommand(),
        shooter.stopJustShooterCommand(),
        feeder.stopFeederCommand());
  }

  /**
   * Autonomous shooting helper used by PathPlanner named command {@code AutoShoot}.
   */
  public static Command createAutoShootCommand(
      IntakeShooterSubsystem intake,
      JustShooterSubsystem shooter,
      FeederSubsystem feeder) {
    return createShootWhileHeldCommand(intake, shooter, feeder, -3000.0, 0.68, 0.8)
        .withTimeout(7.0)
        .andThen(createStopShotCommand(intake, shooter, feeder, 0.4));
  }

  /**
   * Autonomous intake helper used by PathPlanner named command {@code AutoIntake}.
   */
  public static Command createAutoIntakeCommand(
      IntakeShooterSubsystem intake,
      FeederSubsystem feeder) {
    return Commands.sequence(
        Commands.run(() -> {
              intake.setIntakeShooterSpeed(Constants.ManipulatorConstants.kIntakeAutoOutput);
              feeder.setFeederSpeed(Constants.ManipulatorConstants.kFeederIntakeOutput);
            }, intake, feeder)
            .withTimeout(3.0),
        Commands.parallel(
            Commands.runOnce(intake::stopIntakeShooter, intake),
            Commands.runOnce(feeder::stopFeeder, feeder)));
  }
}
