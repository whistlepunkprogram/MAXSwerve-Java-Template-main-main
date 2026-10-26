package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Blinken_LED_Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Reusable LED state command factories.
 *
 * <p>Student note: LEDs are treated like a "status display" for robot state. We centralize this
 * logic so button commands do not fight over colors.
 */
public final class LedCommands {
  private LedCommands() {}

  /**
   * Default LED state machine (runs continuously):
   *
   * <ol>
   *   <li>If intaking fuel -> strobe gold.
   *   <li>If driver LB active and shot is ready on hub tag -> solid green.
   *   <li>If driver LB active with no target -> solid red.
   *   <li>Otherwise -> heartbeat white.
   * </ol>
   *
   * <p>Priority matters: the first matching condition wins.
   */
  public static Command createStatusLedCommand(
      Blinken_LED_Subsystem leds,
      DoubleSupplier intakeOutputSupplier,
      double intakeActiveThreshold,
      BooleanSupplier driverLbActiveSupplier,
      BooleanSupplier driverLbHasHubTargetSupplier,
      BooleanSupplier driverLbFeedReadySupplier) {
    return Commands.run(
            () -> {
              // Highest priority: actively intaking game pieces.
              if (intakeOutputSupplier.getAsDouble() < intakeActiveThreshold) {
                leds.setLEDColor(Blinken_LED_Subsystem.LEDColor.STROBE_GOLD);
                return;
              }

              boolean driverLbActive = driverLbActiveSupplier.getAsBoolean();
              boolean driverLbHasHubTarget = driverLbHasHubTargetSupplier.getAsBoolean();
              boolean driverLbFeedReady = driverLbFeedReadySupplier.getAsBoolean();

              // Next: LB auto-aim is both locked and ready to feed.
              if (driverLbActive && driverLbHasHubTarget && driverLbFeedReady) {
                leds.setLEDColor(Blinken_LED_Subsystem.LEDColor.SOLID_GREEN);
                return;
              }

              // LB is held but no tag is seen.
              if (driverLbActive && !driverLbHasHubTarget) {
                leds.setLEDColor(Blinken_LED_Subsystem.LEDColor.SOLID_RED);
                return;
              }

              // Default calm status when no other conditions are active.
              leds.setLEDColor(Blinken_LED_Subsystem.LEDColor.HEARTBEAT_WHITE);
            },
            leds)
        .ignoringDisable(true);
  }
}