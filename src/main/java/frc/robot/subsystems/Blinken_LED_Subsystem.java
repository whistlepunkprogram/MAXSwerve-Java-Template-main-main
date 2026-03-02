// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Blinkin-based LED subsystem that mirrors the state machine of the CANdle
 * `LEDSubsystem` you provided, but drives a REV Blinkin over PWM. Patterns are
 * chosen from the built-in `LEDColor` PWM mapping.
 */
public class Blinken_LED_Subsystem extends SubsystemBase {

  private final PWM blinkin; // PWM port for the REV Blinkin LED driver
 
  // Enum for LED colors/patterns (PWM values expected by REV Blinkin)
  public enum LEDColor {
    RAINBOW(-0.99),
    RAINBOW_PARTY(-0.97),
    RAINBOW_OCEAN(-0.95), RAINBOW_LAVA(-0.93), RAINBOW_FOREST(-0.91),
    RAINBOW_GLITTER(-0.89),
    CONFETTI(-0.87),
    SHOT_RED(-0.85), SHOT_BLUE(-0.83), SHOT_WHITE(-0.81),
    SINELON_RAINBOW(-0.79), SINELON_PARTY(-0.77), SINELON_OCEAN(-0.75), SINELON_LAVA(-0.73), SINELON_FOREST(-0.71),
    BPM_RAINBOW(-0.69), BPM_PARTY(-0.67), BPM_OCEAN(-0.65), BPM_LAVA(-0.63), BPM_FOREST(-0.61),
    FIRE_MEDIUM(-0.59), FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW(-0.55), TWINKLES_PARTY(-0.53), TWINKLES_OCEAN(-0.51), TWINKLES_LAVA(-0.49), TWINKLES_FOREST(-0.47),
    COLOR_WAVES_RAINBOW(-0.45), COLOR_WAVES_PARTY(-0.43), COLOR_WAVES_OCEAN(-0.41), COLOR_WAVES_LAVA(-0.39),
    COLOR_WAVES_FOREST(-0.37),
    LARSON_SCANNER_RED(-0.35), LARSON_SCANNER_GRAY(-0.33),
    LIGHT_CHASE_RED(-0.31), LIGHT_CHASE_BLUE(-0.29), LIGHT_CHASE_GRAY(-0.27),
    HEARTBEAT_RED(-0.25), HEARTBEAT_BLUE(-0.23), HEARTBEAT_WHITE(-0.21), HEARTBEAT_GRAY(-0.19),
    BREATH_RED(-0.17), BREATH_BLUE(-0.15), BREATH_GRAY(-0.13),
    STROBE_RED(-0.11), STROBE_BLUE(-0.09), STROBE_GOLD(-0.07), STROBE_WHITE(-0.05),
    COLOR1_END_TO_END_BLEND_TO_BLACK(-0.03), COLOR1_LARSON_SCANNER(-0.01), COLOR1_LIGHT_CHASE(0.01),
    COLOR1_HEARTBEAT_SLOW(0.03), COLOR1_HEARTBEAT_MEDIUM(0.05), COLOR1_HEARTBEAT_FAST(0.07),
    COLOR1_BREATH_SLOW(0.09), COLOR1_BREATH_FAST(0.11), COLOR1_SHOT(0.13), COLOR1_STROBE(0.15),
    COLOR2_END_TO_END_BLEND_TO_BLACK(0.17), COLOR2_LARSON_SCANNER(0.19), COLOR2_LIGHT_CHASE(0.21),
    COLOR2_HEARTBEAT_SLOW(0.23), COLOR2_HEARTBEAT_MEDIUM(0.25), COLOR2_HEARTBEAT_FAST(0.27),
    COLOR2_BREATH_SLOW(0.29), COLOR2_BREATH_FAST(0.31), COLOR2_SHOT(0.33), COLOR2_STROBE(0.35),
    SPARKLE_COLOR1_ON_COLOR2(0.37), SPARKLE_COLOR2_ON_COLOR1(0.39), COLOR_GRADIENT_COLOR1_AND_COLOR2(0.41),
    BPM_COLOR1_AND_COLOR2(0.43), END_TO_END_BLEND_COLOR1_TO_COLOR2(0.45), END_TO_END_BLEND_COLOR1_AND_COLOR2(0.47),
    COLOR1_AND_COLOR2_NO_BLENDING(0.49), TWINKLES_COLOR1_AND_COLOR2(0.51), COLOR_WAVES_COLOR1_AND_COLOR2(0.53),
    SINELON_COLOR1_AND_COLOR2(0.55), SOLID_HOT_PINK(0.57), SOLID_DARK_RED(0.59), SOLID_RED(0.61),
    SOLID_RED_ORANGE(0.63),
    SOLID_ORANGE(0.65), SOLID_GOLD(0.67), SOLID_YELLOW(0.69), SOLID_LAWN_GREEN(0.71), SOLID_LIME(0.73),
    SOLID_DARK_GREEN(0.75), SOLID_GREEN(0.77), SOLID_BLUE_GREEN(0.79), SOLID_AQUA(0.81), SOLID_SKY_BLUE(0.83),
    SOLID_DARK_BLUE(0.85), SOLID_BLUE(0.87), SOLID_BLUE_VIOLET(0.89), SOLID_VIOLET(0.91), SOLID_WHITE(0.93),
    SOLID_GRAY(0.95), SOLID_DARK_GRAY(0.97), BLACK(0.99);

    private final double pwmValue;

    LEDColor(double pwmValue) {
      this.pwmValue = pwmValue;
    }

    public double getPWMValue() {
      return pwmValue;
    }
  }

  private static enum LEDSubsystemState {
    DISABLED,
    NEUTRAL,
    INTAKE,
    MANIPULATOR_NOT_READY,
    MANIPULATOR_READY,
    CLIMB,
    ERROR
  }

  private Alliance m_alliance = null;
  private static LEDSubsystemState m_currentState = LEDSubsystemState.DISABLED;
  private static LEDSubsystemState m_pastState = null;

  /** Default no-argument constructor. */
  public Blinken_LED_Subsystem() {
    this(0); // Default to PWM port 0
  }

  /** Constructor with a specified PWM port. */
  public Blinken_LED_Subsystem(int pwmPort) {
    blinkin = new PWM(pwmPort); // Initialize the PWM port
    SmartDashboard.putBoolean("Blinkin Animation", false);
  }

  /** Sets the Blinkin pattern directly. */
  public void setLEDColor(LEDColor color) {
    blinkin.setSpeed(color.getPWMValue());
  }

  public Command setColorCommand(LEDColor color) {
    return new InstantCommand(() -> setLEDColor(color), this);
  }

  @Override
  public void periodic() {
    boolean colorUpdate = true; 
    if (m_alliance == null) {
      Optional<Alliance> aOpt = DriverStation.getAlliance();
      if (aOpt.isPresent()) {
        m_alliance = aOpt.get();
        colorUpdate = true;
      }
    }

    if ((m_currentState != m_pastState) || colorUpdate) {
      switch (m_currentState) {
        case DISABLED:
          if (m_alliance == Alliance.Blue) {
            setLEDColor(LEDColor.SOLID_BLUE);
          } else if (m_alliance == Alliance.Red) {
            setLEDColor(LEDColor.SOLID_RED);
          } else {
            setLEDColor(LEDColor.CONFETTI);
          }
          break;
        case NEUTRAL:
          // Neutral = alliance color solid 
          if (m_alliance == Alliance.Blue) {
            setLEDColor(LEDColor.SOLID_BLUE);
          } else if (m_alliance == Alliance.Red) {
            setLEDColor(LEDColor.SOLID_GREEN); // choose green for red-neutral 
          } else {
            setLEDColor(LEDColor.RAINBOW);
          }
          break;
        case INTAKE:
          setLEDColor(LEDColor.SINELON_PARTY);
          break;
        case MANIPULATOR_NOT_READY:
          setLEDColor(LEDColor.TWINKLES_PARTY);
          break;
        case MANIPULATOR_READY:
          setLEDColor(LEDColor.STROBE_WHITE);
          break;
        case CLIMB:
                   setLEDColor(LEDColor.LARSON_SCANNER_RED);
          break;
        case ERROR:
          setLEDColor(LEDColor.STROBE_RED);
          break;
      }
      System.out.println("Blinken state: " + m_currentState);
    }
    m_pastState = m_currentState;
  }

  
  public static void setDisabled() {
    m_currentState = LEDSubsystemState.DISABLED;
  }

  public static void setNeutral() {
    m_currentState = LEDSubsystemState.NEUTRAL;
  }

  public static void setIntake() {
    m_currentState = LEDSubsystemState.INTAKE;
  }

  public static void setClimb() {
    m_currentState = LEDSubsystemState.CLIMB;
  }

  public static void setManipulatorNotReady() {
    m_currentState = LEDSubsystemState.MANIPULATOR_NOT_READY;
  }

  public static void setManipulatorReady() {
    m_currentState = LEDSubsystemState.MANIPULATOR_READY;
  }

  public static void setError() {
    m_currentState = LEDSubsystemState.ERROR;
  }

}

