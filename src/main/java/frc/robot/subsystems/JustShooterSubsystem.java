package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * JustShooterSubsystem
 *
 * Learner-friendly explanation:
 * - This subsystem controls a single flywheel shooter motor (a SparkFlex motor)
 *   and an attached encoder that measures RPM. It supports both open-loop
 *   commands (set a fixed motor power) and a simple software PID closed-loop
 *   controller that tries to hold a target RPM.
 *
 * - Typical usage:
 *   - Use open-loop commands for quick manual testing or reversing a jam.
 *   - Use the PID closed-loop commands when you want consistent shot speeds
 *     (pick a preset from the Shuffleboard chooser or pass a specific RPM).
 */
public class JustShooterSubsystem extends SubsystemBase {

  // --- Motor and hardware configuration ---
  // The SparkFlex controller is the motor controller for the shooter flywheel.
  // The integer (14) is the CAN ID assigned to that controller on the robot.
  // MotorType.kBrushless means this is a brushless motor (typical for flywheels).
  private static SparkFlex JustShooterMotor = new SparkFlex(14, MotorType.kBrushless);
  private static SparkFlexConfig JustShooterMotorConfig = new SparkFlexConfig();
  // Keep track of the last commanded open-loop speed (useful for debugging or
  // showing the last manual command while PID is disabled).
  private double m_lastSpeed = 0.0;

  // Default speeds and PID constants. These are starting values and should
  // be tuned on the real robot. Negative sign may be needed depending on
  // motor wiring and wheel orientation.
  private static final double kDefaultShootSpeed = -0.65; // open-loop power
  private static final double kReverseShootSpeed = 1.0; // reverse to clear jams
  private static final double kIdleSpeed = -0.30; // slow idle when not shooting
  // PID closed-loop defaults (target RPM and controller gains)
  private static final double kDefaultShootRPM = -5500.0;
  private static final double kP = 0.00025;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // --- Encoder and PID closed-loop controller ---
  // The motor controller exposes a RelativeEncoder we read to obtain the
  // current wheel speed (in RPM). The simple PIDController here computes a
  // motor output value (roughly a power percentage) that we clamp to [-1,1]
  // before writing to the motor. This is a software closed-loop controller
  // (not using built-in motor-controller closed-loop features), which is
  // easier to inspect and teach for students.
  private final RelativeEncoder m_encoder = JustShooterMotor.getEncoder();
  private final PIDController m_pid = new PIDController(kP, kI, kD);
  // State tracking for the software PID loop
  private boolean m_pidEnabled = false; // true when closed-loop mode is active
  private double m_pidTargetRPM = 0.0; // desired wheel speed in RPM
  // Shuffleboard entries
  private final GenericEntry m_measuredRpmEntry;
  private final GenericEntry m_targetRpmEntry;
  private final GenericEntry m_pidEnabledEntry;
  private final GenericEntry m_pidOutputEntry;
  // --- Shuffleboard telemetry and preset chooser ---
  // We expose measured RPM, target RPM, whether PID is enabled, and the
  // controller output so students can tune and observe behavior live.
  private final SendableChooser<Double> m_rpmChooser;

  public JustShooterSubsystem() {
    // Configure motor parameters (idle mode, current limits, etc.). This
    // centralizes hardware setup so the rest of the code can assume sane
    // defaults for motor behavior.
    configureJustShooter();

    // Create a small Shuffleboard tab so you can watch values from the
    // shooter while tuning or testing. These show RPM, target RPM, PID
    // enabled state, and the controller output. The chooser provides
    // convenient preset target RPMs you can pick without changing code.
    var tab = Shuffleboard.getTab("Shooter");
    m_rpmChooser = new SendableChooser<>();
    m_rpmChooser.setDefaultOption("Default (-5500)", kDefaultShootRPM);
    m_rpmChooser.addOption("Low (-4000)", -4000.0);
    m_rpmChooser.addOption("Medium (-6000)", -6000.0);
    m_rpmChooser.addOption("X Button (-6500)", -6500.0);
    m_rpmChooser.addOption("High (-7000)", -7000.0);
    tab.add("RPM Setpoint", m_rpmChooser);

    // Publish display values to the dashboard so students can observe them.
    m_measuredRpmEntry = tab.add("Measured RPM", 0.0).getEntry();
    m_targetRpmEntry = tab.add("Target RPM", 0.0).getEntry();
    m_pidEnabledEntry = tab.add("PID Enabled", false).getEntry();
    m_pidOutputEntry = tab.add("PID Output", 0.0).getEntry();
  }

  /** Configure motor controller parameters for the intake/shooter motor. */
  private void configureJustShooter() {
    JustShooterMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);

    JustShooterMotor.configure(
        JustShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set shooter motor speed (direct open-loop control).
   *
   * speed: motor power in [-1.0, 1.0].
   * Note: when the software PID is enabled we overwrite the motor output in
   * periodic(); use startPIDControl(...) to enable closed-loop control.
   */
  public void setShooterSpeed(double speed) {
    JustShooterMotor.set(speed);
    m_lastSpeed = speed;
  }

  /** Stop the shooter (set to 0). */
  public void stopShooter() {
    setShooterSpeed(0.0);
  }

  /** Return the last commanded shooter speed. */
  public double getLastSpeed() {
    return m_lastSpeed;
  }

  // ---------------------------------------------------------------------
  // Commands: factory methods that return Command objects you can bind to
  // controller buttons. Using Commands.startEnd/Commands.runOnce keeps the
  // button wiring code declarative and easy to read.
  // ---------------------------------------------------------------------

  // SHOOT (open-loop) — hold to run at a fixed power level
  public Command runJustShooterCommand() {
    return Commands.startEnd(() -> setShooterSpeed(kDefaultShootSpeed), this::stopShooter, this);
  }

  /** Run shooter closed-loop to target RPM while held. */
  public Command runJustShooterPIDCommand() {
    // Use the chooser value if the user picked one on Shuffleboard.
    return Commands.startEnd(
        () -> {
          Double selection = m_rpmChooser.getSelected();
          startPIDControl(selection != null ? selection : kDefaultShootRPM);
        },
        this::stopPIDControl,
        this);
  }

  /** Run shooter closed-loop to a specific RPM while held. */
  public Command runJustShooterPIDCommand(double rpm) {
    return Commands.startEnd(() -> startPIDControl(rpm), this::stopPIDControl, this);
  }

  // REVERSE SHOOTER COMMAND FOR REVERSING FUEL (hold to run)
  public Command reverseJustShooterCommand() {
    return Commands.startEnd(() -> setShooterSpeed(kReverseShootSpeed), this::stopShooter, this);
  }

  // STOP/IDLE — sets the shooter to a low idle speed to keep it spinning
  // (or you can change to stop completely by setting 0.0).
  public Command stopJustShooterCommand() {
    return Commands.runOnce(() -> setShooterSpeed(kIdleSpeed), this);
  }

  // AUTO COMMAND FOR PATH PLANNER TO SHOOT FUEL THAT ARE ALREADY LOADED.
  public Command autoJustShooterCommand() {
  return Commands.sequence(
    Commands.runOnce(() -> startPIDControl(kDefaultShootRPM), this), // Start shooter closed-loop
    Commands.waitSeconds(5.5), // Wait for 5.5 seconds
    Commands.runOnce(this::stopPIDControl, this));
  }
  // AUTO COMMAND FOR PATH PLANNER TO REVERSE THE SHOOTER
  public Command autoReverseJustShooterCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setShooterSpeed(kReverseShootSpeed), this), // Start shooter in reverse
        Commands.waitSeconds(6), // Wait for 6.0 seconds
        Commands.runOnce(this::stopShooter, this));
  }

  // Start the closed-loop controller and set the desired RPM.
  private void startPIDControl(double rpm) {
    m_pidTargetRPM = rpm;
    m_pid.reset();
    m_pidEnabled = true;
  }

  // Stop the closed-loop controller and stop the motor.
  private void stopPIDControl() {
    m_pidEnabled = false;
    stopShooter();
  }

  @Override
  public void periodic() {
    // periodic() runs roughly every 20ms on the robot main loop. We use it to
    // update the software PID loop when enabled and to publish telemetry.
    if (m_pidEnabled) {
      // Read current speed from the encoder (in RPM)
      double currentRPM = m_encoder.getVelocity();
      // Compute controller output to move currentRPM -> targetRPM
      double output = m_pid.calculate(currentRPM, m_pidTargetRPM);
      // PID output is treated as motor power; clamp to [-1,1] for safety
      output = MathUtil.clamp(output, -1.0, 1.0);
      // Apply the output to the motor (closed-loop)
      setShooterSpeed(output);

      // Publish live values so students can tune and observe behavior.
      m_measuredRpmEntry.setDouble(currentRPM);
      m_targetRpmEntry.setDouble(m_pidTargetRPM);
      m_pidEnabledEntry.setBoolean(true);
      m_pidOutputEntry.setDouble(output);
    } else {
      // When PID is disabled, keep publishing the encoder and last target so
      // the dashboard still reflects the current state for debugging.
      m_measuredRpmEntry.setDouble(m_encoder.getVelocity());
      m_targetRpmEntry.setDouble(m_pidTargetRPM);
      m_pidEnabledEntry.setBoolean(false);
      m_pidOutputEntry.setDouble(0.0);
    }
  }
}

