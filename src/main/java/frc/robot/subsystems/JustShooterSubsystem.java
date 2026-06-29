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

public class JustShooterSubsystem extends SubsystemBase {

  // Motor configuration for the shooter
  // This is the actual motor controller that spins the flywheel.
  // The ID (14) is the CAN bus address for the motor controller.
  private static SparkFlex JustShooterMotor =
    new SparkFlex(14, MotorType.kBrushless);
  private static SparkFlexConfig JustShooterMotorConfig = new SparkFlexConfig();
  // Keep track of the last commanded open-loop speed (for debugging)
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

  // Encoder and PID controller
  // The encoder measures the wheel speed (RPM). We use a PIDController to
  // compute motor power to reach a target RPM when closed-loop control is on.
  private final RelativeEncoder m_encoder = JustShooterMotor.getEncoder();
  private final PIDController m_pid = new PIDController(kP, kI, kD);
  private boolean m_pidEnabled = false; // true when closed-loop mode is active
  private double m_pidTargetRPM = 0.0; // desired wheel speed in RPM
  // Shuffleboard entries
  private final GenericEntry m_measuredRpmEntry;
  private final GenericEntry m_targetRpmEntry;
  private final GenericEntry m_pidEnabledEntry;
  private final GenericEntry m_pidOutputEntry;
  // A chooser so users can pick a preset RPM from Shuffleboard
  private final SendableChooser<Double> m_rpmChooser;

  public JustShooterSubsystem() {
    // Set motor configuration (current limits, braking mode, etc.)
    configureJustShooter();

    // Create a small Shuffleboard tab so you can watch values from the
    // shooter while tuning or testing. These show RPM, target RPM, PID
    // enabled state, and the controller output.
    var tab = Shuffleboard.getTab("Shooter");
    // RPM chooser: provides simple preset setpoints to pick from on the dashboard
    m_rpmChooser = new SendableChooser<>();
    m_rpmChooser.setDefaultOption("Default (-5500)", kDefaultShootRPM);
    m_rpmChooser.addOption("Low (-3000)", -3000.0);
    m_rpmChooser.addOption("Medium (-4500)", -4500.0);
    m_rpmChooser.addOption("High (-6000)", -6000.0);
    tab.add("RPM Setpoint", m_rpmChooser);
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

  /** Set shooter motor speed (direct control). */
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

  // COMMANDS: these methods return command objects that can be bound to
  // buttons. Commands are reusable and let us describe start/stop behavior
  // without writing low-level looping code.

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
    // This periodic method runs every robot loop (~20ms). If PID is enabled
    // we read the encoder, compute a new motor output from the PID, clamp it
    // to the safe range [-1,1], and apply it to the motor. We also publish
    // useful values to Shuffleboard so students can see what is happening.
    if (m_pidEnabled) {
      double currentRPM = m_encoder.getVelocity();
      double output = m_pid.calculate(currentRPM, m_pidTargetRPM);
      // PID output is a motor power, clamp to valid range
      output = MathUtil.clamp(output, -1.0, 1.0);
      setShooterSpeed(output);
      // Update shuffleboard values while PID is active
      m_measuredRpmEntry.setDouble(currentRPM);
      m_targetRpmEntry.setDouble(m_pidTargetRPM);
      m_pidEnabledEntry.setBoolean(true);
      m_pidOutputEntry.setDouble(output);
    } else {
      // Even when PID is off, show the encoder RPM and the last target
      // so learners can see the system state.
      m_measuredRpmEntry.setDouble(m_encoder.getVelocity());
      m_targetRpmEntry.setDouble(m_pidTargetRPM);
      m_pidEnabledEntry.setBoolean(false);
      m_pidOutputEntry.setDouble(0.0);
    }
  }
}

