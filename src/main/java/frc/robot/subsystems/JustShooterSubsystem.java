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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JustShooterSubsystem extends SubsystemBase {

  // Motor configuration for the intakeShooter subsystem
  private static SparkFlex JustShooterMotor =
    new SparkFlex(14, MotorType.kBrushless); // sets cam ID 14 and type for the shooter motor
  private static SparkFlexConfig JustShooterMotorConfig = new SparkFlexConfig();
  private double m_lastSpeed = 0.0;

  // Reasonable defaults (can be tuned)
  private static final double kDefaultShootSpeed = -0.65;
  private static final double kReverseShootSpeed = 1.0;
  private static final double kIdleSpeed = -0.30;
  // PID closed-loop defaults (tune on robot)
  private static final double kDefaultShootRPM = -5500.0;
  private static final double kP = 0.00025;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private final RelativeEncoder m_encoder = JustShooterMotor.getEncoder();
  private final PIDController m_pid = new PIDController(kP, kI, kD);
  private boolean m_pidEnabled = false;
  private double m_pidTargetRPM = 0.0;

  public JustShooterSubsystem() {
    configureJustShooter();
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

  // SHOOT COMMAND FOR SHOOTING FUEL (hold to run)
  public Command runJustShooterCommand() {
    return Commands.startEnd(() -> setShooterSpeed(kDefaultShootSpeed), this::stopShooter, this);
  }

  /** Run shooter closed-loop to target RPM while held. */
  public Command runJustShooterPIDCommand() {
    return Commands.startEnd(() -> startPIDControl(kDefaultShootRPM), this::stopPIDControl, this);
  }

  // REVERSE SHOOTER COMMAND FOR REVERSING FUEL (hold to run)
  public Command reverseJustShooterCommand() {
    return Commands.startEnd(() -> setShooterSpeed(kReverseShootSpeed), this::stopShooter, this);
  }

  // STOP COMMAND FOR IDLING THE SHOOTER (sets idle or 0 depending on use)
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

  private void startPIDControl(double rpm) {
    m_pidTargetRPM = rpm;
    m_pid.reset();
    m_pidEnabled = true;
  }

  private void stopPIDControl() {
    m_pidEnabled = false;
    stopShooter();
  }

  @Override
  public void periodic() {
    if (m_pidEnabled) {
      double currentRPM = m_encoder.getVelocity();
      double output = m_pid.calculate(currentRPM, m_pidTargetRPM);
      // PID output is a motor power, clamp to valid range
      output = MathUtil.clamp(output, -1.0, 1.0);
      setShooterSpeed(output);
    }
  }
}

