package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JustShooterSubsystem extends SubsystemBase {

  // Motor configuration for the intakeShooter subsystem
  private static SparkFlex JustShooterMotor =
    new SparkFlex(14, MotorType.kBrushless); // sets cam ID 14 and type for the shooter motor
  private static SparkFlexConfig JustShooterMotorConfig = new SparkFlexConfig();

  public JustShooterSubsystem() {
    configureJustShooter();
  }

  /** Configure motor controller parameters for the intake/shooter motor. */
  private void configureJustShooter() {
    JustShooterMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);

    JustShooterMotor.configure(
        JustShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // SHOOT COMMAND FOR SHOOTING FUEL
  public Command runJustShooterCommand() {
    return Commands.runOnce(
        () -> JustShooterMotor.set(-.7), this); // 1 is the speed the shooter will spin.
  }
  // REVERSE SHOOTER COMMAND FOR REVERSING FUEL
  public Command reverseJustShooterCommand() {
    return Commands.runOnce(
        () -> JustShooterMotor.set(1), this); // -1 is the speed the shooter will spin in reverse.
  }

  // STOP COMMAND FOR IDLING THE SHOOTER
  public Command stopJustShooterCommand() {
    return Commands.runOnce(() -> JustShooterMotor.set(-.2), this); // stops the shooter motor
  }

  // AUTO COMMAND FOR PATH PLANNER TO SHOOT FUEL THAT ARE ALREADY LOADED.
  public Command autoJustShooterCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> JustShooterMotor.set(-.7), this), // Start shooter at 100% speed
        Commands.waitSeconds(5.5), // Wait for 5.5 seconds
        Commands.runOnce(() -> JustShooterMotor.set(0), this));
  }
  // AUTO COMMAND FOR PATH PLANNER TO REVERSE THE SHOOTER
  public Command autoReverseJustShooterCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> JustShooterMotor.set(1), this), // Start shooter at 100% speed in reverse
        Commands.waitSeconds(6), // Wait for 6.0 seconds
        Commands.runOnce(() -> JustShooterMotor.set(0), this));
  }
}

