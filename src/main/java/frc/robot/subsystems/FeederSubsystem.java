package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

  // Motor configuration for the intakeShooter subsystem
  private static SparkMax feederMotor =
      new SparkMax(12, MotorType.kBrushless); // sets cam ID 12 and type for the shooter motor
  private static SparkMaxConfig feederMotorConfig = new SparkMaxConfig();

  public FeederSubsystem() {
    configurefeederMotor();
  }

  private void configurefeederMotor() {
    feederMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);

    feederMotor.configure(
        feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // this command will load fuel into the shooter.
  public Command runFeederCommand() {
    return Commands.runOnce(() -> feederMotor.set(0.6), this);
  }
  // this command would intake fuel
  public Command reverseFeederCommand() {
    return Commands.runOnce(() -> feederMotor.set(-0.6), this);
  }

  public Command stopFeederCommand() {
    return Commands.runOnce(() -> feederMotor.set(0), this);
  }

  public Command autoFeederCommand() {
    return Commands.sequence(
        Commands.waitSeconds(.2),     
        Commands.runOnce(() -> feederMotor.set(-0.6), this),
        Commands.waitSeconds(5.0),
        Commands.runOnce(() -> feederMotor.set(0), this));
  }

  public Command autoReverseFeederCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> feederMotor.set(-0.6), this),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> feederMotor.set(0), this));
  }
}
