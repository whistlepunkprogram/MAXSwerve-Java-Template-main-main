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

public class RightClimberSubsystem extends SubsystemBase {

  // Motor configuration for the climber subsystem
  private static SparkMax climberMotor = new SparkMax(18, MotorType.kBrushless);
  private static SparkMaxConfig climberMotorConfig = new SparkMaxConfig();

  public RightClimberSubsystem() {
    configureRightClimberMotor();
  }

  private void configureRightClimberMotor() {
    climberMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
    climberMotor.configure(
        climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This command will run the climber motor.
  public Command runRightClimberCommand() {
    return Commands.runOnce(() -> climberMotor.set(0.6), this);
  }

  // This command will reverse the climber motor.
  public Command reverseRightClimberCommand() {
    return Commands.runOnce(() -> climberMotor.set(-0.6), this);
  }

  // This command will stop the climber motor.
  public Command stopRightClimberCommand() {
    return Commands.runOnce(() -> climberMotor.set(0.0), this);
  }
}
