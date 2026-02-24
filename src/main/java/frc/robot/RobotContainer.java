// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.LeftClimberSubsystem;
import frc.robot.subsystems.RightClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeShooterSubsystem m_IntakeShooterSubsystem;
  private final FeederSubsystem m_FeederSubsystem;
  private final LeftClimberSubsystem m_LeftClimberSubsystem;
  private final RightClimberSubsystem m_RightClimberSubsystem;

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    m_IntakeShooterSubsystem = new IntakeShooterSubsystem();
    m_FeederSubsystem = new FeederSubsystem();
    m_LeftClimberSubsystem = new LeftClimberSubsystem();
    m_RightClimberSubsystem = new RightClimberSubsystem();

    
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController.rightBumper()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.start()
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

       // Outtake and spit out fuel to floor while holding right bumper button
    m_operatorController
        .rightBumper()
        .onTrue(
            new ParallelCommandGroup(
                m_IntakeShooterSubsystem.runSlowIntakeCommand(),
                m_FeederSubsystem.reverseFeederCommand()))
        .onFalse(
            Commands.parallel(
                m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_FeederSubsystem.stopFeederCommand()));

    // Spool up shooter and run intake to SHOOT fuel while holding RIGHT Trigger button
    // Small delay added to prevent the shooter from stalling when the trigger is first pressed,
    // which can cause jams.
    m_operatorController
        .rightTrigger()
        .onTrue(
            new ParallelCommandGroup(
                m_IntakeShooterSubsystem.runIntakeShooterCommand(),
                Commands.waitSeconds(0.2).andThen(m_FeederSubsystem.reverseFeederCommand())))
        .onFalse(
            Commands.parallel(
                m_FeederSubsystem.stopFeederCommand(),
                Commands.waitSeconds(0.2)
                    .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand())));

    // Intake and spin feeder to load FUEL while holding LEFT Trigger button
    m_operatorController
        .leftTrigger()
        .onTrue(
            new ParallelCommandGroup(
                m_IntakeShooterSubsystem.reverseIntakeShooterCommand(),
                m_FeederSubsystem.runFeederCommand()))
        .onFalse(
            Commands.parallel(
                m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_FeederSubsystem.stopFeederCommand()));

    // Attempt to unjam shooter by reversing the motor.
    m_operatorController
        .leftBumper()
        .onTrue(
            new ParallelCommandGroup(
                m_IntakeShooterSubsystem.runUnjamShooterCommand(),
                m_FeederSubsystem.runFeederCommand()))
        .onFalse(
            Commands.parallel(
                m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_FeederSubsystem.stopFeederCommand()));

    // Climber controls: left stick up/down drives both climbers in parallel.
    double climberDeadband = 0.2;
    Command climberUp =
        Commands.parallel(
            m_LeftClimberSubsystem.runLeftClimberCommand(),
            m_RightClimberSubsystem.runRightClimberCommand());
    Command climberDown =
        Commands.parallel(
            m_LeftClimberSubsystem.reverseLeftClimberCommand(),
            m_RightClimberSubsystem.reverseRightClimberCommand());
    Command climberStop =
        Commands.parallel(
            m_LeftClimberSubsystem.stopLeftClimberCommand(),
            m_RightClimberSubsystem.stopRightClimberCommand());

    m_operatorController
        .axisLessThan(XboxController.Axis.kLeftY.value, -climberDeadband)
        .whileTrue(climberUp)
        .onFalse(climberStop);

    m_operatorController
        .axisGreaterThan(XboxController.Axis.kLeftY.value, climberDeadband)
        .whileTrue(climberDown)
        .onFalse(climberStop);
  }        


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = m_autoChooser.getSelected();
    return selected != null ? selected : new InstantCommand();
  }
}
