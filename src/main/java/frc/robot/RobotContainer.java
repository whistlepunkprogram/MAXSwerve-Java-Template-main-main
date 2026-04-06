// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.Blinken_LED_Subsystem;
import frc.robot.subsystems.JustShooterSubsystem;
import frc.robot.subsystems.Limelight4Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final double kShooterMinOutput = 0.6;
    private static final double kShooterMaxOutput = 1;
    private static final double kShooterTriggerDeadband = 0.05;
    private static final double kShooterTyFactor = 0.02;
    private static final double kShooterTargetTimeoutSeconds = 0.25;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeShooterSubsystem m_IntakeShooterSubsystem;
    private final JustShooterSubsystem m_justShooterSubsystem;
  private final FeederSubsystem m_FeederSubsystem;
  private final Blinken_LED_Subsystem m_blinkenLEDSubsystem = new Blinken_LED_Subsystem();
    private final Limelight4Subsystem m_limelight4Subsystem = new Limelight4Subsystem();

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> m_autoChooser;
    private final ShuffleboardTab m_limelightTab = Shuffleboard.getTab("Limelight");
    private double m_lastAButtonShooterOutput = 0.0;
    private double m_lastAButtonTargetTime = -1.0;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    m_IntakeShooterSubsystem = new IntakeShooterSubsystem();
    m_justShooterSubsystem = new JustShooterSubsystem();
    m_FeederSubsystem = new FeederSubsystem();

    // Set up auto commands
    NamedCommands.registerCommand(
        "autoIntake",
        Commands.parallel(
            m_IntakeShooterSubsystem.autoSlowIntakeCommand(),
            m_FeederSubsystem.autoReverseFeederCommand()));
    NamedCommands.registerCommand(
        "autoShoot",
        Commands.parallel(
            m_IntakeShooterSubsystem.autoIntakeShooterCommand(),
            m_justShooterSubsystem.autoJustShooterCommand(),
            m_FeederSubsystem.autoFeederCommand()));
    NamedCommands.registerCommand(
        "autoOutake",
        Commands.parallel(
            m_IntakeShooterSubsystem.autoReverseIntakeShooterCommand(),
            m_FeederSubsystem.autoFeederCommand()));

    // Register individual commands for backward compatibility with PathPlanner
    NamedCommands.registerCommand(
        "autoIntakeShooterCommand", m_IntakeShooterSubsystem.autoIntakeShooterCommand());
    NamedCommands.registerCommand("autoFeederCommand", m_FeederSubsystem.autoFeederCommand());
    NamedCommands.registerCommand("autoJustShooterCommand", m_justShooterSubsystem.autoJustShooterCommand());
    
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
        SmartDashboard.putData("Limelight4", m_limelight4Subsystem);

    m_limelightTab.addBoolean("Has Target", m_limelight4Subsystem::hasTarget).withPosition(0, 0);
    m_limelightTab.addNumber("Tx", m_limelight4Subsystem::getTargetX).withPosition(0, 1);
    m_limelightTab.addNumber("Ty", m_limelight4Subsystem::getTargetY).withPosition(1, 1);
    m_limelightTab.addNumber("Area", m_limelight4Subsystem::getTargetArea).withPosition(2, 1);
    m_limelightTab.addNumber("Tag ID", m_limelight4Subsystem::getAprilTagId).withPosition(3, 1);
    m_limelightTab.addNumber("AutoAim Turn", () -> SmartDashboard.getNumber("AutoAim/TurnCommand", 0.0))
        .withPosition(0, 2);
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

    m_driverController
        .rightTrigger()
        .whileTrue(
            new AutoAimCommand(
                m_robotDrive,
                m_limelight4Subsystem,
                m_driverController::getLeftX,
                m_driverController::getLeftY));

       // Outtake and spit out fuel to floor while holding right bumper button
    m_operatorController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                 m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.SINELON_PARTY),
                m_IntakeShooterSubsystem.runSlowIntakeCommand(),
                m_FeederSubsystem.reverseFeederCommand()))
        .onFalse(
            Commands.parallel(
                 m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
                m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_FeederSubsystem.stopFeederCommand()));

    // Spool up shooter and run intake to SHOOT fuel while holding RIGHT Trigger button
    // Small delay added to prevent the shooter from stalling when the trigger is first pressed,
    // which can cause jams.
    m_operatorController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.STROBE_RED),
                m_IntakeShooterSubsystem.runIntakeShooterCommand(),
                new RunCommand(
                    () -> {
                      double triggerValue = MathUtil.applyDeadband(
                          m_operatorController.getRightTriggerAxis(),
                          kShooterTriggerDeadband);
                      double shooterOutput = -(kShooterMinOutput
                          + (kShooterMaxOutput - kShooterMinOutput) * triggerValue);
                      m_justShooterSubsystem.setShooterSpeed(shooterOutput);
                    },
                    m_justShooterSubsystem),
                Commands.waitSeconds(0.8).andThen(m_FeederSubsystem.reverseFeederCommand())))
        .onFalse(
            Commands.parallel(
                m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
                m_FeederSubsystem.stopFeederCommand(),
                Commands.waitSeconds(0.8)
                    .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                        m_justShooterSubsystem.stopJustShooterCommand())));


    // Intake and spin feeder to load FUEL while holding LEFT Trigger button
    m_operatorController
        .leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.STROBE_BLUE),
                m_IntakeShooterSubsystem.reverseIntakeShooterCommand(),
                m_FeederSubsystem.runFeederCommand()))
        .onFalse(
            Commands.parallel(
                m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
                m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_FeederSubsystem.stopFeederCommand()));

    // Auto-range shooter using Limelight while holding A
    m_operatorController
        .a()
        .whileTrue(
                        new ParallelCommandGroup(
                                m_blinkenLEDSubsystem.setColorCommand(
                                Blinken_LED_Subsystem.LEDColor.STROBE_RED),
                                m_IntakeShooterSubsystem.runIntakeShooterCommand(),
                                new RunCommand(
                                        () -> {
                        double now = Timer.getFPGATimestamp();
                        if (m_limelight4Subsystem.hasTarget()) {
                        m_lastAButtonTargetTime = now;
                        m_lastAButtonShooterOutput = MathUtil.clamp(
                            kShooterMinOutput
                                + (m_limelight4Subsystem.getTargetY() * kShooterTyFactor),
                            kShooterMinOutput,
                            kShooterMaxOutput);
                        }

                        boolean withinTimeout = m_lastAButtonTargetTime > 0
                            && (now - m_lastAButtonTargetTime) <= kShooterTargetTimeoutSeconds;
                        if (withinTimeout) {
                        m_justShooterSubsystem.setShooterSpeed(-m_lastAButtonShooterOutput);
                        } else {
                        m_justShooterSubsystem.setShooterSpeed(0);
                        }
                                        },
                                        m_justShooterSubsystem),
                                Commands.waitSeconds(0.8).andThen(m_FeederSubsystem.reverseFeederCommand())))
                .onFalse(
                        Commands.parallel(
                                m_blinkenLEDSubsystem.setColorCommand(
                                Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
                                m_FeederSubsystem.stopFeederCommand(),
                                Commands.waitSeconds(0.8)
                                        .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                    m_justShooterSubsystem.stopJustShooterCommand(),
                    new InstantCommand(() -> {
                      m_lastAButtonShooterOutput = 0.0;
                      m_lastAButtonTargetTime = -1.0;
                    }))));
     

    // Attempt to unjam shooter by reversing the motor.
    m_operatorController
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.TWINKLES_PARTY),
                m_IntakeShooterSubsystem.runUnjamShooterCommand(),
                m_justShooterSubsystem.reverseJustShooterCommand(),
                m_FeederSubsystem.runFeederCommand()))
        .onFalse(
            Commands.parallel(
                m_blinkenLEDSubsystem.setColorCommand(
                Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
                m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_justShooterSubsystem.stopJustShooterCommand(),
                m_FeederSubsystem.stopFeederCommand()));


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
