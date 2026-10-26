// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// NMRC OPEN READY COMMIT
package frc.robot;

// import edu.wpi.first.wpilibj.XboxController; // unused
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAimCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.Blinken_LED_Subsystem;
import frc.robot.subsystems.JustShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.tuning.AutoAimTuning;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// JoystickButton removed because joystick support is disabled

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /*
   * STUDENT ARCHITECTURE GUIDE
   * --------------------------
   * - Subsystems: low-level hardware control (motors, sensors, direct set/get methods).
   * - Command factories (frc.robot.commands): reusable robot behaviors.
   * - RobotContainer: only wiring (bind buttons, set default commands, register autos).
   *
   * Why this style helps:
   * - Easier to read during troubleshooting
   * - Reusable in teleop and autonomous
   * - Safer command requirements (fewer subsystem conflicts)
   */

  // The robot's subsystems
  // Subsystems are the parts of the robot (drive, shooter, intake, lights).
  // We create one instance of each here so commands can use them.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeShooterSubsystem m_IntakeShooterSubsystem;
  private final JustShooterSubsystem m_justShooterSubsystem;
  private final FeederSubsystem m_FeederSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  // The LED subsystem controls decorative/status lights on the robot.
  private final Blinken_LED_Subsystem m_blinkenLEDSubsystem = new Blinken_LED_Subsystem();

  // Controllers for people driving the robot
  // m_driverController is the main driver (steers and can run simpler actions)
  // m_operatorController is the second person who can control shooter/intake
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> m_autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Constructor: create subsystem instances and configure controls
    // The constructor runs once when the robot program starts.
    m_IntakeShooterSubsystem = new IntakeShooterSubsystem();
    m_justShooterSubsystem = new JustShooterSubsystem();
    m_FeederSubsystem = new FeederSubsystem();
  m_visionSubsystem = new VisionSubsystem();

  // Set up auto commands.
  // These names are used by PathPlanner NamedCommands inside .auto files.
 
 NamedCommands.registerCommand(
    "AutoShoot",
    ShooterCommands.createAutoShootCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem));

 NamedCommands.registerCommand(
    "AutoIntake",
    ShooterCommands.createAutoIntakeCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem));


  // Build a chooser directly from PathPlanner autos found in deploy/pathplanner/autos.
  // This automatically lists available .auto routes on Shuffleboard.
  m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", m_autoChooser);
    AutoAimCommands.initializeTuningEntries();

  // Wire up buttons to commands (see method below). Keeping this in one method makes
  // the constructor easier to scan.
    configureButtonBindings();

    // Centralized LED behavior (no direct LED button bindings).
    // LEDs follow robot state continuously and do not compete with button commands.
    m_blinkenLEDSubsystem.setDefaultCommand(
      LedCommands.createStatusLedCommand(
        m_blinkenLEDSubsystem,
        m_IntakeShooterSubsystem::getIntakeShooterOutput,
        Constants.ManipulatorConstants.kIntakeActiveOutputThreshold,
        () -> m_driverController.leftBumper().getAsBoolean(),
  AutoAimTuning::getHasHubTarget,
  AutoAimTuning::getFeedReady));

  // Default command for the drive subsystem
  // This command runs whenever no other command needs the drive. It reads
  // controller sticks and drives the robot (field-oriented is true here).
  m_robotDrive.setDefaultCommand(
    DriveCommands.createDefaultDriveCommand(
      m_robotDrive,
      m_driverController::getLeftY,
      m_driverController::getLeftX,
      m_driverController::getRightX,
      OIConstants.kDriveDeadband,
      true));

  }

  private void configureButtonBindings() {
  // BUTTON BINDINGS
  // This method connects controller buttons to reusable command objects.
  // Pattern to notice:
  // - .onTrue(...) starts an action
  // - .onFalse(...) performs orderly shutdown
  // - .whileTrue(...) runs continuously while held

  // Driver quick controls:
  // - X button: hold this to make the robot 'set X' (a defensive stance)
  m_driverController.x()
    .whileTrue(DriveCommands.createHoldXCommand(m_robotDrive));
  // - Start button: press to reset the robot's gyro heading to zero
  m_driverController.start()
    .onTrue(DriveCommands.createZeroHeadingCommand(m_robotDrive));

  // OPERATOR CONTROLS (main shooter/intake controls)
  // The operator has detailed control over shooter and intake behavior; we
  // mirror these on the driver where appropriate so either person can act.
  m_operatorController
    .rightBumper()
    .onTrue(ShooterCommands.createShootWhileHeldCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      -3750.0,
      0.8,
      0.8))
    .onFalse(ShooterCommands.createStopShotCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      0.4));

  m_operatorController
    .rightTrigger()
    .onTrue(ShooterCommands.createShootWhileHeldCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      -3000.0,
      0.68,
      0.9))
    .onFalse(ShooterCommands.createStopShotCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      0.4));

  m_operatorController
    .y()
    .onTrue(ShooterCommands.createShootWhileHeldCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      -4200.0,
      0.9,
      0.9))
    .onFalse(ShooterCommands.createStopShotCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      0.4));

  m_operatorController
    .leftTrigger()
    .onTrue(ShooterCommands.createReverseIntakeFeedCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem))
    .onFalse(ShooterCommands.createStopIntakeFeedCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem));

  m_operatorController
    .leftBumper()
    .onTrue(ShooterCommands.createUnjamCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem))
    .onFalse(ShooterCommands.createStopUnjamCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem));

  // DRIVER DUPLICATE CONTROLS
  // The driver gets simpler copies of the operator controls so the driver
  // can shoot or intake without the operator. These are intentionally the
  // same commands to keep behavior predictable.
  m_driverController.y()
    .onTrue(ShooterCommands.createSlowIntakeFeedCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem))
    .onFalse(ShooterCommands.createStopIntakeFeedCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem));

  m_driverController
    .rightTrigger()
    .onTrue(ShooterCommands.createShootWhileHeldCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      -3000.0,
      0.68,
      0.8))
    .onFalse(ShooterCommands.createStopShotCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      0.4));

  m_driverController
    .rightBumper()
    .onTrue(ShooterCommands.createShootWhileHeldCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      -3750.0,
      0.8,
      0.8))
    .onFalse(ShooterCommands.createStopShotCommand(
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      0.4));

  m_driverController
    .leftTrigger()
    .onTrue(ShooterCommands.createReverseIntakeFeedCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem))
    .onFalse(ShooterCommands.createStopIntakeFeedCommand(
      m_IntakeShooterSubsystem,
      m_FeederSubsystem));

  m_driverController
    .leftBumper()
    .whileTrue(AutoAimCommands.createDriverHubAutoAimShootCommand(
      m_robotDrive,
      m_IntakeShooterSubsystem,
      m_justShooterSubsystem,
      m_FeederSubsystem,
      m_visionSubsystem,
      m_driverController::getLeftY,
      m_driverController::getLeftX,
      m_driverController::getRightX,
      OIConstants.kDriveDeadband));

  // End of button bindings. All the important buttons are wired above.
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

  /** Accessor for vision subsystem (Limelight telemetry and pose helpers). */
  public VisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }

}
