// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController; // unused
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.pathplanner.lib.auto.AutoBuilder; // unused
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.Blinken_LED_Subsystem;
import frc.robot.subsystems.JustShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// JoystickButton removed because joystick support is disabled

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // Subsystems are the parts of the robot (drive, shooter, intake, lights).
  // We create one instance of each here so commands can use them.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeShooterSubsystem m_IntakeShooterSubsystem;
  private final JustShooterSubsystem m_justShooterSubsystem;
  private final FeederSubsystem m_FeederSubsystem;
  // The LED subsystem controls decorative/status lights on the robot.
  private final Blinken_LED_Subsystem m_blinkenLEDSubsystem = new Blinken_LED_Subsystem();

  // Controllers for people driving the robot
  // m_driverController is the main driver (steers and can run simpler actions)
  // m_operatorController is the second person who can control shooter/intake
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  private final SendableChooser<String> m_autoChooser;
  // Keep a local map of commands we manually register so we can resolve
  // them later if reflection into NamedCommands doesn't return a Command.
  private final java.util.Map<String, Command> m_localNamedCommands = new java.util.HashMap<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Constructor: create subsystem instances and configure controls
    // The constructor runs once when the robot program starts.
    m_IntakeShooterSubsystem = new IntakeShooterSubsystem();
    m_justShooterSubsystem = new JustShooterSubsystem();
    m_FeederSubsystem = new FeederSubsystem();

    // Set up auto commands
  NamedCommands.registerCommand(
    "autoIntake",
    Commands.parallel(
      m_IntakeShooterSubsystem.autoSlowIntakeCommand(),
      m_FeederSubsystem.autoReverseFeederCommand()));
  m_localNamedCommands.put("autoIntake", Commands.parallel(
      m_IntakeShooterSubsystem.autoSlowIntakeCommand(),
      m_FeederSubsystem.autoReverseFeederCommand()));
  NamedCommands.registerCommand(
    "autoShoot",
    Commands.sequence(
          // Run the main shooting/intake/feeder parallel group for 5 seconds
          Commands.parallel(
            m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.STROBE_RED),
            m_IntakeShooterSubsystem.runIntakeShooterCommand(),
            m_justShooterSubsystem.runJustShooterPIDCommand(),
            Commands.waitSeconds(0.8).andThen(m_FeederSubsystem.reverseFeederCommand())
          ).withTimeout(5.0),
          // After the timeout, run the stop/restore actions to turn things back to default
          Commands.parallel(
            m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
            m_FeederSubsystem.stopFeederCommand(),
            Commands.waitSeconds(0.4)
              .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
                m_justShooterSubsystem.stopJustShooterCommand())
          )
        )
    );

  NamedCommands.registerCommand(
    "autoOutake",
    Commands.parallel(
      m_IntakeShooterSubsystem.autoReverseIntakeShooterCommand(),
      m_FeederSubsystem.autoFeederCommand()));
  m_localNamedCommands.put("autoOutake", Commands.parallel(
      m_IntakeShooterSubsystem.autoReverseIntakeShooterCommand(),
      m_FeederSubsystem.autoFeederCommand()));

    // Register individual commands for backward compatibility with PathPlanner
  NamedCommands.registerCommand(
    "autoIntakeShooterCommand", m_IntakeShooterSubsystem.autoIntakeShooterCommand());
  m_localNamedCommands.put("autoIntakeShooterCommand", m_IntakeShooterSubsystem.autoIntakeShooterCommand());
  NamedCommands.registerCommand("autoFeederCommand", m_FeederSubsystem.autoFeederCommand());
  m_localNamedCommands.put("autoFeederCommand", m_FeederSubsystem.autoFeederCommand());
  NamedCommands.registerCommand("autoJustShooterCommand", m_justShooterSubsystem.autoJustShooterCommand());
  m_localNamedCommands.put("autoJustShooterCommand", m_justShooterSubsystem.autoJustShooterCommand());

  // Register deploy-time PathPlanner autos as named commands so the
  // PathPlanner loader doesn't warn when it looks up these autos. These
  // are currently no-op placeholders; replace them with real sequence
  // commands if you want the autos to execute robot behavior directly.
  NamedCommands.registerCommand("CenterStartShoot", new InstantCommand());
  m_localNamedCommands.put("CenterStartShoot", new InstantCommand());
  NamedCommands.registerCommand("LeftStartShoot", new InstantCommand());
  m_localNamedCommands.put("LeftStartShoot", new InstantCommand());
  NamedCommands.registerCommand("RightStartShoot", new InstantCommand());
  m_localNamedCommands.put("RightStartShoot", new InstantCommand());
    
    // The SendableChooser shows a dropdown on the driver station so you can
    // pick which autonomous routine to run before a match.
  m_autoChooser = new SendableChooser<>();
  m_autoChooser.setDefaultOption("Default", "Default");
    // Try to populate the chooser with PathPlanner-registered autos so they
    // appear as a dropdown on the dashboard. We use reflection in case the
    // NamedCommands API differs between versions or isn't present at compile
    // time.
    try {
      Class<?> cls = NamedCommands.class;
      java.lang.reflect.Method m = null;
      // Try common method names that return a collection of registered auto names
      try {
        m = cls.getMethod("getRegisteredNames");
      } catch (NoSuchMethodException e) {
        try {
          m = cls.getMethod("getRegisteredCommandNames");
        } catch (NoSuchMethodException e2) {
          m = null;
        }
      }

      if (m != null) {
        Object res = m.invoke(null);
        if (res instanceof java.util.Collection) {
          @SuppressWarnings("unchecked")
          java.util.Collection<String> names = (java.util.Collection<String>) res;
          for (String name : names) {
            m_autoChooser.addOption(name, name);
          }
        }
      } else {
        // If NamedCommands doesn't expose a registry method, try populating
        // the chooser from deployed .auto files. PathPlanner deploys autos to
        // /home/lvuser/deploy/pathplanner/autos on the robot. During local
        // testing the files live in src/main/deploy/pathplanner/autos.
        try {
          java.nio.file.Path robotAutos = java.nio.file.Paths.get("/home/lvuser/deploy/pathplanner/autos");
          java.nio.file.Path localAutos = java.nio.file.Paths.get("src/main/deploy/pathplanner/autos");
          java.nio.file.Path chosenDir = java.nio.file.Files.exists(robotAutos) ? robotAutos : localAutos;
          if (java.nio.file.Files.exists(chosenDir)) {
            try (java.util.stream.Stream<java.nio.file.Path> stream = java.nio.file.Files.list(chosenDir)) {
              stream.filter(p -> p.toString().endsWith(".auto")).forEach(p -> {
                String fileName = p.getFileName().toString();
                String name = fileName.substring(0, fileName.length() - ".auto".length());
                m_autoChooser.addOption(name, name);
              });
            }
          } else {
            // Fallback to local map
            for (String name : m_localNamedCommands.keySet()) {
              m_autoChooser.addOption(name, name);
            }
          }
        } catch (Exception ioe) {
          for (String name : m_localNamedCommands.keySet()) {
            m_autoChooser.addOption(name, name);
          }
        }
        // Fallback: populate chooser with locally-registered command names
      }
    } catch (Exception e) {
      System.out.println("Could not populate PathPlanner autos into chooser: " + e.getMessage());
      // Ensure chooser has at least our local commands
      for (String name : m_localNamedCommands.keySet()) {
        m_autoChooser.addOption(name, name);
      }
    }

    SmartDashboard.putData("Auto Mode", m_autoChooser);

    // Wire up buttons to commands (see method below). This keeps the
    // constructor small and readable.
    configureButtonBindings();

  // Default command for the drive subsystem
  // This command runs whenever no other command needs the drive. It reads
  // controller sticks and drives the robot (field-oriented is true here).
  m_robotDrive.setDefaultCommand(
    new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        true),
      m_robotDrive));

  }

  private void configureButtonBindings() {
  // BUTTON BINDINGS
  // This method connects controller buttons to actions (commands). Commands
  // are reusable pieces of behavior (start/stop shooter, run intake, etc.).

  // Driver quick controls:
  // - Right bumper: hold this to make the robot 'set X' (a defensive stance)
  m_driverController.rightBumper()
    .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  // - Start button: press to reset the robot's gyro heading to zero
  m_driverController.start()
    .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

  // OPERATOR CONTROLS (these are the main shooter/intake controls)
  // The operator has detailed control over shooter and intake behavior; we
  // mirror these on the driver where appropriate so either person can act.
  m_operatorController
    .rightBumper()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SINELON_PARTY),
        m_IntakeShooterSubsystem.runSlowIntakeCommand(),
        m_FeederSubsystem.reverseFeederCommand()))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
        m_FeederSubsystem.stopFeederCommand()));

  m_operatorController
    .rightTrigger()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.STROBE_RED),
        m_IntakeShooterSubsystem.runIntakeShooterCommand(),
        m_justShooterSubsystem.runJustShooterPIDCommand(),
        Commands.waitSeconds(0.8).andThen(m_FeederSubsystem.reverseFeederCommand())))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_FeederSubsystem.stopFeederCommand(),
        Commands.waitSeconds(0.4)
          .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
            m_justShooterSubsystem.stopJustShooterCommand())));

  // Additional operator shortcut: X button runs the shooter at a higher RPM
  // while held (6500 RPM setpoint). This gives the operator two preset
  // shooting speeds on different buttons.
  m_operatorController
    .x()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.STROBE_RED),
        m_IntakeShooterSubsystem.runIntakeShooterCommand(),
        m_justShooterSubsystem.runJustShooterPIDCommand(-6500.0),
        Commands.waitSeconds(0.8).andThen(m_FeederSubsystem.reverseFeederCommand())))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_FeederSubsystem.stopFeederCommand(),
        Commands.waitSeconds(0.4)
          .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
            m_justShooterSubsystem.stopJustShooterCommand())));

  m_operatorController
    .leftTrigger()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.STROBE_BLUE),
        m_IntakeShooterSubsystem.reverseIntakeShooterCommand(),
        m_FeederSubsystem.runFeederCommand()))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
        m_FeederSubsystem.stopFeederCommand()));

  m_operatorController
    .leftBumper()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.TWINKLES_PARTY),
        m_IntakeShooterSubsystem.runUnjamShooterCommand(),
        m_justShooterSubsystem.reverseJustShooterCommand(),
        m_FeederSubsystem.runFeederCommand()))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
        m_justShooterSubsystem.stopJustShooterCommand(),
        m_FeederSubsystem.stopFeederCommand()));

  // DRIVER DUPLICATE CONTROLS
  // The driver gets simpler copies of the operator controls so the driver
  // can shoot or intake without the operator. These are intentionally the
  // same commands to keep behavior predictable.
  m_driverController.x()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SINELON_PARTY),
        m_IntakeShooterSubsystem.runSlowIntakeCommand(),
        m_FeederSubsystem.reverseFeederCommand()))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
        m_FeederSubsystem.stopFeederCommand()));

  m_driverController
    .rightTrigger()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.STROBE_RED),
        m_IntakeShooterSubsystem.runIntakeShooterCommand(),
        m_justShooterSubsystem.runJustShooterPIDCommand(),
        Commands.waitSeconds(0.8).andThen(m_FeederSubsystem.reverseFeederCommand())))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_FeederSubsystem.stopFeederCommand(),
        Commands.waitSeconds(0.4)
          .andThen(m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
            m_justShooterSubsystem.stopJustShooterCommand())));

  m_driverController
    .leftTrigger()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.STROBE_BLUE),
        m_IntakeShooterSubsystem.reverseIntakeShooterCommand(),
        m_FeederSubsystem.runFeederCommand()))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
        m_FeederSubsystem.stopFeederCommand()));

  m_driverController
    .leftBumper()
    .onTrue(
      new ParallelCommandGroup(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.TWINKLES_PARTY),
        m_IntakeShooterSubsystem.runUnjamShooterCommand(),
        m_justShooterSubsystem.reverseJustShooterCommand(),
        m_FeederSubsystem.runFeederCommand()))
    .onFalse(
      Commands.parallel(
        m_blinkenLEDSubsystem.setColorCommand(Blinken_LED_Subsystem.LEDColor.SOLID_GOLD),
        m_IntakeShooterSubsystem.stopIntakeShooterCommand(),
        m_justShooterSubsystem.stopJustShooterCommand(),
        m_FeederSubsystem.stopFeederCommand()));

  // End of button bindings. All the important buttons are wired above.
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selected = m_autoChooser.getSelected();
    if (selected == null || selected.equals("Default")) {
      return new InstantCommand();
    }

    // First, try to resolve the named auto via NamedCommands.getCommand(name)
    try {
      java.lang.reflect.Method m = NamedCommands.class.getMethod("getCommand", String.class);
      Object cmd = m.invoke(null, selected);
      if (cmd instanceof Command) {
        return (Command) cmd;
      }
    } catch (NoSuchMethodException ignored) {
      // Method not present on this version of PathPlanner - fallthrough
    } catch (Exception e) {
      System.out.println("Error resolving NamedCommand via reflection: " + e.getMessage());
    }

    // Next, try to load the .auto file into a Command reflectively. Different
    // PathPlanner versions expose different APIs; try a few likely class/method
    // pairs until one succeeds. This allows selecting an auto by filename in
    // the chooser and running the corresponding PathPlanner auto sequence.
    String[] candidateClasses = new String[] {
      "com.pathplanner.lib.auto.AutoBuilder",
      "com.pathplanner.lib.auto.PathPlannerAutoLoader",
      "com.pathplanner.lib.auto.AutoLoader",
      "com.pathplanner.lib.auto.AutoGenerator"
    };

    String[] candidateMethods = new String[] {
      "loadAuto",
      "loadAutoCommand",
      "createAuto",
      "getAutoCommand",
      "loadAutoFromFile"
    };

    for (String clsName : candidateClasses) {
      try {
        Class<?> cls = Class.forName(clsName);
        for (String methodName : candidateMethods) {
          try {
            java.lang.reflect.Method m = cls.getMethod(methodName, String.class);
            Object res = m.invoke(null, selected);
            if (res instanceof Command) {
              return (Command) res;
            }
          } catch (NoSuchMethodException nsme) {
            // try next method
          }
        }
      } catch (ClassNotFoundException cnfe) {
        // try next class
      } catch (Exception e) {
        System.out.println("Error trying to load auto via " + clsName + ": " + e.getMessage());
      }
    }

    // If reflection failed or method missing, check our local map of registered commands
    if (m_localNamedCommands.containsKey(selected)) {
      return m_localNamedCommands.get(selected);
    }

    // Unknown selection: return a no-op and print a warning to help debugging
    System.out.println("Warning: Unknown autonomous selected: " + selected);
    return new InstantCommand();
  }
}
