package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShootDirection;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Test RobotContainer for ShooterSubsystem testing.
 * 
 * To use this instead of the main RobotContainer:
 * 1. In Robot.java, change: new RobotContainer() → new TestShooterRobotContainer()
 * 2. Build and run simulation or deploy
 * 
 * CONTROLLER MAPPING:
 * ┌─────────────────────────────────────────────────────────┐
 * │  A Button     = Enable / Disable shooter               │
 * │  X Button     = Prepare shot (RIGHT, 3m)               │
 * │  Y Button     = Toggle direction (LEFT ↔ RIGHT)        │
 * │  B Button     = Simulate shot (notify shot started)    │
 * │                                                        │
 * │  Right Bumper = Spin up (+250 RPM)                     │
 * │  Left Bumper  = Spin down (-250 RPM)                   │
 * │                                                        │
 * │  D-Pad Up     = Increase hood angle (+5°)              │
 * │  D-Pad Down   = Decrease hood angle (-5°)              │
 * │                                                        │
 * │  Start Button = EMERGENCY STOP                         │
 * │  Back Button  = Clear fault                            │
 * └─────────────────────────────────────────────────────────┘
 */
public class TestShooterRobotContainer {

    // Shooter subsystem for testing
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // Xbox controller
    private final CommandXboxController m_controller =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    // Raw controller for debug
    private final XboxController m_rawController = 
        new XboxController(OperatorConstants.kDriverControllerPort);

    // Track manual angle for testing
    private double m_manualHoodAngle = 150.0; // Start at RIGHT base angle
    
    // Auto test command
    private Command m_autoTestCommand = null;
    
    // Debug counter
    private int m_debugCounter = 0;

    public TestShooterRobotContainer() {
        // Silence joystick warnings when no controller is connected
        DriverStation.silenceJoystickConnectionWarning(true);
        
        configureBindings();
        System.out.println("===========================================");
        System.out.println("  SHOOTER TEST MODE ACTIVE");
        System.out.println("  See TestShooterRobotContainer.java for controls");
        System.out.println("  (Joystick warnings silenced)");
        System.out.println("===========================================");
    }

    private void configureBindings() {
        
        // ============================================================
        // BASIC CONTROLS
        // ============================================================
        
        // A Button: Enable/Disable shooter
        m_controller.a().onTrue(Commands.runOnce(() -> {
            if (!m_shooter.getShooterState().isActive()) {
                m_shooter.enableShooter();
                System.out.println("[TEST] Shooter ENABLED");
            } else {
                m_shooter.disableShooter();
                System.out.println("[TEST] Shooter DISABLED");
            }
        }));

        // X Button: Prepare shot at 3 meters (RIGHT direction)
        m_controller.x().onTrue(Commands.runOnce(() -> {
            m_shooter.prepareShot(ShootDirection.RIGHT, 3.0);
            System.out.println("[TEST] Preparing shot: RIGHT, 3.0m");
        }));

        // Y Button: Toggle direction (LEFT <-> RIGHT)
        m_controller.y().onTrue(Commands.runOnce(() -> {
            m_shooter.toggleShootDirection();
            System.out.println("[TEST] Direction toggled to: " + m_shooter.getShootDirection());
        }));

        // B Button: Simulate shot (tell shooter a ball is being fed)
        m_controller.b().onTrue(Commands.runOnce(() -> {
            if (m_shooter.isSafeToFeed()) {
                m_shooter.notifyShotStarted();
                System.out.println("[TEST] Shot fired! (notifyShotStarted called)");
            } else {
                System.out.println("[TEST] Cannot shoot - isSafeToFeed() = false");
                System.out.println("       State: " + m_shooter.getShooterState());
                System.out.println("       AtSpeed: " + m_shooter.isAtSpeed());
                System.out.println("       AtAngle: " + m_shooter.isAtAngle());
            }
        }));

        // ============================================================
        // RPM CONTROLS
        // ============================================================

        // Right Bumper: Spin up (+250 RPM)
        m_controller.rightBumper().onTrue(Commands.runOnce(() -> {
            m_shooter.spinUp();
            System.out.println("[TEST] Spin up → Target RPM: " + m_shooter.getTargetFlywheelRPM());
        }));

        // Left Bumper: Spin down (-250 RPM)
        m_controller.leftBumper().onTrue(Commands.runOnce(() -> {
            m_shooter.spinDown();
            System.out.println("[TEST] Spin down → Target RPM: " + m_shooter.getTargetFlywheelRPM());
        }));

        // ============================================================
        // HOOD ANGLE CONTROLS
        // ============================================================

        // D-Pad Up: Increase hood angle +5°
        m_controller.povUp().onTrue(Commands.runOnce(() -> {
            // Get current target angle and add 5°
            double currentAngle = m_shooter.getTargetHoodAngle();
            double newAngle = currentAngle + 5.0;
            m_shooter.setTargetHoodAngle(newAngle);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();  // Sync after clamping
            System.out.println("[TEST] Hood angle +5°: " + currentAngle + "° → " + m_manualHoodAngle + "°");
        }));

        // D-Pad Down: Decrease hood angle -5°
        m_controller.povDown().onTrue(Commands.runOnce(() -> {
            // Get current target angle and subtract 5°
            double currentAngle = m_shooter.getTargetHoodAngle();
            double newAngle = currentAngle - 5.0;
            m_shooter.setTargetHoodAngle(newAngle);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();  // Sync after clamping
            System.out.println("[TEST] Hood angle -5°: " + currentAngle + "° → " + m_manualHoodAngle + "°");
        }));

        // D-Pad Left: Set to LEFT direction preset (30°)
        m_controller.povLeft().onTrue(Commands.runOnce(() -> {
            m_shooter.setShootDirection(ShootDirection.LEFT);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();  // Sync with actual angle
            System.out.println("[TEST] Direction: LEFT, Hood angle: " + m_manualHoodAngle + "°");
        }));

        // D-Pad Right: Set to RIGHT direction preset (150°)
        m_controller.povRight().onTrue(Commands.runOnce(() -> {
            m_shooter.setShootDirection(ShootDirection.RIGHT);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();  // Sync with actual angle
            System.out.println("[TEST] Direction: RIGHT, Hood angle: " + m_manualHoodAngle + "°");
        }));

        // ============================================================
        // SAFETY CONTROLS
        // ============================================================

        // Start Button: Emergency Stop
        m_controller.start().onTrue(Commands.runOnce(() -> {
            m_shooter.emergencyStop();
            System.out.println("[TEST] *** EMERGENCY STOP ***");
        }));

        // Back Button: Clear Fault
        m_controller.back().onTrue(Commands.runOnce(() -> {
            m_shooter.clearFault();
            System.out.println("[TEST] Fault cleared");
        }));

        // ============================================================
        // TEST PRESETS (using triggers)
        // ============================================================

        // Left Trigger (held): Prepare LEFT shot at 2m
        m_controller.leftTrigger(0.5).onTrue(Commands.runOnce(() -> {
            m_shooter.prepareShot(ShootDirection.LEFT, 2.0);
            System.out.println("[TEST] Preset: LEFT, 2.0m");
        }));

        // Right Trigger (held): Prepare RIGHT shot at 4m
        m_controller.rightTrigger(0.5).onTrue(Commands.runOnce(() -> {
            m_shooter.prepareShot(ShootDirection.RIGHT, 4.0);
            System.out.println("[TEST] Preset: RIGHT, 4.0m");
        }));
    }

    /**
     * Call this when teleop starts to run automatic test (NO CONTROLLER NEEDED!)
     */
    public void teleopInit() {
        System.out.println("\n========================================");
        System.out.println("  TELEOP STARTED - D-PAD TEST RUNNING");
        System.out.println("  Testing hood angle controls!");
        System.out.println("========================================\n");
        
        // Cancel any running test
        if (m_autoTestCommand != null) {
            m_autoTestCommand.cancel();
        }
        
        // Run automatic test sequence - testing D-Pad functions
        m_autoTestCommand = Commands.sequence(
            // Enable shooter
            Commands.runOnce(() -> {
                System.out.println("[TEST] Enabling shooter...");
                m_shooter.enableShooter();
            }),
            
            Commands.waitSeconds(0.5),
            
            // ============================================================
            // TEST D-PAD RIGHT: Switch to RIGHT direction (150°)
            // ============================================================
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD RIGHT: Switch to RIGHT direction");
                m_shooter.setShootDirection(ShootDirection.RIGHT);
                m_manualHoodAngle = 150.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Direction: RIGHT");
                System.out.println("    Hood angle set to: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(1.0),
            
            // ============================================================
            // TEST D-PAD UP: Increase hood angle +5°
            // ============================================================
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD UP: Increase hood angle +5°");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle += 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(0.5),
            
            // D-PAD UP again
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD UP: Increase hood angle +5°");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle += 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(0.5),
            
            // ============================================================
            // TEST D-PAD DOWN: Decrease hood angle -5°
            // ============================================================
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD DOWN: Decrease hood angle -5°");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle -= 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(0.5),
            
            // D-PAD DOWN again
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD DOWN: Decrease hood angle -5°");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle -= 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(0.5),
            
            // D-PAD DOWN again
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD DOWN: Decrease hood angle -5°");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle -= 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(1.0),
            
            // ============================================================
            // TEST D-PAD LEFT: Switch to LEFT direction (30°)
            // ============================================================
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD LEFT: Switch to LEFT direction");
                m_shooter.setShootDirection(ShootDirection.LEFT);
                m_manualHoodAngle = 30.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Direction: LEFT");
                System.out.println("    Hood angle set to: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(1.0),
            
            // D-PAD UP on LEFT side
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD UP: Increase hood angle +5° (LEFT side)");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle += 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(0.5),
            
            // D-PAD DOWN on LEFT side
            Commands.runOnce(() -> {
                System.out.println("\n>>> D-PAD DOWN: Decrease hood angle -5° (LEFT side)");
                double before = m_shooter.getTargetHoodAngle();
                m_manualHoodAngle -= 5.0;
                m_shooter.setTargetHoodAngle(m_manualHoodAngle);
                System.out.println("    Before: " + before + "° → After: " + m_shooter.getTargetHoodAngle() + "°");
            }),
            
            Commands.waitSeconds(1.0),
            
            // ============================================================
            // SUMMARY
            // ============================================================
            Commands.runOnce(() -> {
                m_shooter.disableShooter();
                System.out.println("\n========================================");
                System.out.println("  D-PAD TEST COMPLETE!");
                System.out.println("========================================");
                System.out.println("  ✓ D-PAD RIGHT: Set direction RIGHT (150°)");
                System.out.println("  ✓ D-PAD UP:    Increased angle +5°");
                System.out.println("  ✓ D-PAD DOWN:  Decreased angle -5°");
                System.out.println("  ✓ D-PAD LEFT:  Set direction LEFT (30°)");
                System.out.println("========================================\n");
            })
        );
        
        m_autoTestCommand.schedule();
    }
    
    /**
     * Returns an autonomous command for testing.
     * This runs a simple test sequence.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Enable shooter
            Commands.runOnce(() -> {
                System.out.println("[AUTO TEST] Enabling shooter...");
                m_shooter.enableShooter();
            }),
            
            // Wait a moment
            Commands.waitSeconds(0.5),
            
            // Prepare shot
            Commands.runOnce(() -> {
                System.out.println("[AUTO TEST] Preparing shot: RIGHT, 3m");
                m_shooter.prepareShot(ShootDirection.RIGHT, 3.0);
            }),
            
            // Wait for shooter to be ready (fixed time for simulation)
            Commands.waitSeconds(2.0),
            
            // Report status
            Commands.runOnce(() -> {
                System.out.println("[AUTO TEST] Shooter state: " + m_shooter.getShooterState());
                System.out.println("[AUTO TEST] Ready to shoot: " + m_shooter.isReadyToShoot());
            }),
            
            // Simulate shot if ready
            Commands.runOnce(() -> {
                if (m_shooter.isSafeToFeed()) {
                    m_shooter.notifyShotStarted();
                    System.out.println("[AUTO TEST] Shot fired!");
                } else {
                    System.out.println("[AUTO TEST] Not safe to feed");
                }
            }),
            
            // Wait then disable
            Commands.waitSeconds(1.0),
            
            Commands.runOnce(() -> {
                m_shooter.disableShooter();
                System.out.println("[AUTO TEST] Test complete, shooter disabled");
            })
        );
    }
    
    // Track last right stick state for edge detection
    private boolean m_lastRightStickUp = false;
    private boolean m_lastRightStickDown = false;
    private boolean m_lastRightStickLeft = false;
    private boolean m_lastRightStickRight = false;
    
    /**
     * Call this periodically during teleop to debug controller input.
     * Shows controller button states in SmartDashboard.
     * Also handles RIGHT STICK as alternative to D-Pad!
     */
    public void teleopPeriodic() {
        // Log controller state to SmartDashboard (every cycle)
        SmartDashboard.putBoolean("Controller/A_Button", m_rawController.getAButton());
        SmartDashboard.putBoolean("Controller/B_Button", m_rawController.getBButton());
        SmartDashboard.putBoolean("Controller/X_Button", m_rawController.getXButton());
        SmartDashboard.putBoolean("Controller/Y_Button", m_rawController.getYButton());
        SmartDashboard.putBoolean("Controller/LB", m_rawController.getLeftBumper());
        SmartDashboard.putBoolean("Controller/RB", m_rawController.getRightBumper());
        SmartDashboard.putNumber("Controller/POV", m_rawController.getPOV());
        SmartDashboard.putNumber("Controller/RightX", m_rawController.getRightX());
        SmartDashboard.putNumber("Controller/RightY", m_rawController.getRightY());
        
        // ============================================================
        // RIGHT STICK AS ALTERNATIVE TO D-PAD (since D-Pad often doesn't work in sim)
        // AXES SWAPPED: Your controller has X/Y swapped
        // ============================================================
        double rightY = m_rawController.getRightY();
        double rightX = m_rawController.getRightX();
        
        // Debug: Show raw axis values
        SmartDashboard.putString("RightStick/Raw", 
            String.format("X=%.2f Y=%.2f", rightX, rightY));
        
        // Right Stick UP (X < -0.5) → Increase hood angle +5°
        // (Axes are swapped on this controller)
        boolean rightStickUp = rightX < -0.5;
        if (rightStickUp && !m_lastRightStickUp) {
            double currentAngle = m_shooter.getTargetHoodAngle();
            double newAngle = currentAngle + 5.0;
            m_shooter.setTargetHoodAngle(newAngle);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();
            System.out.println("[RIGHT STICK UP] Hood angle +5°: " + currentAngle + "° → " + m_manualHoodAngle + "°");
        }
        m_lastRightStickUp = rightStickUp;
        
        // Right Stick DOWN (X > 0.5) → Decrease hood angle -5°
        boolean rightStickDown = rightX > 0.5;
        if (rightStickDown && !m_lastRightStickDown) {
            double currentAngle = m_shooter.getTargetHoodAngle();
            double newAngle = currentAngle - 5.0;
            m_shooter.setTargetHoodAngle(newAngle);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();
            System.out.println("[RIGHT STICK DOWN] Hood angle -5°: " + currentAngle + "° → " + m_manualHoodAngle + "°");
        }
        m_lastRightStickDown = rightStickDown;
        
        // RIGHT STICK LEFT/RIGHT - try both axes since controller mapping varies
        // Try Y axis
        boolean rightStickLeftY = rightY < -0.5;
        boolean rightStickRightY = rightY > 0.5;
        // Try X axis (in case they're swapped)
        boolean rightStickLeftX = rightX < -0.5;
        boolean rightStickRightX = rightX > 0.5;
        
        // LEFT STICK for direction (more reliable)
        double leftX = m_rawController.getLeftX();
        boolean leftStickLeft = leftX < -0.5;
        boolean leftStickRight = leftX > 0.5;
        
        // Combine: Any left input triggers LEFT direction
        boolean anyLeft = rightStickLeftY || leftStickLeft;
        if (anyLeft && !m_lastRightStickLeft) {
            m_shooter.setShootDirection(ShootDirection.LEFT);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();
            System.out.println("[STICK LEFT] Direction: LEFT, Hood angle: " + m_manualHoodAngle + "°");
        }
        m_lastRightStickLeft = anyLeft;
        
        // Combine: Any right input triggers RIGHT direction  
        boolean anyRight = rightStickRightY || leftStickRight;
        if (anyRight && !m_lastRightStickRight) {
            m_shooter.setShootDirection(ShootDirection.RIGHT);
            m_manualHoodAngle = m_shooter.getTargetHoodAngle();
            System.out.println("[STICK RIGHT] Direction: RIGHT, Hood angle: " + m_manualHoodAngle + "°");
        }
        m_lastRightStickRight = anyRight;
        
        // ============================================================
        // BUTTON DETECTION
        // ============================================================
        boolean anyButton = m_rawController.getAButton() || m_rawController.getBButton() ||
                           m_rawController.getXButton() || m_rawController.getYButton() ||
                           m_rawController.getLeftBumper() || m_rawController.getRightBumper() ||
                           m_rawController.getStartButton() || m_rawController.getBackButton();
        
        // Only print once when button is first pressed (not while held)
        if (anyButton && m_debugCounter == 0) {
            m_debugCounter = 10;  // Debounce: wait 10 cycles before detecting again
            System.out.println("[CONTROLLER] Button detected!");
            if (m_rawController.getAButton()) System.out.println("  → A pressed");
            if (m_rawController.getBButton()) System.out.println("  → B pressed");
            if (m_rawController.getXButton()) System.out.println("  → X pressed");
            if (m_rawController.getYButton()) System.out.println("  → Y pressed");
            if (m_rawController.getLeftBumper()) System.out.println("  → LB pressed");
            if (m_rawController.getRightBumper()) System.out.println("  → RB pressed");
            if (m_rawController.getStartButton()) System.out.println("  → START pressed");
            if (m_rawController.getBackButton()) System.out.println("  → BACK pressed");
        }
        
        // Decrement debounce counter
        if (m_debugCounter > 0 && !anyButton) {
            m_debugCounter--;
        }
        
        // D-Pad detection (POV) - may not work in simulation
        int pov = m_rawController.getPOV();
        if (pov != -1) {
            System.out.println("[D-PAD] POV: " + pov + "° (" + getPOVName(pov) + ")");
        }
    }
    
    private String getPOVName(int pov) {
        switch (pov) {
            case 0: return "UP";
            case 45: return "UP-RIGHT";
            case 90: return "RIGHT";
            case 135: return "DOWN-RIGHT";
            case 180: return "DOWN";
            case 225: return "DOWN-LEFT";
            case 270: return "LEFT";
            case 315: return "UP-LEFT";
            default: return "?";
        }
    }
}
