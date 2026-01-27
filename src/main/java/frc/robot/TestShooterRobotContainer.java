package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShootDirection;
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

    // Track manual angle for testing
    private double m_manualHoodAngle = 150.0; // Start at RIGHT base angle
    
    // Auto test command
    private Command m_autoTestCommand = null;

    public TestShooterRobotContainer() {
        configureBindings();
        System.out.println("===========================================");
        System.out.println("  SHOOTER TEST MODE ACTIVE");
        System.out.println("  See TestShooterRobotContainer.java for controls");
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

        // D-Pad Up: Increase hood angle
        m_controller.povUp().onTrue(Commands.runOnce(() -> {
            m_manualHoodAngle += 5.0;
            m_shooter.setTargetHoodAngle(m_manualHoodAngle);
            System.out.println("[TEST] Hood angle +5° → Target: " + m_shooter.getTargetHoodAngle());
        }));

        // D-Pad Down: Decrease hood angle
        m_controller.povDown().onTrue(Commands.runOnce(() -> {
            m_manualHoodAngle -= 5.0;
            m_shooter.setTargetHoodAngle(m_manualHoodAngle);
            System.out.println("[TEST] Hood angle -5° → Target: " + m_shooter.getTargetHoodAngle());
        }));

        // D-Pad Left: Set to LEFT direction preset
        m_controller.povLeft().onTrue(Commands.runOnce(() -> {
            m_shooter.setShootDirection(ShootDirection.LEFT);
            m_manualHoodAngle = 30.0;
            System.out.println("[TEST] Direction: LEFT, Base angle: 30°");
        }));

        // D-Pad Right: Set to RIGHT direction preset
        m_controller.povRight().onTrue(Commands.runOnce(() -> {
            m_shooter.setShootDirection(ShootDirection.RIGHT);
            m_manualHoodAngle = 150.0;
            System.out.println("[TEST] Direction: RIGHT, Base angle: 150°");
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
        System.out.println("  TELEOP STARTED - AUTO TEST RUNNING");
        System.out.println("  No controller needed!");
        System.out.println("========================================\n");
        
        // Cancel any running test
        if (m_autoTestCommand != null) {
            m_autoTestCommand.cancel();
        }
        
        // Run automatic test sequence
        m_autoTestCommand = Commands.sequence(
            // Enable shooter
            Commands.runOnce(() -> {
                System.out.println("[TELEOP TEST] Enabling shooter...");
                m_shooter.enableShooter();
            }),
            
            Commands.waitSeconds(0.5),
            
            // Prepare shot RIGHT
            Commands.runOnce(() -> {
                System.out.println("[TELEOP TEST] Preparing shot: RIGHT, 3m");
                m_shooter.prepareShot(ShootDirection.RIGHT, 3.0);
            }),
            
            Commands.waitSeconds(1.0),
            
            // Check status
            Commands.runOnce(() -> {
                System.out.println("[TELEOP TEST] State: " + m_shooter.getShooterState());
                System.out.println("[TELEOP TEST] Ready: " + m_shooter.isReadyToShoot());
            }),
            
            // Fire shot 1
            Commands.runOnce(() -> {
                if (m_shooter.isSafeToFeed()) {
                    m_shooter.notifyShotStarted();
                    System.out.println("[TELEOP TEST] SHOT 1 FIRED!");
                }
            }),
            
            Commands.waitSeconds(0.5),
            
            // Fire shot 2
            Commands.runOnce(() -> {
                if (m_shooter.isSafeToFeed()) {
                    m_shooter.notifyShotStarted();
                    System.out.println("[TELEOP TEST] SHOT 2 FIRED!");
                }
            }),
            
            Commands.waitSeconds(0.5),
            
            // Fire shot 3
            Commands.runOnce(() -> {
                if (m_shooter.isSafeToFeed()) {
                    m_shooter.notifyShotStarted();
                    System.out.println("[TELEOP TEST] SHOT 3 FIRED!");
                }
            }),
            
            Commands.waitSeconds(1.0),
            
            // Toggle to LEFT
            Commands.runOnce(() -> {
                System.out.println("[TELEOP TEST] Toggling to LEFT...");
                m_shooter.toggleShootDirection();
            }),
            
            Commands.waitSeconds(1.0),
            
            // Fire shot 4 (LEFT)
            Commands.runOnce(() -> {
                System.out.println("[TELEOP TEST] State: " + m_shooter.getShooterState());
                if (m_shooter.isSafeToFeed()) {
                    m_shooter.notifyShotStarted();
                    System.out.println("[TELEOP TEST] SHOT 4 FIRED (LEFT)!");
                }
            }),
            
            Commands.waitSeconds(1.0),
            
            // Done
            Commands.runOnce(() -> {
                m_shooter.disableShooter();
                System.out.println("\n========================================");
                System.out.println("  TELEOP TEST COMPLETE!");
                System.out.println("  Shooter disabled.");
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
}
