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
            
            // Wait for ready (with timeout)
            Commands.waitUntil(() -> m_shooter.isReadyToShoot())
                .withTimeout(5.0),
            
            // Report status
            Commands.runOnce(() -> {
                if (m_shooter.isReadyToShoot()) {
                    System.out.println("[AUTO TEST] Shooter READY!");
                } else {
                    System.out.println("[AUTO TEST] Shooter NOT ready after 5 seconds");
                    System.out.println("            State: " + m_shooter.getShooterState());
                }
            }),
            
            // Simulate shot
            Commands.runOnce(() -> {
                if (m_shooter.isSafeToFeed()) {
                    m_shooter.notifyShotStarted();
                    System.out.println("[AUTO TEST] Shot fired!");
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
