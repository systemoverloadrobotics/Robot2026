# TODO

Follow-ups from the `Merge branch 'pinnacles' into latest` merge (`89684e9`). That merge took
`Robot.java` from `pinnacles` but `RobotContainer.java` from `latest`, so some `pinnacles`
changes were lost.

## Autonomous

- [ ] **Confirm `shootPreloadLeft` is the left-trench shot.** No branch ever defined it in Java;
  commit `3ad596d` renamed the step in the auto files without registering it. It is now
  registered in `RobotContainer.java` using the left-trench hood angle and flywheel speed.
- [ ] **Check `right-single-swipe.auto`.** It's a right-side auto but calls `shootPreloadLeft`.
  It may need `shootRight` instead.
- [ ] **Fix the empty path step in `right-double-swipe.auto`.** The robot code logs
  "null.path (No such file or directory)" at startup. Pick the path in PathPlanner or delete the step.
- [ ] **Test all autos in the simulator.** The code builds, but the autos and the auto
  chooser haven't been run yet.
- [ ] **Decide whether to restore `wheelX`, `shootLeft` and `shootAutoMid`.** They exist on
  `pinnacles` but no auto uses them. Note: `pinnacles`' `wheelX` calls
  `drivetrain.applyRequest(...)` inside `runOnce`, which builds a command but never runs it.

## Robot.java

- [ ] **Restore the match timer?** `autonomousInit()` no longer calls
  `m_robotContainer.startMatchTimer()`.
## RobotContainer.java

- [ ] **Review the ~260 lines of `pinnacles` changes the merge dropped**, including button
  bindings and the intake stow button (`87d5ecb`). Compare with
  `git diff 89684e9^2 HEAD -- src/main/java/frc/robot/RobotContainer.java`.
- [ ] **Fix the D-pad down heading check.** `>= -60 || <= 60` is true at every heading, so
  outside CALIBRATION it always picks the left trench shot. It probably needs `&&`, and the
  right-side range needs the same fix.
- [ ] **Split the Back button's two actions.** In MANUAL, one press both locks the wheels and
  flips the driving direction.

## Other

- [ ] **Wire up the `Climb` subsystem.** Nothing creates it, and its motor ID and gains in
  `Constants.Climb` are still placeholder `-1` values.

## Driver

- [ ] **Confirm the driver doesn't need "vision distance, manual aim".** The AUTO shooter mode is
  being removed (option A): X aligns and B aligns and shoots, in every mode. Ask the driver whether
  they ever want the right trigger to shoot using the vision distance while aiming by hand. If yes,
  it can come back as a Y toggle for the distance source (option C).

## Lab measurements

- [ ] **Measure the hood angles (signed scale).** The code uses a signed scale: 0° is the hood
  centered over the flywheel, and the motor soft limits are ±90°. Today's presets are −88°
  (front shot) and +87° (back shot). Measure with the robot **disabled** (the hood motor is in
  coast, so it moves by hand) and the hopper empty.
  1. Connect Phoenix Tuner X to the robot and open **CANcoder 18** (hood). Plot or read its
     **Absolute Position**. Hood angle in degrees = Absolute Position × 360. (This assumes the
     CANcoder turns 1:1 with the hood, which is what the code assumes. If the CANcoder is not on
     the hood pivot shaft, note that instead.)
  2. **Zero:** move the hood by hand so it's centered over the flywheel. Record the angle. It
     should be close to 0°. If it's off by more than 1–2°, record the reading so the CANcoder
     offset (`MagnetOffset = 0.166` in `ShooterSubSystem`) can be corrected.
  3. **Signs:** tilt the hood toward the position for a front shot (fuel leaving the back-right
     end and flying toward the front). Record whether the angle goes negative or positive. Do the
     same for a back shot. The code expects front = negative, back = positive.
  4. **End stops:** push the hood gently to each mechanical end of travel and record both
     angles. Both must be inside ±90°; if an end stop is beyond ±90°, the soft limit will stop
     the motor first, so note it.
  5. **Usable range for each shot:** for each direction, find the angle where the fuel path
     first opens enough for a fuel to pass through the hood by hand (the smallest usable angle)
     and the largest angle before the hood hits its stop or rubs. Record a min and max for the
     front shot and for the back shot.
  6. Write the results in this table and in the team notebook:

     | | Front shot | Back shot |
     |---|---|---|
     | Sign (− / +) | | |
     | Usable min (°) | | |
     | Usable max (°) | | |
     | End stop (°) | | |
     | Zero reading when centered (°) | | |

  These replace the unused 0–180° range constants (`LEFT_HOOD_MIN_ANGLE` … `HOOD_ABSOLUTE_MAX`)
  in `Constants.Shooter`.

- [ ] **Bring up vision and shooting.** Follow the lab steps in `docs/shooter-vision-guide.html`
  (hood, flywheel direction, camera names and positions, launch point, field type), enter the results
  in `Constants.java` (`VisionConstants`, `ShotConstants`), then test X with a lab tag, then B, then
  build the shot table with CALIBRATION + Y.

## Done

- [x] Renamed `subsystems/climb.java` to `Climb.java` to match the class name.
- [x] Restored the auto chooser, `getAutonomousCommand()` and the `drivetrain.configAuto()` call.
- [x] Registered the `shootRight`, `shootPreloadLeft` and `stopShoot` named commands.
- [x] Vision + shooter integration (spec `docs/superpowers/specs/2026-10-08-vision-shooter-integration-design.md`):
  two-camera pose fusion, X align to hub, B align and shoot, shot table, calibration mode with Y to save rows.
- [x] Removed `PointToHub` (fixes its alliance-at-boot and constructor issues) and the old periodic shooter loop.
- [x] Fixed the "commands,s" comment typo in `Robot.robotPeriodic()`.
