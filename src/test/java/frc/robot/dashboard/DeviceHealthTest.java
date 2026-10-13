package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class DeviceHealthTest {

  @Test
  void allConnectedReadsOk() {
    assertEquals("OK", DeviceHealth.summarize(List.of("A (1)", "B (2)"), new boolean[] {true, true}));
  }

  @Test
  void listsOnlyDisconnectedDevicesInOrder() {
    assertEquals("A (1), C (3)", DeviceHealth.summarize(
        List.of("A (1)", "B (2)", "C (3)"), new boolean[] {false, true, false}));
  }

  @Test
  void noDevicesReadsOk() {
    assertEquals("OK", DeviceHealth.summarize(List.of(), new boolean[] {}));
  }
}
