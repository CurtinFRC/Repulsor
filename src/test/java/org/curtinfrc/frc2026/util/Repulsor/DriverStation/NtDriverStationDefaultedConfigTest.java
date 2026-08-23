package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class NtDriverStationDefaultedConfigTest {
  private NetworkTableInstance inst;

  private static final class BareNtRepulsorDriverStation extends NtRepulsorDriverStation {
    BareNtRepulsorDriverStation(NetworkTableInstance instance, String root) {
      super(instance, root);
    }

    @Override
    protected void declareSharedConfig(Schema schema) {}
  }

  private static final class DeclaringNtRepulsorDriverStation extends NtRepulsorDriverStation {
    DeclaringNtRepulsorDriverStation(NetworkTableInstance instance, String root) {
      super(instance, root);
    }

    @Override
    protected void declareSharedConfig(Schema schema) {
      schema.configBool("declared_bool", false);
      schema.configDouble("declared_double", 0.0);
    }
  }

  @BeforeEach
  void setUp() {
    inst = NetworkTableInstance.create();
  }

  @AfterEach
  void tearDown() {
    inst.close();
  }

  @Test
  void defaultedAccessorsReturnDefaultsWhenKeysUndeclared() {
    try (NtRepulsorDriverStation ds = new BareNtRepulsorDriverStation(inst, "/test/bare")) {
      assertTrue(ds.getConfigBool("missing", true));
      assertFalse(ds.getConfigBool("missing", false));
      assertEquals(3.5, ds.getConfigDouble("missing", 3.5));
      assertEquals(-1.0, ds.getConfigDouble("missing", -1.0));
    }
  }

  @Test
  void defaultedAccessorsReadDeclaredKeysInsteadOfDefault() {
    try (NtRepulsorDriverStation ds =
        new DeclaringNtRepulsorDriverStation(inst, "/test/declaring")) {
      assertFalse(ds.getConfigBool("declared_bool", true));
      assertEquals(0.0, ds.getConfigDouble("declared_double", 7.5));

      ds.setConfigBool("declared_bool", true);
      ds.setConfigDouble("declared_double", 9.0);

      assertTrue(ds.getConfigBool("declared_bool", false));
      assertEquals(9.0, ds.getConfigDouble("declared_double", -1.0));

      assertTrue(ds.getConfigBool("undeclared", true));
      assertEquals(7.5, ds.getConfigDouble("undeclared", 7.5));
    }
  }

  @Test
  void requiredAccessorsStillRejectUndeclaredKeys() {
    try (NtRepulsorDriverStation ds = new BareNtRepulsorDriverStation(inst, "/test/strict")) {
      assertThrows(IllegalStateException.class, () -> ds.getConfigBool("nope"));
      assertThrows(IllegalStateException.class, () -> ds.getConfigDouble("nope"));
      assertThrows(IllegalStateException.class, () -> ds.setConfigBool("nope", true));
      assertThrows(IllegalStateException.class, () -> ds.setConfigDouble("nope", 1.0));
    }
  }

  @Test
  void declaredSchemaRemainsUsableThroughDefaultedAccessors() {
    try (NtRepulsorDriverStation ds = new DefaultNtRepulsorDriverStation(inst, "/test/default")) {
      assertFalse(ds.getConfigBool("force_controller_override", true));
      assertEquals(1.0, ds.getConfigDouble("clearance_scale", 0.0));
    }
  }
}
