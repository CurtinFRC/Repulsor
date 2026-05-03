/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;

/**
 * Provides reactive bypass config loader functionality for the Repulsor runtime helper layer shared
 * by behaviours and planners. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
final class ReactiveBypassConfigLoader {
  private ReactiveBypassConfigLoader() {}

  /**
   * Runs load config from yaml in the Repulsor runtime.
   *
   * @param cfg value used by this operation.
   * @param schemaClass value used by this operation.
   */
  static void loadConfigFromYaml(Object cfg, Class<?> schemaClass) {
    try {
      Path deployDir = resolveDeployDirectory();
      Path yamlPath = deployDir.resolve("ReactiveBypassConfig.yaml");
      if (!Files.exists(yamlPath)) return;
      List<String> lines = Files.readAllLines(yamlPath, StandardCharsets.UTF_8);
      for (String raw : lines) {
        String line = raw.trim();
        if (line.isEmpty()) continue;
        if (line.startsWith("#")) continue;
        int idx = line.indexOf(':');
        if (idx <= 0) continue;
        String key = line.substring(0, idx).trim();
        String valueStr = line.substring(idx + 1).trim();
        if (!valueStr.isEmpty()) applyConfigField(cfg, schemaClass, key, valueStr);
      }
    } catch (Throwable e) {
      System.err.println("ReactiveBypass: Failed to load config from YAML: " + e);
    }
  }

  private static Path resolveDeployDirectory() {
    String forced = System.getProperty("repulsor.deploy.dir");
    if (forced != null && !forced.isBlank()) {
      return Path.of(forced.trim());
    }

    String env = System.getenv("REPULSOR_DEPLOY_DIR");
    if (env != null && !env.isBlank()) {
      return Path.of(env.trim());
    }

    if (!OffloadExecutionContext.isWorkerOrLegacyThread()) {
      try {
        Class<?> fs = Class.forName("edu.wpi.first.wpilibj.Filesystem");
        Method getDeployDirectory = fs.getMethod("getDeployDirectory");
        Object dir = getDeployDirectory.invoke(null);
        if (dir instanceof File file) {
          return file.toPath();
        }
      } catch (Throwable ignored) {
      }
    }

    Path cwd = Path.of(System.getProperty("user.dir", "."));
    return cwd.resolve("deploy");
  }

  private static void applyConfigField(
      Object cfg, Class<?> schemaClass, String key, String valueStr) {
    try {
      Field f = schemaClass.getField(key);
      Class<?> t = f.getType();
      if (t == double.class) {
        double v = Double.parseDouble(valueStr);
        f.setDouble(cfg, v);
      } else if (t == int.class) {
        int v = Integer.parseInt(valueStr);
        f.setInt(cfg, v);
      } else if (t == boolean.class) {
        boolean v = Boolean.parseBoolean(valueStr);
        f.setBoolean(cfg, v);
      }
    } catch (NoSuchFieldException | IllegalAccessException | IllegalArgumentException ignored) {
    }
  }
}
