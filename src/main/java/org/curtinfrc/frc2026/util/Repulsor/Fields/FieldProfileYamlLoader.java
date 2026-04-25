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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

public final class FieldProfileYamlLoader {
  private static final String PROFILE_PATH_PROPERTY = "repulsor.profile.path";
  private static final String PROFILE_DIR_PROPERTY = "repulsor.profile.dir";

  private FieldProfileYamlLoader() {}

  public static FieldProfileConfig loadOrDefault(String profileId, FieldProfileConfig defaults) {
    FieldProfileConfig fallback = defaults == null ? new FieldProfileConfig() : defaults;
    Path path = findProfilePath(profileId);
    if (path == null) {
      FieldProfileValidator.requireValid(fallback, "built-in " + profileId);
      return fallback;
    }

    try (InputStream in = Files.newInputStream(path)) {
      Yaml yaml = new Yaml(new Constructor(FieldProfileConfig.class, new LoaderOptions()));
      FieldProfileConfig cfg = yaml.load(in);
      if (cfg == null) {
        DriverStation.reportWarning("Repulsor profile YAML is empty: " + path, false);
        FieldProfileValidator.requireValid(fallback, "built-in " + profileId);
        return fallback;
      }
      FieldProfileConfig merged = FieldProfileConfig.merge(fallback, cfg);
      FieldProfileValidator.requireValid(merged, path.toString());
      return merged;
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to load Repulsor profile YAML "
              + path
              + "; using built-in defaults. "
              + ex.getMessage(),
          true);
      FieldProfileValidator.requireValid(fallback, "built-in " + profileId);
      return fallback;
    }
  }

  private static Path findProfilePath(String profileId) {
    String explicit = System.getProperty(PROFILE_PATH_PROPERTY);
    if (explicit != null && !explicit.isBlank()) {
      Path path = Path.of(explicit.trim());
      return Files.exists(path) ? path : null;
    }

    for (Path candidate : candidatePaths(profileId)) {
      if (Files.exists(candidate)) return candidate;
    }
    return null;
  }

  private static List<Path> candidatePaths(String profileId) {
    String fileName =
        profileId.endsWith(".yaml") || profileId.endsWith(".yml") ? profileId : profileId + ".yaml";
    List<Path> paths = new ArrayList<>();

    String profileDir = System.getProperty(PROFILE_DIR_PROPERTY);
    if (profileDir != null && !profileDir.isBlank()) {
      paths.add(Path.of(profileDir.trim()).resolve(fileName));
    }

    Path cwd = Path.of(System.getProperty("user.dir", "."));
    paths.add(cwd.resolve("src/main/deploy/repulsor/profiles").resolve(fileName));
    paths.add(cwd.resolve("deploy/repulsor/profiles").resolve(fileName));
    paths.add(cwd.resolve("repulsor/profiles").resolve(fileName));

    return paths;
  }
}
