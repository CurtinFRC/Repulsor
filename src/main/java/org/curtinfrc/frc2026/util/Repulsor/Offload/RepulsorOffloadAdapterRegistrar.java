package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

/**
 * Provides repulsor offload adapter registrar functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class RepulsorOffloadAdapterRegistrar implements OffloadAdapterRegistrar {
  /**
   * Updates register adapters state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   */
  @Override
  public void registerAdapters() {
    OffloadValueCodec.registerAdapter(
        "edu.wpi.first.math.geometry.Translation2d",
        Translation2dDTO.class,
        DragShotOffloadMapper::toTranslationDto,
        DragShotOffloadMapper::fromTranslationDto);

    OffloadValueCodec.registerAdapter(
        "edu.wpi.first.math.geometry.Pose2d",
        Pose2dDTO.class,
        DragShotOffloadMapper::toPoseDto,
        DragShotOffloadMapper::fromPoseDto);

    OffloadValueCodec.registerAdapter(
        "org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics",
        GamePiecePhysicsDTO.class,
        DragShotOffloadMapper::toGamePieceDto,
        DragShotOffloadMapper::fromGamePieceDto);

    OffloadValueCodec.registerAdapter(
        "org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints",
        ConstraintsDTO.class,
        DragShotOffloadMapper::toConstraintsDto,
        DragShotOffloadMapper::fromConstraintsDto);

    OffloadValueCodec.registerAdapter(
        "org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution",
        ShotSolutionDTO.class,
        DragShotOffloadMapper::toShotSolutionDto,
        DragShotOffloadMapper::fromShotSolutionDto);

    OffloadValueCodec.registerAdapter(
        "java.util.Optional<org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution>",
        DragShotAutoResponseDTO.class,
        DragShotOffloadMapper::toOptionalShotSolutionDto,
        DragShotOffloadMapper::fromOptionalShotSolutionDto);

    OffloadValueCodec.registerAdapter(
        "java.util.List<? extends org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle>",
        ObstacleListDTO.class,
        this::obstaclesToWire,
        this::obstaclesFromWire);
  }

  private ObstacleListDTO obstaclesToWire(List<? extends Obstacle> obstacles) {
    ObstacleListDTO wire = new ObstacleListDTO();
    wire.setObstacles(DragShotOffloadMapper.toObstacleDtos(obstacles));
    return wire;
  }

  private List<? extends Obstacle> obstaclesFromWire(ObstacleListDTO wire) {
    if (wire == null) {
      return List.of();
    }
    return DragShotOffloadMapper.fromObstacleDtos(wire.getObstacles());
  }

  /**
   * Provides obstacle list dto functionality for the Repulsor offload serialization and native/JNI
   * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating
   * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static class ObstacleListDTO {
    private List<ObstacleDTO> obstacles = List.of();

    /**
     * Returns the get obstacles value maintained by this Repulsor component.
     *
     * @return list of obstacle dto values produced by this operation.
     */
    public List<ObstacleDTO> getObstacles() {
      return obstacles;
    }

    /**
     * Updates set obstacles state or telemetry as part of the Repulsor runtime loop. This may
     * mutate local state, NetworkTables output, planner caches, or command-side runtime state
     * depending on the owning type.
     *
     * @param obstacles obstacle set used for safety checks, costs, or replanning.
     */
    public void setObstacles(List<ObstacleDTO> obstacles) {
      this.obstacles = obstacles;
    }
  }
}
