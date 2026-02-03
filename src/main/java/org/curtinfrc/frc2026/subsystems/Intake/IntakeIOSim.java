package org.curtinfrc.frc2026.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.FeedforwardEstimate;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.FeedforwardTestConfig;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.MechanismSimParams;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.SingleMotorRotarySim;

public class IntakeIOSim implements IntakeIO {
  private static final double DT = 0.02;
  private static final double GEAR_RATIO = 100.0;
  private static final double INERTIA = 0.02;

  private final SingleMotorRotarySim sim;
  private final PIDController velocityController;
  private final FeedforwardEstimate feedforward;

  private double appliedVoltage = 0.0;
  private double velocitySetpointRps = 0.0;
  private boolean closedLoop = false;

  public IntakeIOSim() {
    MechanismSimParams params =
        MechanismSimParams.rotary(DCMotor.getNEO(1), GEAR_RATIO, INERTIA).build();
    sim = new SingleMotorRotarySim(params);
    feedforward = sim.estimateFeedforward(FeedforwardTestConfig.builder().build());
    velocityController = new PIDController(Math.max(0.0, feedforward.getKV()), 0.0, 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      double setpointRadPerSec = Units.rotationsToRadians(velocitySetpointRps);
      double ffVolts =
          feedforward.getKS() * Math.signum(setpointRadPerSec)
              + feedforward.getKV() * setpointRadPerSec;
      double feedbackVolts =
          velocityController.calculate(sim.getVelocityRadPerSec(), setpointRadPerSec);
      appliedVoltage = MathUtil.clamp(ffVolts + feedbackVolts, -12.0, 12.0);
    }
    sim.setInputVoltage(appliedVoltage);
    sim.update(DT);

    inputs.AppliedVoltage = appliedVoltage;
    inputs.CurrentAmps = sim.getCurrentAmps();
    inputs.angularVelocity = sim.getVelocityRotationsPerSecond();
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(voltage, -12, 12);
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoop = true;
    velocitySetpointRps = velocity;
  }
}
