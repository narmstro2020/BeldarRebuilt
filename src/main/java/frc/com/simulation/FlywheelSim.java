// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.com.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated flywheel mechanism. */
public class FlywheelSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the flywheel.
  private final DCMotor m_gearbox;

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param kV      The velocity gain.
   * @param kA      The acceleration gain.
   * @param gearbox The type of and number of motors in the flywheel gearbox.
   * @param gearing The gearing of the flywheel (numbers greater than 1 represent
   *                reductions).
   */
  public FlywheelSim(double kV, double kA, DCMotor gearbox) {
    super(new LinearSystem<>(
        Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, -kV / kA),
        VecBuilder.fill(0, 1 / kA),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1())));
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param kV                 The velocity gain.
   * @param kA                 The acceleration gain.
   * @param gearbox            The type of and number of motors in the flywheel
   *                           gearbox.
   * @param gearing            The gearing of the flywheel (numbers greater than 1
   *                           represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public FlywheelSim(
      double kV,
      double kA,
      DCMotor gearbox,
      Matrix<N2, N1> measurementStdDevs) {
    super(new LinearSystem<>(
        Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, -kV / kA),
        VecBuilder.fill(0, 1 / kA),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1())), measurementStdDevs);
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param plant   The linear system that represents the flywheel. This system
   *                can be created with
   *                {@link edu.wpi.first.math.system.plant.LinearSystemId#createFlywheelSystem(DCMotor, double,
   *                double)}.
   * @param gearbox The type of and number of motors in the flywheel gearbox.
   * @param gearing The gearing of the flywheel (numbers greater than 1 represent
   *                reductions).
   */
  public FlywheelSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox) {
    super(plant);
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param plant              The linear system that represents the flywheel.
   * @param gearbox            The type of and number of motors in the flywheel
   *                           gearbox.
   * @param gearing            The gearing of the flywheel (numbers greater than 1
   *                           represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public FlywheelSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      Matrix<N2, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param gearbox          The type of and number of motors in the flywheel
   *                         gearbox.
   * @param gearing          The gearing of the flywheel (numbers greater than 1
   *                         represent reductions).
   * @param jKgMetersSquared The moment of inertia of the flywheel. If this is
   *                         unknown, use the
   *                         {@link #FlyWheelSim(LinearSystem, DCMotor, double, Matrix)}
   *                         constructor.
   */
  public FlywheelSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
    super(new LinearSystem<>(
        Matrix.mat(Nat.N2(), Nat.N2())
            .fill(
                0,
                1,
                0,
                -gearing
                    * gearing
                    * gearbox.KtNMPerAmp
                    / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * jKgMetersSquared)),
        VecBuilder.fill(0, gearing * gearbox.KtNMPerAmp / (gearbox.rOhms * jKgMetersSquared)),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1())));
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param gearbox            The type of and number of motors in the flywheel
   *                           gearbox.
   * @param gearing            The gearing of the flywheel (numbers greater than 1
   *                           represent reductions).
   * @param jKgMetersSquared   The moment of inertia of the flywheel. If this is
   *                           unknown, use the
   *                           {@link #FlyWheelSim(LinearSystem, DCMotor, double, Matrix)}
   *                           constructor.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public FlywheelSim(
      DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N2, N1> measurementStdDevs) {
    super(
        new LinearSystem<>(
            Matrix.mat(Nat.N2(), Nat.N2())
                .fill(
                    0,
                    1,
                    0,
                    -gearing
                        * gearing
                        * gearbox.KtNMPerAmp
                        / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * jKgMetersSquared)),
            VecBuilder.fill(0, gearing * gearbox.KtNMPerAmp / (gearbox.rOhms * jKgMetersSquared)),
            Matrix.eye(Nat.N2()),
            new Matrix<>(Nat.N2(), Nat.N1())),
        measurementStdDevs);
    m_gearbox = gearbox;
  }

  /**
   * Sets the flywheel's state.
   *
   * @param velocityRadPerSec The new velocity in radians per second.
   */
  public void setState(double velocityRadPerSec) {
    setState(VecBuilder.fill(getAngularPositionRad(), velocityRadPerSec));
  }

  /**
   * Returns the DC motor position.
   *
   * @return The DC motor position.
   */
  public double getAngularPositionRad() {
    return getOutput(0);
  }

  /**
   * Returns the DC motor position in rotations.
   *
   * @return The DC motor position in rotations.
   */
  public double getAngularPositionRotations() {
    return Units.radiansToRotations(getAngularPositionRad());
  }

  /**
   * Returns the flywheel velocity.
   *
   * @return The flywheel velocity.
   */
  public double getAngularVelocityRadPerSec() {
    return getOutput(1);
  }

  /**
   * Returns the flywheel velocity in RPM.
   *
   * @return The flywheel velocity in RPM.
   */
  public double getAngularVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getOutput(1));
  }

  /**
   * Returns the flywheel current draw.
   *
   * @return The flywheel current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is
    // spinning
    // 2x faster than the flywheel
    double kA = 1 / m_plant.getB().get(1, 0);
    double kV = -m_plant.getA().get(1, 1) * kA;
    double motorVelocityRadPerSec = getAngularVelocityRadPerSec() * kV * m_gearbox.KvRadPerSecPerVolt;
    var appliedVoltage = m_u.get(0, 0);

    return m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
        * Math.signum(appliedVoltage);
  }

  /**
   * Sets the input voltage for the flywheel.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }
}
