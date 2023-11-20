// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.com.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated DC motor mechanism. */
public class ModuleSteerSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the DC motor.
  private final DCMotor m_gearbox;

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant   The linear system representing the DC motor. This system can
   *                be created with
   *                {@link edu.wpi.first.math.system.plant.LinearSystemId#createDCMotorSystem(DCMotor, double,
   *                double)}.
   * @param gearbox The type of and number of motors in the DC motor gearbox.
   * @param gearing The gearing of the DC motor (numbers greater than 1 represent
   *                reductions).
   */
  public ModuleSteerSim(double kV, double kA, DCMotor gearbox) {
    super(new LinearSystem<>(
        Matrix.mat(Nat.N2(), Nat.N2())
            .fill(
                0,
                1,
                0,
                -kV / kA),
        VecBuilder.fill(0, 1 / kA),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1())));
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant   The linear system representing the DC motor. This system can
   *                be created with
   *                {@link edu.wpi.first.math.system.plant.LinearSystemId#createDCMotorSystem(DCMotor, double,
   *                double)}.
   * @param gearbox The type of and number of motors in the DC motor gearbox.
   * @param gearing The gearing of the DC motor (numbers greater than 1 represent
   *                reductions).
   */
  public ModuleSteerSim(double kV, double kA, DCMotor gearbox, Matrix<N2, N1> measurementStdDevs) {
    super(new LinearSystem<>(
        Matrix.mat(Nat.N2(), Nat.N2())
            .fill(
                0,
                1,
                0,
                -kV / kA),
        VecBuilder.fill(0, 1 / kA),
        Matrix.eye(Nat.N2()),
        new Matrix<>(Nat.N2(), Nat.N1())), measurementStdDevs);
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant   The linear system representing the DC motor. This system can
   *                be created with
   *                {@link edu.wpi.first.math.system.plant.LinearSystemId#createDCMotorSystem(DCMotor, double,
   *                double)}.
   * @param gearbox The type of and number of motors in the DC motor gearbox.
   * @param gearing The gearing of the DC motor (numbers greater than 1 represent
   *                reductions).
   */
  public ModuleSteerSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
    super(plant);
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant              The linear system representing the DC motor. This
   *                           system can be created with
   * @param gearbox            The type of and number of motors in the DC motor
   *                           gearbox.
   * @param gearing            The gearing of the DC motor (numbers greater than 1
   *                           represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public ModuleSteerSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      Matrix<N2, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox          The type of and number of motors in the DC motor
   *                         gearbox.
   * @param gearing          The gearing of the DC motor (numbers greater than 1
   *                         represent reductions).
   * @param jKgMetersSquared The moment of inertia of the DC motor. If this is
   *                         unknown, use the
   *                         {@link #WheelSim(LinearSystem, DCMotor, double, Matrix)}
   *                         constructor.
   */
  public ModuleSteerSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
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
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox            The type of and number of motors in the DC motor
   *                           gearbox.
   * @param gearing            The gearing of the DC motor (numbers greater than 1
   *                           represent reductions).
   * @param jKgMetersSquared   The moment of inertia of the DC motor. If this is
   *                           unknown, use the
   *                           {@link #WheelSim(LinearSystem, DCMotor, double, Matrix)}
   *                           constructor.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public ModuleSteerSim(
      DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N2, N1> measurementStdDevs) {
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
        new Matrix<>(Nat.N2(), Nat.N1())), measurementStdDevs);
    m_gearbox = gearbox;
  }

  /**
   * Sets the state of the DC motor.
   *
   * @param angularPositionRad       The new position in radians.
   */
  public void setPositionRadians(double angularPositionRad) {
    setState(VecBuilder.fill(angularPositionRad, getVelocityRadiansPerSecond()));
  }

    /**
   * Sets the state of the DC motor.
   *
   * @param angularPositionDeg       The new position in degrees.
   */
  public void setPositionDegrees(double angularPositionDeg) {
    setState(VecBuilder.fill(Math.toRadians(angularPositionDeg), getVelocityRadiansPerSecond()));
  }

  /**
   * Returns the DC motor position in radians in range (-inf, inf).
   *
   * @return The DC motor position.
   */
  public double getPositionRadians() {
    return getOutput(0);
  }

  /**
   * Returns the DC motor position in degrees i nthe range [0, 360).
   *
   * @return The DC motor position.
   */
  public double getPositionDegrees() {
    return MathUtil.inputModulus(Math.toDegrees(getPositionRadians()), 0, 360);
  }

  /**
   * Returns the DC motor velocity in radians per second.
   *
   * @return The DC motor velocity.
   */
  public double getVelocityRadiansPerSecond() {
    return getOutput(1);
  }

  /**
   * Returns the DC motor velocity in degrees per second.
   *
   * @return The DC motor velocity.
   */
  public double getVelocityDegreesPerSecond() {
    return Math.toDegrees(getOutput(1));
  }

  /**
   * Returns the DC motor current draw.
   *
   * @return The DC motor current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is
    // spinning
    // 2x faster than the output
    double kA = 1 / m_plant.getB().get(1, 0);
    double kV = -m_plant.getA().get(1, 1) * kA;
    return m_gearbox.getCurrent(getVelocityRadiansPerSecond() * kV * m_gearbox.KvRadPerSecPerVolt, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the DC motor.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }
}
