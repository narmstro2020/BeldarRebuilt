// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

public class WheelSim extends Wheel {

    private static final class Constants {
        // TODO: create an int called numMotors and initialize to 1
        // TODO: create a DCMotor called dcMotor and initialize with DCMotor.getNEO
        private static final double kS = 0.0;
    }

    // TODO: create a SimDouble field called simRotations
    // TODO: create a SimDouble field called simRPM
    // TODO: create a SimDouble field called simCurrent
    // TODO: create a SimDouble field called simVolts
    // TODO: create a LinearWheelSim field called wheelSim

    public WheelSim(int moduleNumber) {
        super(Constants.kS);
        // TODO: initialize wheelSim
        // TODO: create a SimDevice called simDevice and initialize with SimDevice.create("NEO", moduleNumber + 20)
        // TODO: initialize simRotations with simDevice.createDouble("Rotations", Direction.kBidir, 0.0)
        // TODO: intialize simRPM with simDevice.createDouble("RPM", Direction.kBidir, 0.0)
        // TODO: initialize simCurrent with simDevice.createDouble("Amps", Direction.kBidir, 0.0)
        // TODO: initialize simVolts with simDevice.createDouble("Volts", Direction.kBidir, 0.0)
    }

    @Override
    public double getPositionMeters() {
        // TODO: return getPositonMeters() from wheelSim
        return 0.0; // TODO: remove this line when done.
    }

    @Override
    public double getVelocityMetersPerSecond() {
        // TODO: return getVelocityMetersPerSecond() from wheelSim
        return 0.0; // TODO: remove this line when done.
    }

    @Override
    public void setPositionMeters(double meters) {
        // TODO: setPositionMeters for wheelSim
    }

    @Override
    public void setInputVoltage(double voltage) {
        // TODO: set simVolts to voltage
        // TODO: setInputVoltage for wheelSim
        // TODO: set simRotations with wheelSim.getPositionMeters() * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters
        // TODO: set simRPM with wheelSim.getVelocityMetersPerSecond() * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters
        // TODO: set simCurrent with wheelSim.getCurrentDrawAmps()
        // TODO: update wheelSim with dtSeconds
    }
}
