// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

public class ElevatorSubsystemSim extends ElevatorSubsystem {

        private static final class Constants {
                // TODO: create an int called numMotors and set to 1
                // TODO: create a DCMotor called dcMotor and initialize with DCMotor.getNEO
                private static final double kS = 0.0;
                // TODO: create a double called minHeightMeters and initialize to 0.0
                // TODO: create a double called maxHeightMeters and initialize to 1.0
                // TODO: create a boolean called simulateGravity and initialize to true
                // TODO: create a double called startingHeightMeter and initialize to 0.0
        }

        // TODO: create a SimDouble field called simRotations
        // TODO: create a SimDouble field called simRPM
        // TODO: create a SimDouble field called simCurrent
        // TODO: create a SimDouble field called simVolts
        // TODO: create an ElevatorSim field called elevatorSim

        public ElevatorSubsystemSim() {
                super(Constants.kS);
                // TODO: initialize elevatorSim with appropriate constants
                // TODO: create a SimDevice object called simDevice and initialize with
                // SimDevice.create("NEO", ElevatorSubsystem.Constants.deviceId);
                // TODO: initialize simRotations with simDevice.createDouble("Rotations",
                // Direction.kBidir, 0.0);
                // TODO: initialize simRPM with simDevice.createDouble("RPM", Direction.kBidir,
                // 0.0);
                // TODO: initialize simCurrent with simDevice.createDouble("Amps",
                // Direction.kBidir, 0.0);
                // TODO: initialize simVolts with simDevice.createDouble("Volts",
                // Direction.kBidir, 0.0);
        }

        @Override
        public double getPositionMeters() {
                // TODO: return getPositionMeters() from elevatorSim
                return 0.0; // TODO: remove this line when done
        }

        @Override
        public double getVelocityMetersPerSecond() {
                // TODO: return getVelocityMetersPerSecond() from elevatorSim
                return 0.0; // TODO: remove this line when done

        }

        @Override
        public void setPositionMeters(double meters) {
                //TODO: setState for elevatorSim to meters and getVelocityMetersPerSecond()
        }

        @Override
        public void setInputVoltage(double voltage) {
                // TODO: setInputVoltage for elevatorSim using voltage
                // TODO: set simVolts to voltage
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                // update elevatorSim with dtSeconds
                // set simRotations with getPosition() * gearing / 2 / pi / drumRadius
                // set simRPM with getVelocity() * gearing / 2 / pi / drumRadius
                // set simCurrent with getCurrentDrawAmps() from elevatorSim
        }
}
