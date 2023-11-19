// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

public final class ArmSubsystemSim extends ArmSubsystem {

        private static final class Constants {
                // TODO: create an int called numMotors and set to 1
                // TODO: create a DCMotor called dcMotor and initialize with DCMotor.getNEO
                private static final double kS = 0.0;
                // TODO: create a double called minAngleRads and initialize to
                // Math.toRadians(-38)
                // TODO: create a double called maxAngleRads and initialize to
                // Math.toRadians(80)
                // TODO: create a boolean called simulateGravity and initialize to true
                // TODO: create a double called startingAngleRads and initialize to 0.0
        }

        // TODO: create a SimDouble field called simRotations
        // TODO: create a SimDouble field called simRPM
        // TODO: create a SimDouble field called simCurrent
        // TODO: create a SimDouble field called simVolts
        // TODO: create an SingleJointedArmSim field called singleJointedArmSim

        public ArmSubsystemSim() {
                super(Constants.kS);
                // TODO: initialize singleJointedArmSim with appropriate constants
                // TODO: create a SimDevice object called simDevice and initialize with
                // SimDevice.create("NEO", ArmSubsystem.Constants.deviceId);
                // TODO: initialize simRotations with simDevice.createDouble("Rotations",
                // Direction.kBidir, 0.0);
                // TODO: initialize simRPM with simDevice.createDouble("RPM", Direction.kBidir,
                // 0.0);
                // TODO: initialize simCurrent with simDevice.createDouble("Amps",
                // Direction.kBidir, 0.0);
                // TODO: initialize simVolts with simDevice.createDouble("Volts",
                // Direction.kBidir, 0.0);
        }

        public double getAngleRads() {
                // TODO: return getAngleRads() from singleJointedArmSim
                return 0.0; // TODO: remove this line when done
        }

        public double getVelocityRadPerSec() {
                // TODO: return getVelocityRadPerSec() from singleJointedArmSim
                return 0.0; // TODO: remove this line when done
        }

        public void setAngleRads(double radians) {
                //TODO: setState for singleJointedArmSim to radians and getVelocityRadPerSec()
        }

        @Override
        public void setInputVoltage(double voltage) {
                // TODO: setInputVoltage for singleJointedArmSim using voltage
                // TODO: set simVolts to voltage
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                // TODO: update singleJointedArmSim with dtSeconds
                // TODO: set simRotations with getAngleRads() * gearing / 2 / pi
                // TODO: set simRPM with getVelocityRadPerSec() * gearing * 60 / 2 / pi / drumRadius
                // TODO: set simCurrent with getCurrentDrawAmps() from elevatorSim
        }
}
