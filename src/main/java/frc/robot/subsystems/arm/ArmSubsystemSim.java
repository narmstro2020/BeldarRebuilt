// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ArmSubsystemSim extends ArmSubsystem {

        private static final class Constants {
                int numMotors = 1;
                DCMotor dcMotor = DCMotor.getNEO(numMotors);
                private static final double kS = 0.0;
                double minAngleRads = Math.toRadians(-38);
                double maxAngleRads = Math.toRadians(80);
                boolean simulateGravity = true;
                double startingAngleRads = 0.0;
        }

        // TODO: create a SimDouble field called simRotations
        SimDouble simRotations;
        // TODO: create a SimDouble field called simRPM
        SimDouble simRPM;
        // TODO: create a SimDouble field called simCurrent
        SimDouble simCurrent;
        // TODO: create a SimDouble field called simVolts
        SimDouble simVolts;
        // TODO: create an SingleJointedArmSim field called singleJointedArmSim
        SingleJointedArmSim singleJointedArmSim;

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
