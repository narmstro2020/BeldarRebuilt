// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.com.simulation.ElevatorSim;
import frc.com.simulation.SingleJointedArmSim;

public final class ArmSubsystemSim extends ArmSubsystem {

        private static final class Constants {
                private static final int numMotors = 1;
                private static final DCMotor dcMotor = DCMotor.getNEO(numMotors);
                private static final double kS = 0.0;
                private static final double minAngleRads = Math.toRadians(-38);
                private static final double maxAngleRads = Math.toRadians(80);
                private static final boolean simulateGravity = true;
               private static final  double startingAngleRads = 0.0;
        }

        SimDouble simRotations;
        SimDouble simRPM;
        SimDouble simCurrent;
        SimDouble simVolts;
        SingleJointedArmSim singleJointedArmSim;

        public ArmSubsystemSim() {
                super(Constants.kS);
                singleJointedArmSim = new SingleJointedArmSim(
                        null, 
                        Constants.dcMotor, 
                        ArmSubsystem.Constants.gearing, 
                        Constants.minAngleRads, 
                        Constants.maxAngleRads, 
                        Constants.simulateGravity, 
                        Constants.startingAngleRads, 
                       ArmSubsystem.Constants.kG);
                SimDevice simDevice = SimDevice.create("NEO", ArmSubsystem.Constants.deviceId);
                simRotations = simDevice.createDouble("Rotations", Direction.kBidir, 0.0);
                simRPM = simDevice.createDouble("RPM", Direction.kBidir, 0.0);
                simCurrent = simDevice.createDouble("Amps", Direction.kBidir, 0.0);
                simVolts = simDevice.createDouble("Volts", Direction.kBidir, 0.0);
        }

        public double getAngleRads() {
                return singleJointedArmSim.getAngleRads();
        }

        public double getVelocityRadPerSec() {
                return singleJointedArmSim.getVelocityRadPerSec();
        }

        public void setAngleRads(double radians) {
                singleJointedArmSim.setState(radians, getVelocityRadPerSec());
        }

        @Override
        public void setInputVoltage(double voltage) {
                singleJointedArmSim.setInputVoltage(voltage);
                simVolts.set(voltage);
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                simRotations.set(getAngleRads() * ArmSubsystem.Constants.gearing / 2 / Math.PI);
                simRPM.set(getVelocityRadPerSec() * ArmSubsystem.Constants.gearing * 60 / 2 / Math.PI / ArmSubsystem.Constants.drumRadius);
                simCurrent.set(ElevatorSim.getCurrentDrawAmps());
        }
}
