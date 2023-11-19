// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

public final class ArmSubsystemReal extends ArmSubsystem {

        private static final class Constants {
                private static final double kS = 0.057659;
                // TODO: create a MotorType called motorType and set to MotorType.kBrushless
        }

        // TODO: create a field of type CANSparkMax called canSparkMax
        public ArmSubsystemReal() {
                super(Constants.kS);
                // TODO: initialize canSparkMax appropriately
        }

        @Override
        public double getAngleRads() {
                // TODO: return canSparkMax.getEncoder().getPosition() * 2 * pi / gearing
                return 0.0; // TODO: remove this line when finished

        }

        @Override
        public double getVelocityRadPerSec() {
                // TODO: return canSparkMax.getEncoder().getVelocity() * 2 * pi / 60 / gearing
                return 0.0; // TODO: remove this line when finished
        }

        @Override
        public void setAngleRads(double radians) {
                // TODO: setPosition using canSparkMax.getEncoder() to the following value
                // radians * gearing / 2 / pi
        }

        @Override
        public void setInputVoltage(double voltage) {
                // TODO: use the canSparkMax's setVoltage method set the voltage
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                super.periodic();
        }
}
