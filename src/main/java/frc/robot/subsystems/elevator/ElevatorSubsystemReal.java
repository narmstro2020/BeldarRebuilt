// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

public class ElevatorSubsystemReal extends ElevatorSubsystem {

        private static final class Constants {
                private static final double kS = 0.047211;
                // TODO: create a MotorType called motorType and set to MotorType.kBrushless
        }

        // TODO: create a field of type CANSparkMax called canSparkMax       
        public ElevatorSubsystemReal() {
                super(Constants.kS);
                // TODO: initialize canSparkMax appropriately
        }

        @Override
        public double getPositionMeters() {
                // TODO: return canSparkMax.getEncoder().getPosition()) * 2 * pi * drumRadius / gearing
                return 0.0; // TODO: remove this line when finished
        }

        @Override
        public double getVelocityMetersPerSecond() {
                // TODO: return canSparkMax.getEncoder().getVelocity() * 2 * pi * drumRadius / 60 / gearing
                return 0.0; // TODO: remove this line when finished
        }

        @Override
        public void setPositionMeters(double meters) {
                // TODO: setPosition using canSparkMax.getEncoder() to the following value
                // meters * gearing / 2 / pi / drumRadiusMeters
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
