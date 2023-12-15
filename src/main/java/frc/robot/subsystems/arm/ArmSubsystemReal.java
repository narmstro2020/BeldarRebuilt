// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public final class ArmSubsystemReal extends ArmSubsystem {

        private static final class Constants {
                private static final double kS = 0.057659;
                private static final MotorType motorType = MotorType.kBrushless;
        }


        private final CANSparkMax canSparkMax;
        public ArmSubsystemReal() {
                super(Constants.kS);
                canSparkMax = new CANSparkMax(ArmSubsystem.Constants.deviceId, Constants.motorType);
        }

        @Override
        public double getAngleRads() {
                return canSparkMax.getEncoder().getPosition() * 2 * Math.PI / ArmSubsystem.Constants.gearing;

        }

        @Override
        public double getVelocityRadPerSec() {
                return canSparkMax.getEncoder().getVelocity() * 2 * Math.PI / 60 / ArmSubsystem.Constants.gearing;
        }

        @Override
        public void setAngleRads(double radians) {
                setPosition(canSparkMax.getEncoder(radians * ArmSubsystem.Constants.gearing / 2 / Math.PI) );
        }

        @Override
        public void setInputVoltage(double voltage) {
                canSparkMax.setVoltage(voltage);
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                super.periodic();
        }
}
