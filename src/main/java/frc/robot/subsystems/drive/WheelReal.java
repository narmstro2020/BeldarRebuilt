// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

public class WheelReal extends Wheel {

    private static final class Constants {
        private static final double kS = 0.10045;
        // TODO: create a MotorType called motorType and initialize to MotorType.kBrushless
    }

    // TODO: create a field of type CANSparkMax called canSparkMax
    
    public WheelReal(int moduleNumber) {
        super(Constants.kS);
        // TODO: initialize canSparkMax a deviceId of moduleNumber + 20 and appropiate constants
    }

    @Override
    public double getPositionMeters() {
        // TODO: create a double called meters and initialize with canSparkMax.getEncoder().getPosition() * 2 * Math.PI * Wheel.Constants.wheelRadiusMeters / Wheel.Constants.gearing;
        // TODO: return meters
        return 0.0; // TODO: remove this line when done
    }

    @Override
    public double getVelocityMetersPerSecond() {
        // TODO: create a double called metersPerSecond and initialize with canSparkMax.getEncoder().getVelocity() * 2 * Math.PI * Wheel.Constants.wheelRadiusMeters / 60 / Wheel.Constants.gearing;
        // TODO: return metersPerSecond
        return 0.0; // TODO: remove this line when done
    }

    @Override
    public void setPositionMeters(double meters) {
        // TODO: create a double called rotations and initialize with meters * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters
        // TODO: setPosition of canSparkMax's encoder to rotations
    }

    @Override
    public void setInputVoltage(double voltage) {
        // TODO: setVoltage of the canSparkMax to voltage
    }
}
