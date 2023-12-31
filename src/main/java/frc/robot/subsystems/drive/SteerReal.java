package frc.robot.subsystems.drive;

public class SteerReal extends Steer {

    private static final class Constants {
        private static final double kS = 0.12055;
        // TODO: create a MotorType called motortype and initialize to MotorType.kBrushless;
    }

    // TODO: create a field of type CANSparkMax called canSparkMax

    public SteerReal(int moduleNumber) {
        super(Constants.kS);
        // TODO: initialize canSparkMax with a deviceId of moduleNumber + 10 and the appropriate constants
    }

    @Override
    public double getPositionDegrees() {
        // TODO: create a double called degrees and initialize to canSparkMax.getEncoder().getPosition() * 360 / Steer.Constants.gearing
        // TODO: set degrees = MathUtil.inputModulus(degrees, 0, 360)
        // TODO: return degrees
        return 0.0;  // TODO: remove this line when done.  
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        // TODO: create a double called degreesPerSecond and initialize to canSparkMax.getEncoder().getVelocity() * 360 / 60 / Steer.Constants.gearing;
        // TODO: return degreesPerSecond
        return 0.0;  // TODO: remove this line when done. 
    }

    @Override
    public void setPositionDegrees(double degrees) {
        // TODO: create a double called degrees and initialize to MathUtil.inputModulus(degrees, 0, 360);
        // TODO: create a double called rotations and initialize to degrees * Steer.Constants.gearing / 360;
        // TODO: setPosition for the canSpark encoder to rotations
    }

    @Override
    public void setInputVoltage(double voltage) {
        //TODO: setVoltage for the canSparkMax to voltage
    }
}
