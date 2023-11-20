package frc.robot.subsystems.drive;

public class SteerSim extends Steer {

    private static final class Constants {
        private static final double kS = 0.0;
        // TODO: create an int numMotors to 1
        // TODO: create a DCMotor called dcMotor and initialize to DCMotor.getNEO(numMotors);
    }

    // TODO: create a SimDouble field called simRotations
    // TODO: create a SimDouble field called simRPM
    // TODO: create a SimDouble field called simCurrent
    // TODO: create a SimDouble field called simVolts
    // TODO: create a ModuleSteerSim field called steerSim

    public SteerSim(int moduleNumber) {
        super(Constants.kS);
        // TODO: initialize steerSim with appropriate constants
        // TODO: create a SimDevice called simDevice and initialize to SimDevice.create("NEO", moduleNumber + 10);
        // TODO: initialize simRotations to simDevice.createDouble("Rotations", Direction.kBidir, 0.0);
        // TODO: initialize simRPM to simDevice.createDouble("RPM", Direction.kBidir, 0.0);
        // TODO: initialize simCurrent to simDevice.createDouble("Amps", Direction.kBidir, 0.0);
        // TODO: initialize simVolts to simDevice.createDouble("Volts", Direction.kBidir, 0.0);
    }

    @Override
    public double getPositionDegrees() {
        // TODO: return getPositionDegrees() from steerSim
        return 0.0; // TODO: remove this line when done       
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        // TODO: return getVelocityDegreesPerSecond() from steerSim
        return 0.0; // TODO: remove this line when done
    }

    @Override
    public void setPositionDegrees(double degrees) {
        // TODO: setPositionDegrees for steerSim
    }

    @Override
    public void setInputVoltage(double voltage) {
        // TODO: set simVolts to voltage
        // TODO: set simRotations to steerSim.getPositionRadians() * Constants.gearing / 2 / Math.PI)
        // TODO: set simRPM to steerSim.getVelocityRadiansPerSecond() * 60 * Constants.gearing / 2 / Math.PI)
        // TODO: set simCurrent to steerSim.getCurrentDrawAmps())
        // TODO: setInputVoltage for steerSim
        // TODO: update steerSim with dtSeconds
    }
}
