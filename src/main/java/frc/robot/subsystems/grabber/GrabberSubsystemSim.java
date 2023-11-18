package frc.robot.subsystems.grabber;

public class GrabberSubsystemSim extends GrabberSubsystem {

  private static final class Constants {
    // TODO: create an integer called numMotorsPerGripper and set equal to 1
    // TODO: create a DCMotor called dcMotor and initialize with DCMotor.getNeo550
    private static final double kS = 0.0;
  }

  // fields
  // TODO: declare a SimDouble variable for leftSimRotations
  // TODO: declare a SimDouble variable for leftSimRPM
  // TODO: declare a SimDouble variable for leftSimCurrent
  // TODO: declare a SimDouble variable for leftSimVolts
  // TODO: declare a SimDouble variable for rightSimRotations
  // TODO: declare a SimDouble variable for rightSimRPM
  // TODO: declare a SimDouble variable for rightSimCurrent
  // TODO: declare a SimDouble variable for rightSimVolts
  // TODO: declare a FlyWheelSim variable for leftGripperSim: import the one from
  // frc.robot
  // TODO: declare a FlyWheelSim variable for rightGripperSim

  // constructor
  public GrabberSubsystemSim() {
    super(Constants.kS);
    // TODO: create a SimDevice called leftSimDevice and set equal to
    // SimDevice.create("NEO550", Constants.leftGripperDeviceId)
    // TODO: create a SimDevice called rightSimDevice and set equal to
    // SimDevice.create("NEO550", Constants.leftGripperDeviceId)
    // TODO: initialize leftSimRotations to leftSimDevice.createDouble("Rotations",
    // Direction.kBidir, 0.0)
    // TODO: initialize leftSimRPM to leftSimDevice.createDouble("RPM",
    // Direction.kBidir, 0.0)
    // TODO: initialize leftSimCurrent to leftSimDevice.createDouble("Amps",
    // Direction.kBidir, 0.0)
    // TODO: initialize leftSimVolts to leftSimDevice.createDouble("Volts",
    // Direction.kBidir, 0.0)
    // TODO: initialize rightSimRotations to
    // rightSimDevice.createDouble("Rotations",
    // Direction.kBidir, 0.0)
    // TODO: initialize rightSimRPM to rightSimDevice.createDouble("RPM",
    // Direction.kBidir, 0.0)
    // TODO: initialize rightSimCurrent to rightSimDevice.createDouble("Amps",
    // Direction.kBidir, 0.0)
    // TODO: initialize rightSimVolts to rightSimDevice.createDouble("Volts",
    // Direction.kBidir, 0.0)
    // TODO: initialize leftGripperSim with appropriate constants
    // TODO: intialize rightGripperSim with appropriate constants
  }

  // telemetry methods
  @Override
  public double getLeftGripperVelocityRPM() {
    // TODO: use the getAngularVelocityRPM method from leftGripperSim to return the
    // RPM
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public double getLeftGripperVelocityRadPerSec() {
    // TODO: use the getAngularVelocityRadPerSec method from leftGripperSim to
    // return
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public double getRightGripperVelocityRPM() {
    // TODO: use the getAngularVelocityRPM method from rightGripperSim to return the
    // RPM
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public double getRightGripperVelocityRadPerSec() {
    // TODO: use the getAngularVelocityRadPerSec method from rightGripperSim to
    // return
    return 0.0; // TODO: remove this line when done
  }

  // control methods
  @Override
  public void setLeftGripperInputVoltage(double voltage) {
    // TODO: set leftSimVolts to voltage
    // TODO: setInputVoltage for leftGripperSim to voltage
  }

  @Override
  public void setRightGripperInputVoltage(double voltage) {
    // TODO: set rightSimVolts to voltage
    // TODO: setInputVoltage for rightGripperSim to voltage
  }

  @Override
  public void periodic() {
    // TODO: update leftGripperSim
    // TODO: update rightGripperSim
    // TODO: set all SimDoubles, rotations and velocities form methods (multiply by
    // gearing)
  }
}
