package frc.robot.subsystems.grabber;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.proto.Plant;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class GrabberSubsystemSim extends GrabberSubsystem {

  private static final class Constants {
    private final static int numMotorsPerGripper = 1;
    private final static DCMotor dcmotor = DCMotor.getNeo550(numMotorsPerGripper);
    private static final double kS = 0.0;
  }

  // fields
  SimDouble leftSimRotations;
  SimDouble leftSimRPM;
  SimDouble leftSimCurrent;
  SimDouble leftSimVolts;
  SimDouble rightSimrotations;
  SimDouble rightSimRPM;
  SimDouble rightSimCurrent;
  SimDouble rightSimVolts;
  FlywheelSim leftGripperSim;
  FlywheelSim rightGripperSim;

  // constructor
  public GrabberSubsystemSim() {
    super(Constants.kS);
    SimDevice leftSimDevice = SimDevice.create("NEO550", 1);
    SimDevice rightSimDevice = SimDevice.create("NEO550", 2);
    var leftSimRotations = leftSimDevice.createDouble("Rotations",Direction.kBidir, 0.0);
    var leftSimRPM = leftSimDevice.createDouble("RPM", Direction.kBidir, 0.0);
    var leftSimCurrent = leftSimDevice.createDouble("Amps", Direction.kBidir, 0.0);
    var leftSimVolts = leftSimDevice.createDouble("volts", Direction.kBidir, 0.0);
    var rightSimRotations = rightSimDevice.createDouble("Rotations", Direction.kBidir, 0.0);
    var rightSimRPM = rightSimDevice.createDouble("RPM", Direction.kBidir , 0.0);
    var rightSimCurrent = rightSimDevice.createDouble("Amps", Direction.kBidir, 0.0);
    var rightSimVolts = rightSimDevice.createDouble("Volts", Direction.kBidir, 0.0);

    // leftGripperSim(Constants.dcmotor, GrabberSubsystem.Constants.gearing);
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
