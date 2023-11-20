package frc.robot.subsystems.grabber;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GrabberSubsystem extends SubsystemBase {

  protected static class Constants {
    protected static final double dtSeconds = 0.020;
    private final static int leftGripperDeviceId = 14;
    private final static int rightGripperDeviceId = 24;
    private final static double gearing = 12;
    private final static double kV = 0.121272;
    private final static double kA = 0.00251016;
    private final static double maxVelocityErrorRadPerSec = 46.3;

    // private final LinearSystem<N1, N1, N1> plant, initalize

    // TODO: create a LinearSystem<N1, N1, N1> called plant and initialize to
    // LinearSystemId.identifyVelocitySystem
    // TODO: create a Vector<N1> called qelms and initialize to
    // VecBuilder.fill(maxVelocityErrorRadPerSec)
    // TODO: create a Vector<N1> called relms and initialize to
    // VecBuilder.fill(RobotController.getBatteryVoltage())
    // TODO: create a LinearQuadraticRegulator<N1, N1, N1> called velocityController
    double kPVelocity = velocityController.getK().get(0, 0);
    double KIVelocity = 0.0;
    double KDVelocity = 0.0;
  }

  // fields
  // TODO: declare a TrapezoidProfile variable for leftTrapezoidProfile
  // TODO: declare a TrapezoidProfile variable for rightTrapezoidProfile
  // TODO: declare a PIDController variable for leftVelocityPIDController
  // TODO: declare a PIDController variable for rightVelocityPIDController
  // TODO: declare a SimpleMotorFeedforward variable for
  // leftSimpleMotorFeedforward
  // TODO: declare a SimpleMotorFeedforward varialbe for
  // rightSimpleMotorFeedForward
  // TODO: declare a variable called lastVelocityLeftGripper
  // TODO: declare a variable called lastVelocityRightGripper

  // constructor
  public GrabberSubsystem(double kS) {
    // TODO: initialize leftSimpleMotorFeedForward with appropriate constants
    // TODO: intialize rightSimpleMotorFeedForward with approrpiate constants
    // TODO: initialize leftTrapezoidProfile with appropriate constants
    // TODO: intialize rightTrapezoidProfile with appropriate constants
    // TODO: initialize leftVelocityPIDController with appropriate constants
    // TODO: initailize rightVelocityPIDController with appropriate constants
    // TODO: initialize lastVelocityLeftGripper to 0.0
    // TODO: initialize lastVelocityRightGripper to 0.0
  }

  // telemetry methods
  public abstract double getLeftGripperVelocityRPM();

  public abstract double getLeftGripperVelocityRadPerSec();

  public abstract double getRightGripperVelocityRPM();

  public abstract double getRightGripperVelocityRadPerSec();

  public double getLeftGripperAccelerationRadPerSecondSquared() {
    // TODO: create a double called currentVelocity and get from leftGripperSim in
    // radpersec
    return (currentVelocity - lastVelocityLeftGripper) / 0.020
  }

  public double getRightGripperAccelerationRadPerSecondSquared() {
    // TODO: create a double called currentVelocity and get from rightGripperSim in
    // radpersec
    return (currentVelocity - lastVelocityRightGripper) / 0.020
  }

  // control methods
  public void driveLeftGripperAtVelocity(double rpm) {
    double measurementVelocity = getLeftGripperVelocityRadPerSec();
    double setpoint = Units.rotationsPerMinutetoRadiansPerSecond(rpm);
    // TODO: create a State called current and use measurementVelocity for position
    // and getLeftGripperAcceleration for velocity
    // TODO: create a State called goal and use setpoint for position and 0.0 for
    // velocity
    // TODO: create a State called achievableSetpoint and calculate from
    // leftTrapezoidProfile.
    // TODO: create a double called feedbackVoltage enad calculate from
    // leftVelocityPIDController using measurement Velocity and
    // achievableSetpoint.position
    // TODO: create a double called feedforwardVoltage and get from
    // leftSimpleMotorFeedForward using measurementVelocity,
    // achievableSetpoint.position, and dtSeconds
    // TODO: create a double called voltage and add two previous voltages
    // TODO: set leftSimVolts to voltage
    // TODO: setInputVoltage on leftGripperSim
  }

  public void driveRightGripperAtVelocity(double rpm) {
    // TODO: same code as driveLeftGripperAtVelocity but for rightGripper
  }

  public abstract void setLeftGripperInputVoltage(double voltage);

  public abstract void setRightGripperInputVoltage(double voltage);

  // command creation methods
  public Command createDriveAtVelocityCommand(double rpm) {
    // TODO: create a Runnable called resetVelocityPIDControllersCommandRunnable and
    // reset both PIDControllers
    // TODO: create a runOnce command called resetVelocityPIDControllersCommand
    // using previous runnable
    // TODO: create a Runnable called driveGrabberAtVelocityCommandRunnable and set
    // left to rpm and right to -rpm
    // TODO: create a run command called driveGrabberAtVelocityCommand using
    // previous runnable
    // TODO: create a runnable called stopGrabberCommandRunnable that sets each
    // Gripper's inputVoltage to 0 and each simVolts to 0
    // TODO: create a Command called commmand set equal to
    // resetVelocityPIDControllesCommand.andThen(driveGrabberAtVelocityCommand).finallyDo(stopGrabberCommandRunnable)
    // TODO: setName of command to "Drive at " + rpm
    // TODO: return command
    return null; // TODO: remove this line when done
  }

  public void setDefaultCommand() {
    // TODO: this.setDefaultCommand(createDriveAtVelocityCommand(0.0))
  }

  @Override
  public void periodic() {
    // TODO: lastVelocityLeftGripper = getLeftGripperVelocityRadPerSec and also for right
  }

  // initSendable handles Dashboard
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Left Gripper Speed (RPM)", () -> getLeftGripperVelocityRPM(), null);
    builder.addDoubleProperty("Right Gripper Speed (RPM)", () -> getRightGripperVelocityRPM(), null);
    builder.addDoubleProperty("Left Gripper Acceleration (RadPerSecSquared)",
        () -> getLeftGripperAccelerationRadPerSecondSquared(), null);
    builder.addDoubleProperty("Right Gripper Acceleration (RadPerSecSquared)",
        () -> getRightGripperAccelerationRadPerSecondSquared(), null);
  }
}
