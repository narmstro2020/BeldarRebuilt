package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class GrabberSubsystemReal extends GrabberSubsystem {

  private static final class Constants {
    private static final double kS = 0.23971;
    MotorType motorType = MotorType.kBrushless;
  }

  // fields
  private CANSparkMax leftGripperCanSparkMax;
  private CANSparkMax rightGripperCanSparkMax;

  // constructor
  public GrabberSubsystemReal() {
    super(Constants.kS);
    CANSparkMax leftGripperCanSparkMax = new CANSparkMax(1,MotorType.kBrushless);
    CANSparkMax rightGripperCanSparkMax = new CANSparkMax(1,MotorType.kBrushless);
  }

  // telemetry methods
  @Override
  public double getLeftGripperVelocityRPM() {
    return leftGripperCanSparkMax.getEncoder().getVelocity() * GrabberSubsystem.Constants.gearing;
  }

  @Override
  public double getLeftGripperVelocityRadPerSec() {
    return getLeftGripperVelocityRPM() / 60;
    // TODO:  Use the Units class to convert RPM from getLeftGripperVelocityRPM() to RadPerSec
  }

  @Override
  public double getRightGripperVelocityRPM() {
    return rightGripperCanSparkMax.getEncoder().getVelocity() * GrabberSubsystem.Constants.gearing;
  }

  @Override
  public double getRightGripperVelocityRadPerSec() {
    return getLeftGripperVelocityRPM() / 60;
    // TODO:  Use the Units class to convert RPM from getRightGripperVelocityRPM() to RadPerSec
  }

  @Override
  public void setLeftGripperInputVoltage(double voltage) {
    
    // TODO: use the setVoltage Method for the leftCanSparkMax
    // leftGripperCanSparkMax(voltage);
  }

  @Override
  public void setRightGripperInputVoltage(double voltage) {
    // TODO: use the setVoltage Method for the rightCanSparkMax
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
