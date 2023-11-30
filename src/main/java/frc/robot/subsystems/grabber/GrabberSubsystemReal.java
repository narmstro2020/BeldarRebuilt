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
    leftGripperCanSparkMax = new CANSparkMax(Constants.kS);





    // TODO: initialize leftGripperCanSparkMax with appropriate constants
    // TODO: initialize rightGripperCanSparkMax with appropriate constants
  }

  // telemetry methods
  @Override
  public double getLeftGripperVelocityRPM() {
    // TODO: use the getEncoder().getVelocity() method from the leftCanSparkMax and multiply by the gearing
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public double getLeftGripperVelocityRadPerSec() {
    // TODO:  Use the Units class to convert RPM from getLeftGripperVelocityRPM() to RadPerSec
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public double getRightGripperVelocityRPM() {
    // TODO: use the getEncoder().getVelocity() method from the rightCanSparkMax and multiply by the gearing
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public double getRightGripperVelocityRadPerSec() {
    // TODO:  Use the Units class to convert RPM from getRightGripperVelocityRPM() to RadPerSec
    return 0.0; // TODO: remove this line when done
  }

  @Override
  public void setLeftGripperInputVoltage(double voltage) {
    // TODO: use the setVoltage Method for the leftCanSparkMax
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
