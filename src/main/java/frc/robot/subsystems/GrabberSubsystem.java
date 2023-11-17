package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

  private static final class Constants{

  }

  //fields
  //constructor
  public GrabberSubsystem() {

  }

  //telemetry methods
  public double getVelocityMetersPerSecond(){
   return 0.0; //TODO 
  }

  //control methods
  public void driveAtVelocity(double metersPerSecond){
    //TODO
  }

  //command creation methods
  public Command createDriveAtVelocityCommand(double metersPerSecond){
    return null; //TODO
  }

  public void setDefaultCommand(){
    //TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //initSendable handles Dashboard
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
