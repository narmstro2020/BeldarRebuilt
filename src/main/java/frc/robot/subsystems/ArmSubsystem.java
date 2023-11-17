package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private static final class Constants{

  }

  //fields
  //constructor
  public ArmSubsystem() {

  }

  //telemetry methods
  public double getPositionRadians(){
    return 0.0; //TODO
  }

  public double getVelocityRadiansPerSecond(){
   return 0.0; //TODO 
  }

  public void setPosition(double radians){
    //TODO
  }

  //control methods
  public void turnToPosition(double degrees){
    //TODO
  }

  public void driveAtVelocity(double degreesPerSecond){
    //TODO
  }

  //command creation methods
  public Command createTurnToPositionCommand(double degrees){
    return null; //TODO
  }

  public Command createDriveAtVelocityCommand(double degreesPerSecond){
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
