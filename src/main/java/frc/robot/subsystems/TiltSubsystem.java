package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TiltSubsystem extends SubsystemBase {

  public static enum State{
    NONE,
    SHORT,
    LONG,
    FULL
  }
  
  private static final class Constants{
    
  }

  //fields
  //constructor
  public TiltSubsystem() {

  }

  //telemetry methods
  public State getState(){
    return null; //TODO
  }

  //control methods
  public void setState(State state){
    //TODO
  }

  //command creation methods
  public Command createSetStateCommand(State state){
    return null; //TODO
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
