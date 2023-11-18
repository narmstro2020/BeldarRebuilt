package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TiltSubsystem extends SubsystemBase {

  public enum State {
    FULL,
    LONG,
    SHORT,
    NONE
  }

  private static final class Constants {
    // TODO: create PneumaticsModuleType set equal to PneumaticsModuletype.REVPH
    // TODO: create integer for moduleNumber set equal to 17
    // TODO: create integer for shortChannel set equal to 2
    // TODO: create integer for longChannel set equal to 3
  }

  // fields
  // TODO: declare Solenoid variable for shortSolenoid
  // TODO: declare Solenoid variable for longSolenoid
  // constructor
  public TiltSubsystem() {
    // TODO: initialize shortSolenoid with appropriate constants
    // TODO: initialize longSolenoid with appropriate constants
  }

  // telemetry methods
  public State getState() {
    // TODO: if both the shortSolenoid and longSolenoid are true then return
    // State.FULL,
    // TODO: else if the shortSolenoid is true and the longSolenoid is false then
    // return
    // State.SHORT
    // TODO: else if the shortSolenoid is false and the longSolenoid is true then
    // return
    // State.LONG
    // TODO: else return State.NONE
    return null; // TODO: remove this line when done
  }

  // control methods
  public void setState(State state) {
    // TODO: if state equals State.FULL then set both solenoids to true
    // TODO: else if state equals State.LONG then set longSolenoid to true and
    // shortSolenoid to false
    // TODO: else if state equals State.SHORT then set longSolenoid to false and
    // shortSolenoid to true
    // TODO: else set them both to false
  }

  // command creation methods
  public Command createSetStateCommand(State state) {
    // TODO: create a Runnable called tiltSetCommandRunnable set equal to () ->
    // setState(state)
    // TODO: create a Command called tiltSetCommand set equal to
    // runOnce(tiltSetCommandRunnable)
    // TODO: setName for tiltSetCommand to "Tilt " + state.name()
    // TODO: return tiltSetCommand
    return null; // TODO: remove this line when done
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // initSendable handles Dashboard
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Tilt State", () -> getState().toString(), null);

  }
}
