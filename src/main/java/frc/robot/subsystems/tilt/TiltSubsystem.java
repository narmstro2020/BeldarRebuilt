package frc.robot.subsystems.tilt;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
    private static final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH; 
    private static final int module = 17;
    private static final int shortChannel = 2;
    private static final int longChannel = 3;
  }

  // fields
  private final Solenoid shortSolenoid;
  private final Solenoid longSolenoid;
  // constructor
  public TiltSubsystem() {
    shortSolenoid = new Solenoid(Constants.module, Constants.moduleType, Constants.shortChannel);
    longSolenoid = new Solenoid(Constants.module, Constants.moduleType, Constants.shortChannel);
  }

  // telemetry methods
  public State getState() {
    if (shortSolenoid.get() && longSolenoid.get()) {
      return State.FULL;
  } else
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
