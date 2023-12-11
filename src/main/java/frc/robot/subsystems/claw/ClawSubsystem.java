package frc.robot.subsystems.claw;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

  public static enum State {
    OPEN,
    CLOSED
  }

  private static final class Constants {
    // TODO: create PneumaticsModuleType set equal to PneumaticsModuletype.REVPH
    private static final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
    // TODO: create integer for moduleNumber set equal to 17
    private static final int module = 17;
    // TODO: create integer for leftChannel set equal to 0
    private static final int leftChannel = 0;
    // TODO: create integer for rightChannel set equal to 1
    private static final int rightChannel = 1;
  }

  // fields
  // TODO: declare Solenoid variable for leftSolenoid
  // TODO: declare Solenoid variable for rightSolenoid
  // constructor
  public ClawSubsystem() {
    // TODO: initialize leftSolenoid with appropriate constants
    // TODO: initialize rightSolenoid with appropriate constants
  }

  // telemetry methods
  public State getState() {
    // TODO: if both the leftSolenoid and rightSolenoid are true then return
    // State.OPEN,
    // TODO: else return State.CLOSED
    return null; // TODO: remove this line when done
  }

  // control methods used by commands
  public void setState(State state) {
    // TODO: if state equals State.OPEN then set both solenoids to true
    // TODO: else set them both to false
  }

  // command creation methods. Note this only makes a command according
  // to the instructions in the method. It does not connect it to a trigger.
  public static Command createSetStateCommand(State state) {
    // TODO: create a Runnable called clawSetCommandRunnable set equal to () ->
    // setState(state)
    // TODO: create a Command called clawSetCommand set equal to
    // runOnce(clawSetCommandRunnable)
    // TODO:  setName for clawSetCommand to "Claw " + state.name()
    // TODO: return clawSetCommand
    return null; // TODO: remove this line when done
  }

  public Command createToggleCommand(){
    // TODO: create a Command called setOpenCommand and initialize with createSetStateCommand
    // TODO: create a Command called setClosedCommand and initialize with createSetStateCommand
    // TODO: create a BooleanSupplier called selector and initialize with () -> getState() == State.Closed
    // TODO: create a Command called toggleCommand and initialize with Commands.either(setOpenCommand, setClosedCommand, selector);
    // TODO: use the setName() method from toggleCommand to "Toggle Command"
    // TODO: return toggleCommand;
    return null; // TODO: remove this line when done.  
  }

  // Method runs stuff on subsystem that must change all of the time.
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // initSendable handles Dashboard
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Claw State", () -> getState().toString(), null);

  }
}
