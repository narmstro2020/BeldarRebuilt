package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.tilt.TiltSubsystem;

public class Manipulator {

    public static final class State {

        public State(

        ) {

        }
    }

    // TODO: ANGEL: create a private final field for clawSubsystem of type
    // ClawSubsystem
    private final TiltSubsystem tiltSubsystem; 

    // TODO: BRADEN: create a private final field for grabberSubsystem of type
    // GrabberSubsystem.

    public Manipulator() {
        // TODO: ANGEL: initialize clawSubsystem to new ClawSubsystem();
        // TODO: KEITH: initialize tiltSubsystem to new TiltSubsystem();
        tiltSubsystem = new TiltSubsystem();
        // TODO: BRADEN: initialize grabberSubsystem to new GrabberSubsystemReal if real
        // and GrabberSubsystemSim if simulated.
        // use RobotBase.isSimulation() in your if statement
    }

    public void addToDashboard() {
        // TODO: ANGEL: use the SmartDashboard.putData() method to put clawSubsystem to
        // the Dashboard
        // TODO: KEITH: use the SmartDashboard.putData() method to put tiltSubsystem to
        // the Dashboard
        // TODO: BRADEN: use the SmartDashboard.putData() method to put the
        // grabberSubsystem to the Dashboard.

    }

    public void setDefaultCommands() {
        // TODO: BRADEN: call grabberSubsystem's setDefaultCommand.  The one that takes no arguments that you made.  

    }

    public void addMechanism2dWidget() {
        // TODO:
    }

    public void addTestingCommandsToDashboard() {
        // TODO: ANGEL create a Command called testOPENCommand using clawSubsystem's
        // createSetStateCommand()
        // TODO: ANGEL create a Command called testCLOSEDCommand using clawSubsystem's
        // createSetStateCommand()

        // TODO: KEITH create a Command called testFULLcommand using tiltSubsystem's
        // createSetStateCommand()
        // TODO: KEITH create a Command called testLONGcommand using tiltSubsystem's
        // createSetStateCommand()
        // TODO: KEITH create a Command called testSHORTcommand using tiltSubsystem's
        // createSetStateCommand()
        // TODO: KEITH create a Command called testNONEcommand using tiltSubsystem's
        // createSetStateCommand()

        // TODO: BRADEN create a Command called testGrabber750RPMCommand using grabberSubsystem's
        // createDriveAtVelocityCommand()
        // TODO: BRADEN create a Command called testGrabberMinus650RPMCommand using grabberSubsystem's
        // createDriveAtVelocityCommand()



    }

    public void bindArmManualControlToController(
            CommandXboxController controller,
            XboxController.Axis axis,
            double deadband) {
        // TODO:

    }

    public void bindElevatorManualControlToController(
            CommandXboxController controller,
            XboxController.Axis axis,
            double deadband) {
        // TODO:

    }

    public void bindGrabberManualControlToController(
            CommandXboxController controller,
            double deadband) {
        // TODO:

    }

    public void bindTiltManualControlToControllerPOV(
            CommandXboxController controller) {
        // TODO: KEITH: create a Trigger called fullTrigger and initialize with
        // controller.povUp();
        // TODO: KEITH: create a Trigger called longTrigger and initialize with
        // controller.povUp();
        // TODO: KEITH: create a Trigger called shortTrigger and initialize with
        // controller.povRight();
        // TODO: KEITH: create a Trigger called NoneTrigger and initialize with
        // controller.povDown();
        // TODO: KEITH: create a Command called fullCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: create a Command called longCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: create a Command called shortCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: create a Command called noneCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: bind the fullTrigger to the fullCommand with fullTrigger's
        // onTrue method

    }

    public void bindClawManualControlToController(
            CommandXboxController controller,
            XboxController.Button button) {
        // TODO: ANGEL: create a Command called toggleCommand using clawSubsystems
        // createToggleCommand method
        // TODO: ANGEL: create a Trigger called trigger and initialize to
        // controller.button(button.value)
        // TODO: ANGEL: bind the trigger to the toggleCommand using trigger's onTrue
        // method.

    }

    public void bindManipulatorStateToController(
            CommandXboxController controller,
            XboxController.Button button) {
        // TODO:

    }

    public void addManipulatorStateToNamedCommands(
            String stateName,
            Manipulator.State state) {
        // TODO:

    }

}
