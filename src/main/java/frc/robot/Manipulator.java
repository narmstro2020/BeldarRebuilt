package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystemReal;
import frc.robot.subsystems.grabber.GrabberSubsystemSim;

public class Manipulator {

    public static final class State {

        public State(

        ) {

        }
    }

    private final GrabberSubsystem grabberSubsystem;

    public Manipulator() {
        // TODO: ANGEL: initialize clawSubsystem to new ClawSubsystem();
        // TODO: KEITH: initialize tiltSubsystem to new TiltSubsystem();

        if(RobotBase.isSimulation()){
            grabberSubsystem = new GrabberSubsystemSim();
        }else{
            grabberSubsystem = new GrabberSubsystemReal();
        };
    }

    public void addToDashboard() {
        // TODO: ANGEL: use the SmartDashboard.putData() method to put clawSubsystem to
        // the Dashboard
        // TODO: KEITH: use the SmartDashboard.putData() method to put tiltSubsystem to
        // the Dashboard
        SmartDashboard.putData(grabberSubsystem);

    }

    public void setDefaultCommands() {
        grabberSubsystem.setDefaultCommand();
    }

    public void addMechanism2dWidget() {
        // TODO:
    }

    public void addTestingCommandsToDashboard() {
        // TODO: ANGEL create a Command called testOPENCommand using clawSubsystem's
        // createSetStateCommand()
        // TODO: ANGEL create a Command called testCLOSEDCommand using clawSubsystem's
        // createSetStateCommand()

        Command testFULLCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.FULL);
        SmartDashboard.putData(testFULLCommand);    

        Command testLONGCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.LONG);
        SmartDashboard.putData(testLONGCommand);
        
        Command testSHORTCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.SHORT);
        SmartDashboard.putData(testSHORTCommand);
      
        Command testNONECommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.NONE);
        SmartDashboard.putData(testNONECommand);
        
        Command testGrabber750RPMCommand = grabberSubsystem.createDriveAtVelocityCommand(750);
        Command testGrabber650RPMCommand = grabberSubsystem.createDriveAtVelocityCommand(650);
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

    public void bindTiltManualControlToControllerPOV(CommandXboxController controller) {
        Trigger fullTrigger = controller.povUp();
        Trigger longTrigger = controller.povLeft();
        Trigger shortTrigger = controller.povRight();
        Trigger NoneTrigger = controller.povDown();

        Command fullCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.FULL);
        Command longCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.LONG);
        Command shortCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.SHORT);
        Command noneCommand = tiltSubsystem.createSetStateCommand(TiltSubsystem.State.NONE);

        fullTrigger.onTrue(fullCommand);
        longTrigger.onTrue(longCommand);
        shortTrigger.onTrue(shortCommand);
        NoneTrigger.onTrue(noneCommand);

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
