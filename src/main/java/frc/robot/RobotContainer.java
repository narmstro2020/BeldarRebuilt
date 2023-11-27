// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemReal;
import frc.robot.subsystems.arm.ArmSubsystemSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystemReal;
import frc.robot.subsystems.elevator.ElevatorSubsystemSim;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystemReal;
import frc.robot.subsystems.grabber.GrabberSubsystemSim;

public class RobotContainer {

  private static final class Constants{

  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // TODO: Create ClawSubsystem called clawSubsystem
    // TODO: add the clawSubsystem to the dashboard using the SmartDashboard's
    // putData method.
    // TODO: create a command using createClawSetCommand for State.OPEN
    // TODO: create a command using createClawSetCommand for State.CLOSED
    // TODO: add each command to the dashboard using the SmartDashboard's putData
    // method. Note these are for testing

    // TODO: Create TiltSubsystem called tiltSubsystem
    // TODO: add the tiltSubsystem to the dashboard using the SmartDashboard's
    // putData method.
    // TODO: create a command using createTiltSetCommand for State.FULL
    // TODO: create a command using createTiltSetCommand for State.LONG
    // TODO: create a command using createTiltSetCommand for State.SHORT
    // TODO: create a command using createTiltSetCommand for State.NONE
    // TODO: add each command to the dashboard using the SmartDashboard's putData
    // method. Note these are for testing

    GrabberSubsystem grabberSubsystem = RobotBase.isSimulation()
        ? new GrabberSubsystemSim()
        : new GrabberSubsystemReal();
    // TODO: set grabberSubsystem's default command.  You have a method for this so use the one with no args
    // TODO: use the putData method for the SmartDashboard on the grabberSubsystems
    // TODO: use the putData method for the SmartDashboard to put a DriveAtVelocityCommand() with rpm set to 750

    
    ElevatorSubsystem elevatorSubsystem = RobotBase.isSimulation()
        ? new ElevatorSubsystemSim()
        : new ElevatorSubsystemReal();
    // TODO: set elevatorSubsystem's default command.  You have a method for this so use the one with no args
    // TODO: use the putData method for the SmartDashboard on the elevatorSubsystem
    // TODO: use the putData method to create a MoveToPositionCommand for 0.5 meters.  You'll also do this again for 0.25, 0.75
    // TODO: use the putData method to create a DriveAtVelocity for 0.50 metersPerSecond, also do for -0.5, and 1.0

    ArmSubsystem armSubsystem = RobotBase.isSimulation()
        ? new ArmSubsystemSim()
        : new ArmSubsystemReal();
    // TODO: set armSubsystem's default command.  You have a method for this so use the one with no args.  
    // TODO: use SmartDashboards putData method on the armSubsystem
    // TODO: use the putData method again to create a TurnToPositionCommand for -20, 45 and 60 degrees.  You'll use multiple times
    // TODO: use the putData method again to create a DriveAVelocityCommand for 1, -2, and 10 degreesPerSecond
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
