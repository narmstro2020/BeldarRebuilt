// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    grabberSubsystem.setDefaultCommand();
    SmartDashboard.putData(grabberSubsystem);
    SmartDashboard.putData(grabberSubsystem.createDriveAtVelocityCommand(750));

    ElevatorSubsystem elevatorSubsystem = RobotBase.isSimulation()
        ? new ElevatorSubsystemSim()
        : new ElevatorSubsystemReal();
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.createDriveAtVelocityCommand(0.0));
    SmartDashboard.putData(elevatorSubsystem);
    SmartDashboard.putData(elevatorSubsystem.createTurnToPositionCommand(0.5));
    SmartDashboard.putData(elevatorSubsystem.createTurnToPositionCommand(0.25));
    SmartDashboard.putData(elevatorSubsystem.createTurnToPositionCommand(0.75));
    SmartDashboard.putData(elevatorSubsystem.createDriveAtVelocityCommand(0.50));
    SmartDashboard.putData(elevatorSubsystem.createDriveAtVelocityCommand(-0.50));
    SmartDashboard.putData(elevatorSubsystem.createDriveAtVelocityCommand(1.00));

    ArmSubsystem armSubsystem = RobotBase.isSimulation()
        ? new ArmSubsystemSim()
        : new ArmSubsystemReal();
    armSubsystem.setDefaultCommand(armSubsystem.createDriveAtVelocityCommand(0.0));
    SmartDashboard.putData(armSubsystem);
    SmartDashboard.putData(armSubsystem.createTurnToPositionCommand(-20));
    SmartDashboard.putData(armSubsystem.createTurnToPositionCommand(45));
    SmartDashboard.putData(armSubsystem.createTurnToPositionCommand(60));
    SmartDashboard.putData(armSubsystem.createDriveAtVelocityCommand(2000));
    SmartDashboard.putData(armSubsystem.createDriveAtVelocityCommand(1));
    SmartDashboard.putData(armSubsystem.createDriveAtVelocityCommand(-2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
