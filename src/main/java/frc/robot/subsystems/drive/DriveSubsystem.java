package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSubsystem extends SubsystemBase {

  private static final class Constants{

  }

  //fields
  //constructor
  public DriveSubsystem() {

  }

  //telemetry methods
  public Pose2d getPosition(){
    return null; //TODO
  }

  public ChassisSpeeds getVelocity(){
   return null; //TODO 
  }

  public void setPosition(Pose2d radians){
    //TODO
  }

  //control methods
  public void driveAtVelocity(ChassisSpeeds chassisSpeeds){
    //TODO
  }

  //command creation methods
  public Command createManualDriveCommand(CommandXboxController commandXboxController){
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
