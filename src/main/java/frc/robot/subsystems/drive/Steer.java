package frc.robot.subsystems.drive;

public class Steer {
    private static final class Constants{
        //all variables here should have private static final before the type
    }

    //fields
    //constructor
    public Steer(){

    }

    //telemetry methods
    public double getPositionDegrees(){
        return 0.0; //TODO
    }

    public double getVelocityDegreesPerSecond(){
        return 0.0; //TODO
    }

    public void setPositionDegrees(){
        //TODO
    }

    //control methods
    public void turnToPosition(double degrees){
        //TODO
    }

    public double wrapSetpointAngle(double measurementRadians, double degrees){
        return 0.0; //TODO
    }

}
