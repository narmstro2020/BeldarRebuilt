package frc.com.sensors;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteAngle {

    public static final Supplier<Rotation2d> createAbsoluteAngleSupplier(
        CANCoder cancoder){
            return () -> {
                double angleDegrees = cancoder.getAbsolutePosition();
                angleDegrees = angleDegrees < 0 ? angleDegrees + 360 : angleDegrees;
                return Rotation2d.fromDegrees(angleDegrees);
            };       
        }
    
    public static final Supplier<Rotation2d> createAbsoluteAngleSupplier(
        Supplier<Double> angleDegreesSupplier
    ){
        return () -> {
            //TODO: create a double called angleDegrees and initialize it to the angleDegreesSupplier's get method
            //TODO: make sure the angle is between 0 and 360 degrees like I did for the cancoder example
            //TODO: return the Rotation2d object from angleDegrees
            return null;  //TODO:  remove this line when done
        };
    }
    
}
