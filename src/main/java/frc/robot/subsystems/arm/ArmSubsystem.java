// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ArmSubsystem extends SubsystemBase {

        protected static final class Constants {
                double dtSeconds = 0.020;
                int deviceId = 15;
                double gearing = 64;
                double kG = 0.01839;
                double kV = 1.215936;
                double kA = 0.0368736;
                double maxPositionErrorRadians = 0.125;
                double maxVelocityErrorRadiansPerSec = 1;
                LinearSystem<N2, N1, N1> positionPlant = LinearSystemId.identifyPositionSystem(kV, kA);
                Vector<N2> positionQelms = VecBuilder.fill(maxPositionErrorRadians, maxVelocityErrorRadiansPerSec);
                Vector<N1> positionRelms = VecBuilder.fill(RobotController.getBatteryVoltage());
                LinearQuadraticRegulator<N2, N1, N1> positionController = new LinearQuadraticRegulator<>(
                        positionPlant, 
                        positionQelms, 
                        positionRelms, 
                        dtSeconds); 
                LinearSystem<N1, N1, N1> velocityPlant = LinearSystemId.identifyVelocitySystem(kV, kA);
                Vector<N1> velocityQelms = VecBuilder.fill(maxVelocityErrorRadiansPerSec);
                Vector<N1> velocityRelms = VecBuilder.fill(RobotController.getBatteryVoltage());
                LinearQuadraticRegulator<N1, N1, N1> velocityController = new LinearQuadraticRegulator<>(
                        velocityPlant,
                        velocityQelms,
                        velocityRelms, 
                        dtSeconds);
                double kPPosition = positionController.getK().get(0, 0);
                double kIPosition = 0.0;
                double kDPosition = positionController.getK().get(0, 1);
                double kPVelocity = velocityController.getK().get(0, 0);
                double kIVelocity = 0.0;
                double kDVelocity = 0.0;
                double tolerance = 0.0239 * 2 * Math.PI / gearing;
        }

        // TODO: create a field of type PIDController called positionPIDController
        // TODO: create a field of type PIDController called velocityPIDController
        // TODO: create a field of type TrapezoidProfile called trapezoidProfilePosition
        // TODO: create a field of type TrapezoidProfile called trapezoidProfileVelocity
        // TODO: create a field of type ArmFeedforward called armFeedforward

        // TODO: create a field of type double called lastVelocity

        public ArmSubsystem(double kS) {
                // TODO: initialize armFeedForward with appropriate constants
                // TODO: create a double called maxVelocity and initialize to
                // armFeedForward.maxachievableVelocity(12, 0)
                // TODO: create a double called maxAcceleration and intialize to
                // armFeedForward.maxAchievableAcceleration(12, 0)
                // TODO: create a Constraints object called positionConstraints and intialize
                // with
                // maxVelocity and maxAcceleration
                // TODO: create a Constraints object called velocityContraints and intialize
                // with maxAcceleration and Double.PositiveInfinity
                // TODO: initialize positionPIDController with appropriate constants
                // TODO: initialize velocityPIDController with appropriate constants
                // TODO: initialize trapezoidProfilePosition with appropriate constants
                // TODO: initialize trapezoidProfileVelocity with appropriate constants
                // TODO: set tolerance for positionPIDController

                // TODO: initialize lastVelocity to 0.0
        }

        public abstract double getAngleRads();

        public abstract double getVelocityRadPerSec();

        public double getAccelerationRadPerSecSquared() {
                // TODO: create a double called currentVelocity and initialize to
                // getVelocityRadPerSec()
                // TODO: return (currentVelocity - lastVelocity) / Constants.dtSeconds
                return 0.0; // TODO: remove this line when finished
        }

        public abstract void setAngleRads(double radians);

        public void turnToPosition(double degrees) {
                // TODO: create a double called measurement and initialize to getAngleRads()
                // TODO: create a double called measurementVelocity and initialize to
                // getVelocityRadPerSec()
                // TODO: create a State called current and initialize with measurement and
                // measurementVelocity
                // TODO: create a State called goal and initialize with Math.toRadians(degrees)
                // and 0.0 for the velocity
                // TODO: create a State called achievableSetpoint initialize with
                // trapezoidProfilePosition.calculate
                // TODO: create a double called feedbackVoltage and initialize to
                // positionPIDController.calculate
                // TODO: create a double called feedforwardVoltage and initialize from
                // armFeedforward.calculate using achievableSetpoint.position and
                // achievableSetpoint.velocity
                // TODO: create a double called voltage equal to the sume of the previous
                // voltages
                // TODO: voltage = MathUtil.clamp(voltage, -12.0, 12.0);
                // TODO: setInputVoltage to voltage
        }

        public void driveAtVelocity(double degreesPerSecond) {
                // TODO: create a double called measurementVelocity and initialize to
                // getVelocityRadPerSec()
                // TODO: create a double called setpoint and initialize to
                // Math.toRadians(degreesPerSecond)
                // TODO: create a State called current and initialize using measurementVelocity
                // and getAccelerationRadPerSecSquared()
                // TODO: create a State called goal and initialize to setpoint and 0.0 for the
                // velocity
                // TODO: create a State called achievableSetpoint and initialize to
                // trapezoidProfileVelocity(dtSeconds, goal, current)
                // TODO: create a double called feedbackVoltage and calculate with
                // velocityPIDController
                // TODO: create a double called feedforwardVoltage and calculate with
                // armFeedforward
                // TODO: create a double called voltage equal to the sume of the previous
                // voltages
                // TODO: voltage = MathUtil.clamp(voltage, -12.0, 12.0)
                // TODO: setInputVoltage to voltage
        }

        public abstract void setInputVoltage(double voltage);

        public Command createTurnToPositionCommand(double degrees) {
                // TODO: create a Runnable called resetControllers and set to () ->
                // positionPIDController.reset()3
                // TODO: create a Command called resetControllersCommand and initialize to
                // runOnce(resetControllers)
                // TODO: create a Runnable called turnToPosition and set to () ->
                // turnToPosition(degrees)
                // TODO: create a Command called turnToPositionCommand and initialize to
                // run(turnToPosition)
                // TODO: create a BooleanSupplier called endingCondition and set to () ->
                // positionPIDController.atSetpoint()
                // TODO: create a Runnable called endingCleanup and set to () ->
                // setInputVoltage(0.0)
                // TODO: create a Command called command and initialize to
                // resetControllersCommand
                // .andThen(turnToPositionCommand)
                // .until(endingCondition)
                // .finallyDo(endingCleanup);
                // TODO: setName for command to String.format("%s degrees Command", degrees)
                // TODO: return command
                return null; // TODO: remove this line when done
        }

        public Command createDriveAtVelocityCommand(double degreesPerSecond) {
                // TODO: create a Runnable called resetControllers and set to () ->
                // velocityPIDController.reset()
                // TODO: create a Command called resetControllersCommand and initialize to
                // runOnce(resetControllers)
                // TODO: create a Runnable called driveAtVelocity and set to () ->
                // driveAtVelocity(degreesPerSecond)
                // TODO: create a Command called driveAtVelocityCommand and initialize to
                // run(driveAtVelocity)
                // TODO: create a Runnable called endingCleanup and set to () ->
                // setInputVoltage(0.0)
                // TODO: create a Command called command and set to
                // resetControllersCommand
                // .andThen(driveAtVelocityCommand)
                // .finallyDo(endingCleanup)
                // TODO: setName for command to String.format("%s DPS Command",
                // degreesPerSecond)
                // TODO: return command
                return null; // TODO: remove this line when done
        }

        public void setDefaultCommand() {
                // TODO: create a Runnable called defaultRunnable and set to () ->
                // driveAtVelocity(0.0)
                // TODO: create a Command called defaultCommand and intialize to
                // runOnce(defaultRunnable)
                // TODO: setName for defaultCommand to String.format("Stop Command")
                // TODO: setDefaultCommand(defaultCommand);
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                // TODO: set lastVelocity to getVelocityRadPerSec()
        }

        // Method handles reporting to the smartdashboard
        @Override
        public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);
                builder.addDoubleProperty("Arm Position (Degrees)", () -> Math.toDegrees(getAngleRads()), null);
                builder.addDoubleProperty("Arm Speed (DPS)", () -> Math.toDegrees(getVelocityRadPerSec()), null);
                builder.addDoubleProperty("Arm Speed (DPS_SQ)", () -> Math.toDegrees(getAccelerationRadPerSecSquared()),
                                null);
        }
}
