// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ArmSubsystem extends SubsystemBase {

        protected static final class Constants {
                protected static final double dtSeconds = 0.020;
                protected static final int deviceId = 15;
                protected static final double gearing = 64;
                protected static final double kG = 0.01839;
                protected static final double kV = 1.215936;
                protected static final double kA = 0.0368736;
                protected static final double maxPositionErrorRadians = 0.125;
                protected static final double maxVelocityErrorRadiansPerSec = 1;
                protected static final LinearSystem<N2, N1, N1> positionPlant = LinearSystemId.identifyPositionSystem(kV, kA);
                protected static final Vector<N2> positionQelms = VecBuilder.fill(maxPositionErrorRadians, maxVelocityErrorRadiansPerSec);
                protected static final Vector<N1> positionRelms = VecBuilder.fill(RobotController.getBatteryVoltage());
                protected static final LinearQuadraticRegulator<N2, N1, N1> positionController = new LinearQuadraticRegulator<>(
                        positionPlant, 
                        positionQelms, 
                        positionRelms, 
                        dtSeconds); 
                protected static final LinearSystem<N1, N1, N1> velocityPlant = LinearSystemId.identifyVelocitySystem(kV, kA);
                protected static final Vector<N1> velocityQelms = VecBuilder.fill(maxVelocityErrorRadiansPerSec);
                protected static final Vector<N1> velocityRelms = VecBuilder.fill(RobotController.getBatteryVoltage());
                protected static final LinearQuadraticRegulator<N1, N1, N1> velocityController = new LinearQuadraticRegulator<>(
                        velocityPlant,
                        velocityQelms,
                        velocityRelms, 
                        dtSeconds);
                protected static final double kPPosition = positionController.getK().get(0, 0);
                protected static final double kIPosition = 0.0;
                protected static final double kDPosition = positionController.getK().get(0, 1);
                protected static final double kPVelocity = velocityController.getK().get(0, 0);
                protected static final double kIVelocity = 0.0;
                protected static final double kDVelocity = 0.0;
                protected static final double tolerance = 0.0239 * 2 * Math.PI / gearing;
                public static int drumRadius;
        }

        private final PIDController positionPIDController;
        private final PIDController velocityPIDController;
        private final TrapezoidProfile trapezoidProfilePosition;
        private final TrapezoidProfile trapezoidProfileVelocity;
        private final ArmFeedforward armFeedforward;

        private double lastVelocity;

        public ArmSubsystem(double kS) {
                armFeedforward = new ArmFeedforward(kS, Constants.kG, Constants.kV, Constants.kA);
                double maxVelocity = armFeedforward.maxAchievableVelocity(12.0, Math.PI / 2, 0.0);
                double maxAcceleration = armFeedforward.maxAchievableAcceleration(12.0, Math.PI / 2, 0.0);
                Constraints positionConstraints = new Constraints(maxVelocity, maxAcceleration);
                Constraints velocityConstraints = new Constraints(maxAcceleration, Double.POSITIVE_INFINITY);
                positionPIDController = new PIDController(Constants.kPPosition, Constants.kIPosition, Constants.kDPosition);
                velocityPIDController = new PIDController(Constants.kPVelocity, Constants.kIVelocity, Constants.kDVelocity);
                trapezoidProfilePosition = new TrapezoidProfile(positionConstraints);
                trapezoidProfileVelocity = new TrapezoidProfile(positionConstraints);
                positionPIDController.setTolerance(maxAcceleration, maxVelocity);

                lastVelocity = 0.0;
        }

        public abstract double getAngleRads();

        public abstract double getVelocityRadPerSec();

        public double getAccelerationRadPerSecSquared() {
                double currentVelocity = getVelocityRadPerSec();
                return (currentVelocity - lastVelocity) / Constants.dtSeconds;
        }

        public abstract void setAngleRads(double radians);

        public void turnToPosition(double degrees) {
                double measurement = getAngleRads();
                double measurementVelocity = getVelocityRadPerSec();
                double radians = Math.toRadians(degrees);
                State current = new State(measurement, measurementVelocity);
                State goal =new State(radians, 0);
                State achievableSetpoint = trapezoidProfilePosition.calculate(Constants.dtSeconds, goal, current);
                double feedbackVoltage = positionPIDController.calculate(measurement, achievableSetpoint.position);
                double feedforwardVoltage = armFeedforward.calculate(achievableSetpoint.position, achievableSetpoint.velocity);
                double voltage = feedbackVoltage + feedforwardVoltage;
                voltage = MathUtil.clamp(voltage, -12.0, 12.0);
                setInputVoltage(voltage);
        }

        public void driveAtVelocity(double degreesPerSecond) {
                double measurementVelocity = getVelocityRadPerSec();
                double setpoint = Math.toRadians(degreesPerSecond);
                State current = new State(measurementVelocity, getAccelerationRadPerSecSquared());
                State goal = new State(setpoint, 0.0);
                State achievableSetpoint = trapezoidProfileVelocity.calculate(Constants.dtSeconds, goal, current);
                double feedbackVoltage = velocityPIDController.calculate(measurementVelocity, setpoint);
                double feedforwardVoltage = armFeedforward.calculate(measurementVelocity, setpoint);
                double voltage = feedbackVoltage + feedforwardVoltage;
                voltage = MathUtil.clamp(voltage, -12.0, 12.0);
                setInputVoltage(voltage);
        }

        public abstract void setInputVoltage(double voltage);

        public Command createTurnToPositionCommand(double degrees) {
                Runnable resetControllers = () -> positionPIDController.reset();
                Command resetControllersCommand = runOnce(resetControllers);
                Runnable turnToPosition = () -> turnToPosition(degrees);
                Command turnToPositionCommand = run(turnToPosition);
                BooleanSupplier endingCondition = () -> positionPIDController.atSetpoint();
                Runnable endingCleanup = () -> setInputVoltage(0.0);
                Command command = resetControllersCommand
                .andThen(turnToPositionCommand)
                .until(endingCondition)
                .finallyDo(endingCleanup);
                command.setName(String.format("%s degrees Command", degrees));
                return command;
        }

        public Command createDriveAtVelocityCommand(double degreesPerSecond) {
                Runnable resetControllers = () -> velocityPIDController.reset();
                Command resetControllersCommand = runOnce(resetControllers);
                Runnable drive = () -> driveAtVelocity(degreesPerSecond);
                Command driveAtVelocityCommand = run(drive);
                Runnable endingCleanup = () -> setInputVoltage(0.0);
                Command command = resetControllersCommand
                .andThen(driveAtVelocityCommand)
                .finallyDo(endingCleanup);
                command.setName(String.format("%s DPS Command",degreesPerSecond));
                return command;
        }

        public void setDefaultCommand() {
                Runnable defaultRunnable = () -> driveAtVelocity(0.0);
                Command defaultCommand = runOnce(defaultRunnable);
                defaultCommand.setName(String.format("Stop Command"));
                setDefaultCommand(defaultCommand);
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                lastVelocity = getVelocityRadPerSec();
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
