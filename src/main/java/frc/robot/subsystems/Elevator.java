package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {
    private static final double UPPER_LIMIT = -37.4;
    private static final double LOWER_LIMIT = -0.9;
    private final SparkFlex elevatorMotor;
    private final PIDController pidController;
    private double elevatorPosition;

    @Override
    public void periodic() {
        elevatorPosition=elevatorMotor.getEncoder().getPosition();
        SmartDashboard.putNumber("Elevator", elevatorPosition);
    }

    public Elevator(int elevatorMotorCanId) {
        elevatorMotor = new SparkFlex(elevatorMotorCanId, SparkLowLevel.MotorType.kBrushless);

        pidController = new PIDController(0.01, 0.0, 0.0); // Just an example, please tune for your system.
        pidController.setTolerance(.25); // Just an example, please adjust according to your lift's specifics.
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }



    public void lowerElevator() {
       double output = pidController.calculate(getElevatorPosition(), LOWER_LIMIT);
        elevatorMotor.set(output);
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    public void raiseElevator() {
        double output = pidController.calculate(getElevatorPosition(), UPPER_LIMIT);
        elevatorMotor.set(output);
    }
    public Command raiseElevatorCommand(){
        return new InstantCommand(this::raiseElevator);
    }
    public BooleanSupplier atUpperLimit() {
        return ()-> Math.abs(getElevatorPosition() - UPPER_LIMIT) < pidController.getErrorTolerance();
    }
    public boolean atLowerLimit() {
        return Math.abs(getElevatorPosition() - LOWER_LIMIT) < pidController.getErrorTolerance();
    }
}
