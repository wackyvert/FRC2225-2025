package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static final double UPPER_LIMIT = 100.0;
    private static final double LOWER_LIMIT = 0.0;
    private final SparkFlex elevatorMotor;
    private final PIDController pidController;
    private double elevatorPosition;

    @Override
    public void periodic() {
        elevatorPosition=elevatorMotor.getEncoder().getPosition();
    }

    public ElevatorSubsystem(int elevatorMotorCanId) {
        elevatorMotor = new SparkFlex(elevatorMotorCanId, SparkLowLevel.MotorType.kBrushless);

        pidController = new PIDController(0.1, 0.0, 0.0); // Just an example, please tune for your system.
        pidController.setTolerance(5); // Just an example, please adjust according to your lift's specifics.
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

    public void raiseElevator() {
        double output = pidController.calculate(getElevatorPosition(), UPPER_LIMIT);
        elevatorMotor.set(output);
    }

    public void lowerElevator() {
        double output = pidController.calculate(getElevatorPosition(), LOWER_LIMIT);
        elevatorMotor.set(output);
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    public boolean atUpperLimit() {
        return Math.abs(getElevatorPosition() - UPPER_LIMIT) < pidController.getErrorTolerance();
    }

    public boolean atLowerLimit() {
        return Math.abs(getElevatorPosition() - LOWER_LIMIT) < pidController.getErrorTolerance();
    }
}
