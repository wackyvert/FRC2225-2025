package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final TalonSRX algaeIntake = new TalonSRX(16);
    private double elevatorPosition;
    private DigitalInput topLimitSwitch = new DigitalInput(0);
    private DigitalInput bottomLimitSwitch = new DigitalInput(1);
   // private DigitalInput ballLimitSwitch = new DigitalInput(2);
    private SparkFlex climberMotor;
    @Override
    public void periodic() {
        elevatorPosition=elevatorMotor.getEncoder().getPosition();
        SmartDashboard.putNumber("Elevator", elevatorPosition);
        SmartDashboard.putBoolean("Top Limit Switch", topLimitSwitch.get());
        SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.get());
        //SmartDashboard.putBoolean("Ball Limit Switch", ballLimitSwitch.get());
    }

    public Elevator(int elevatorMotorCanId) {
        algaeIntake.setNeutralMode(NeutralMode.Brake);
        elevatorMotor = new SparkFlex(elevatorMotorCanId, SparkLowLevel.MotorType.kBrushless);
climberMotor= new SparkFlex(51, SparkLowLevel.MotorType.kBrushless);
        pidController = new PIDController(0.01, 0.0, 0.0); // Just an example, please tune for your system.
        pidController.setTolerance(.25); // Just an example, please adjust according to your lift's specifics.
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }
    public void stopAlgaeIntake(){
        algaeIntake.set(ControlMode.PercentOutput, 0);
    }
    public void spinAlgaeIntake(){
        algaeIntake.set(ControlMode.PercentOutput, -.4);
    }

    public void lowerElevator() {
        double output = pidController.calculate(getElevatorPosition(), UPPER_LIMIT);
        if(topLimitSwitch.get()){
            elevatorMotor.set(output);}
            else{
                elevatorMotor.stopMotor();
            }
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    public void raiseClimber() {
        climberMotor.set(0.4);
    }
    public void stopClimber() {climberMotor.set(0);}
    public void lowerClimber(){climberMotor.set(-.4);}

    public void raiseElevator() {
        double output = pidController.calculate(getElevatorPosition(), UPPER_LIMIT);
       if(!bottomLimitSwitch.get()) {
           elevatorMotor.set(output);
       }
        else{
            elevatorMotor.stopMotor();
       }
    }
     public BooleanSupplier atUpperLimit() {
        return ()-> Math.abs(getElevatorPosition() - UPPER_LIMIT) < pidController.getErrorTolerance() || !topLimitSwitch.get();
    }

    public BooleanSupplier atLowerLimit() {
        return ()-> Math.abs(getElevatorPosition() - LOWER_LIMIT) < pidController.getErrorTolerance() || bottomLimitSwitch.get();
    }
}
