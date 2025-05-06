package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {
    private static final double UPPER_LIMIT = -37.4;
    private static final double LOWER_LIMIT = 0;
    private static final double AUTO_DOWN = -5.309;
    private static final double AUTO_UP = -4;
    private final SparkFlex elevatorMotor;
    private final PIDController pidController;
    private final TalonSRX algaeIntake = new TalonSRX(16);
    private double elevatorPosition;
    private CommandJoystick driverJoystick = new CommandJoystick(0);
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
    public static final SparkFlexConfig elevatorMotorConfig= new SparkFlexConfig();
    
    public Elevator(int elevatorMotorCanId) {
        algaeIntake.setNeutralMode(NeutralMode.Brake);
        elevatorMotor = new SparkFlex(elevatorMotorCanId, SparkLowLevel.MotorType.kBrushless);
     
        elevatorMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        climberMotor= new SparkFlex(51, SparkLowLevel.MotorType.kBrushless);
        pidController = new PIDController(0.05, 0.0, 0.01); // Just an example, please tune for your system.
        pidController.setTolerance(.05); // Just an example, please adjust according to your lift's specifics.
    }
    public void lowerElevatorAuto(){
        double output = pidController.calculate(getElevatorPosition(), AUTO_DOWN);
        if(!bottomLimitSwitch.get()) {
            elevatorMotor.set(output);
        }
        else{
            elevatorMotor.stopMotor();
        }
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
        double output = pidController.calculate(getElevatorPosition(), LOWER_LIMIT);
        if(!bottomLimitSwitch.get()){
            elevatorMotor.set(output);}
            else{
                elevatorMotor.stopMotor();
            }
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }
    public void raiseElevatorAuto(){
        double output = pidController.calculate(getElevatorPosition(), UPPER_LIMIT);
        if(topLimitSwitch.get()) {
            elevatorMotor.set(output);
        }
         else{
             elevatorMotor.stopMotor();
        }
    }

    public void raiseClimber(double speed) {
        climberMotor.set(.8);
       // climberMotor.set(MathUtil.clamp(driverJoystick.getRawAxis(3), 0, .8));
  
    }
    public void stopClimber() {climberMotor.set(0);}
    public void lowerClimber(double speed){climberMotor.set(-.8);}
    public void raiseElevator() {
        double output = pidController.calculate(getElevatorPosition(), UPPER_LIMIT);
       if(topLimitSwitch.get()) {
           elevatorMotor.set(output);
       }
        else{
            elevatorMotor.stopMotor();
       }
    }
     public BooleanSupplier atUpperLimit() {
        return ()-> Math.abs(getElevatorPosition() - UPPER_LIMIT) < pidController.getErrorTolerance() || !topLimitSwitch.get();
    }
    public BooleanSupplier atAutoLimit() {
        return ()-> Math.abs(getElevatorPosition() - AUTO_UP) < pidController.getErrorTolerance() || !topLimitSwitch.get();
    }

    public BooleanSupplier atLowerLimit() {
        return ()-> Math.abs(getElevatorPosition() - LOWER_LIMIT) < pidController.getErrorTolerance() || bottomLimitSwitch.get();
    }
}
