package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.LOWER_TARGET_POSITION;
import static frc.robot.Constants.UPPER_TARGET_POSITION;

import java.util.function.BooleanSupplier;


public class Intake extends SubsystemBase {

    SparkMax intakeMotor;
    TalonSRX drawbridgeMotor;
    PIDController pidController;


    double drawBridgePosition;
    public Intake(){

        drawbridgeMotor = new TalonSRX(Constants.DRAWBRIDGE_ID);
        drawbridgeMotor.setNeutralMode(NeutralMode.Brake);
        drawbridgeMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        intakeMotor = new SparkMax(Constants.INTAKE_ID, SparkLowLevel.MotorType.kBrushless);
        pidController = new PIDController(.001,0,0);
        drawbridgeMotor.setNeutralMode(NeutralMode.Brake);
        pidController.setTolerance(100);
    }
    @Override
    public void periodic(){
        //Get the position of the motor once per loop only, and store it in the global variable.
        drawBridgePosition = getMotorPosition();
        SmartDashboard.putNumber("Intake Position", drawBridgePosition);
    }
    public double getDrawbridgePosition(){
        return drawBridgePosition;
    }
    public boolean atSetpoint(){
        return pidController.atSetpoint();
    }
    public void runIntake(){
        intakeMotor.set(.55);
        DriverStation.reportWarning("Intake should be spinning",false);
    }
    public void outIntake(){
        intakeMotor.set(-.55);
    }
    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    public void raiseIntake() {
        double output = MathUtil.clamp(pidController.calculate(getDrawbridgePosition(), UPPER_TARGET_POSITION), 0, .55);
        drawbridgeMotor.set(ControlMode.PercentOutput, output);
    }
    public void lowerIntake() {
        double output = MathUtil.clamp(pidController.calculate(getDrawbridgePosition(), LOWER_TARGET_POSITION), -.35, 0);
        drawbridgeMotor.set(ControlMode.PercentOutput,output);
    }
    public void lowerIntakeAuto() {
        double output = MathUtil.clamp(pidController.calculate(getDrawbridgePosition(), -380), -.35, 0);
        drawbridgeMotor.set(ControlMode.PercentOutput,output);
    }
    public void stopDrawBridge(){
        drawbridgeMotor.set(ControlMode.PercentOutput,0);
    }

public BooleanSupplier atUpperLimit() {
        return ()-> Math.abs(getDrawbridgePosition() - UPPER_TARGET_POSITION) < pidController.getErrorTolerance();
    }
    public BooleanSupplier atLowerLimit() {
        return ()-> Math.abs(getDrawbridgePosition() - LOWER_TARGET_POSITION) < pidController.getErrorTolerance();
    }

    //Get the raw motor position. Only use this once per loop to minimize CAN usage.
    public double getMotorPosition(){
        return drawbridgeMotor.getSelectedSensorPosition();
    }


}
