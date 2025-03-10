package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.LOWER_TARGET_POSITION;
import static frc.robot.Constants.UPPER_TARGET_POSITION;


public class Intake extends SubsystemBase {

    SparkMax intakeMotor;
    TalonSRX drawbridgeMotor;
    PIDController pidController;

    double drawBridgePosition;
    public Intake(){
        drawbridgeMotor = new TalonSRX(Constants.DRAWBRIDGE_ID);
        drawbridgeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor = new SparkMax(Constants.INTAKE_ID, SparkLowLevel.MotorType.kBrushless);
        pidController = new PIDController(.1,0,0);
        pidController.setTolerance(5);
    }
    @Override
    public void periodic(){
        //Get the position of the motor once per loop only, and store it in the global variable.
        drawBridgePosition = getMotorPosition();
        SmartDashboard.putNumber("Intake", drawBridgePosition);
    }
    public double getDrawbridgePosition(){
        return drawBridgePosition;
    }
    public boolean atSetpoint(){
        return pidController.atSetpoint();
    }
    public void runIntake(){
        intakeMotor.set(.8);
        DriverStation.reportWarning("Intake should be spinning",false);
    }
    public void outIntake(){
        intakeMotor.set(-.8);
    }
    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    public void raiseIntake() {
        double output = pidController.calculate(getDrawbridgePosition(), UPPER_TARGET_POSITION);
        drawbridgeMotor.set(ControlMode.PercentOutput, .4);
    }
    public void lowerIntake() {
        double output = pidController.calculate(getDrawbridgePosition(), LOWER_TARGET_POSITION);
        drawbridgeMotor.set(ControlMode.PercentOutput,-.4);
    }
    public void stopDrawBridge(){
        drawbridgeMotor.set(ControlMode.PercentOutput,0);
    }



    //Get the raw motor position. Only use this once per loop to minimize CAN usage.
    public double getMotorPosition(){
        return drawbridgeMotor.getSelectedSensorPosition();
    }


}
