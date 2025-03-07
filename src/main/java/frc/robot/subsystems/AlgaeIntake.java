package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.RunAlgaeIntakeLimit;

import static frc.robot.Constants.LOWER_TARGET_POSITION;
import static frc.robot.Constants.UPPER_TARGET_POSITION;


public class AlgaeIntake extends SubsystemBase {
    private final TalonSRX algaeIntake = new TalonSRX(16);
    private DigitalInput ballLimitSwitch= new DigitalInput(2);
    public AlgaeIntake(){
        setDefaultCommand(new RunAlgaeIntakeLimit(this));
    }
    @Override
    public void periodic(){

    }
    public void runAlgaeIfLimitSwitch(){
        if(!ballLimitSwitch.get()){
            runAlgaeIntake();
        }
    }
    public void runAlgaeIntake(){
      algaeIntake.set(ControlMode.PercentOutput, -.4);
    }
    public void backAlgaeIntake(){
        algaeIntake.set(ControlMode.PercentOutput, .4);
    }
    public void stopAlgaeIntake(){

        algaeIntake.set(ControlMode.PercentOutput, 0);
    }
    public boolean getAlgaeIntakeLimit(){
        return ballLimitSwitch.get();
    }



}
