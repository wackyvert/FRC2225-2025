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
import frc.robot.commands.SetGreenCommand;
import frc.robot.commands.SetRainbowCommand;

import static frc.robot.Constants.LOWER_TARGET_POSITION;
import static frc.robot.Constants.UPPER_TARGET_POSITION;


public class AlgaeIntake extends SubsystemBase {
    LightSubsystem lightSubsystem;
    private final TalonSRX algaeIntake = new TalonSRX(16);
    private DigitalInput ballLimitSwitch= new DigitalInput(2);
    public AlgaeIntake(LightSubsystem lightSubsystem){
        this.lightSubsystem=lightSubsystem;
        setDefaultCommand(new RunAlgaeIntakeLimit(this));
    }
    private boolean triggered = false;
    public void setLED(){

        if (!getAlgaeIntakeLimit() && !triggered) {
            new SetGreenCommand(lightSubsystem).schedule();
            triggered = true;
        } else if (getAlgaeIntakeLimit() && triggered) {
            // Optionally revert to rainbow when the switch becomes true again
            new SetRainbowCommand(lightSubsystem).schedule();
            triggered = false;
        }
    }
    @Override
    public void periodic(){
SmartDashboard.putBoolean("algae limit", getAlgaeIntakeLimit());
    }
    public void runAlgaeIfLimitSwitch(){
        if(!getAlgaeIntakeLimit()){
            runAlgaeIntakeL();
        }
    }
    public void runAlgaeIntake(){
      algaeIntake.set(ControlMode.PercentOutput, -.75);
    }
    public void runAlgaeIntakeL(){
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
