package frc.robot.commands;

import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.LightSubsystem;

public class RunAlgaeIntakeLimit extends Command {
    AlgaeIntake algaeIntake;
    LightSubsystem lightSubsystem;

    public RunAlgaeIntakeLimit(AlgaeIntake algaeIntake, LightSubsystem lightSubsystem) {
        this.algaeIntake = algaeIntake;
        this.lightSubsystem=lightSubsystem;
        addRequirements(algaeIntake);
    }

    @Override
    public void initialize() {
        //I'm not going to be using this method in this example. There is nothing I need to use it for.
        
    }

    @Override
    public void execute() {
        //This method is ran every CPU cycle by the RoboRio while the command is scheduled.
        //For example, if it is bound to A, while A is pressed, it will continually be sent a command.
        //So, 60 times a second, the PID Controller calculates a new output level, and sends that to the motor.
        //This is how we can make it go up and down quickly and smoothly.
        algaeIntake.runAlgaeIfLimitSwitch();
        if(!algaeIntake.getAlgaeIntakeLimit()){
            lightSubsystem.setGreen();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //This method gets called after the isFinished method returns true.
        // It is always necessary to stop your motors once the command finishes,
        // otherwise they will spin indefinitely.
        algaeIntake.stopAlgaeIntake();
    }

    @Override
    public boolean isFinished() {
        //When the drawbridge is at the setpoint, this boolean will return true, causing the command to stop (and the motors to be stopped.)
        if (algaeIntake.getAlgaeIntakeLimit()) {
            return true;
        }
        return false;
    }
}
