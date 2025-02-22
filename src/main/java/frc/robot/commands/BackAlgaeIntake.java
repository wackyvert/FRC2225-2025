package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class BackAlgaeIntake extends Command {
  AlgaeIntake algaeIntake;

    public BackAlgaeIntake(AlgaeIntake algaeIntake) {
        this.algaeIntake = algaeIntake;
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
        algaeIntake.backAlgaeIntake();
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
        return false;
    }
}
