package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;


public class LightsRainbow extends Command {
   private final LightSubsystem lightSubsystem;

    public LightsRainbow(LightSubsystem lightSubsystem) {
        this.lightSubsystem = lightSubsystem;
        addRequirements(lightSubsystem);
    }

    @Override
    public void initialize() {
       lightSubsystem.setRainbow();
    }

    @Override
    public void execute() {
       
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}
