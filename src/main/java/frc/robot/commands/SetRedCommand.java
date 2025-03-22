package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LightSubsystem;

// Example command
public class SetRedCommand extends InstantCommand {
    public SetRedCommand(LightSubsystem lights) {
        addRequirements(lights);
        this.lights = lights;
    }
    private final LightSubsystem lights;

    @Override
    public void initialize() {
        lights.setLEDState(LightSubsystem.LEDState.SOLID_RED);
    }
}
