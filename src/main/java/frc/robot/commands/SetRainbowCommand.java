package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LightSubsystem;

public class SetRainbowCommand extends InstantCommand {
    public SetRainbowCommand(LightSubsystem lights) {
        addRequirements(lights);
        this.lights = lights;
    }
    private final LightSubsystem lights;

    @Override
    public void initialize() {
        lights.setLEDState(LightSubsystem.LEDState.FIRE);
    }
}
