// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import frc.robot.RobotContainer;

import java.util.Optional;

public class LightSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(44, "rio");
    private final int LedCount = 100;


    private Animation m_toAnimate = null;

    public enum AnimationTypes {
       solid_yellow_strobe,
        solid_green_strobe,
        solid_purple_strobe,
        solid_orange_strobe,
        solid_white_strobe,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    private AnimationTypes m_currentAnimation;
    private XboxController noah = new XboxController(2);
    public LightSubsystem() {

        //changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
       // configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = .4;
       // configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }
    public void setGreen(){
        m_candle.setLEDs(0,255, 0, 0, 0, 100);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch(toChange)
        {
            case solid_yellow_strobe:
                m_toAnimate = new StrobeAnimation(226,255,0, 0, .8, LedCount);
                break;
            case solid_orange_strobe:
                m_toAnimate = new StrobeAnimation(226,255,0, 0, .8, LedCount);
                break;
            case solid_green_strobe:
                m_toAnimate = new StrobeAnimation(255, 0,0, 0, .8, LedCount);
                break;
            case solid_purple_strobe:
                m_toAnimate = new StrobeAnimation(154, 0, 255, 0, .8, LedCount);
                break;
            case solid_white_strobe:
                m_toAnimate = new StrobeAnimation(255, 255, 255, 0, .8, LedCount);
                break;
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        //System.out.println("Changed to " + m_currentAnimation.toString());
    }
    boolean clearCandle=false;
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
            if(!clearCandle){
                m_candle.animate(m_toAnimate);
            }
            if (noah.getAButton()){
            changeAnimation(AnimationTypes.solid_yellow_strobe);
           turnOnAnimation();
            }
            else if (noah.getXButton()){
                changeAnimation(AnimationTypes.solid_green_strobe);
            turnOnAnimation();
            }
            else if (noah.getYButton()){
              
                changeAnimation(AnimationTypes.solid_purple_strobe);
                turnOnAnimation();
            }
            else if (noah.getRightTriggerAxis()>.5){
                changeAnimation(AnimationTypes.solid_orange_strobe);
                turnOnAnimation();
            }
            else if (noah.getLeftTriggerAxis()>.5){
            changeAnimation(AnimationTypes.solid_white_strobe);
            turnOnAnimation();
            
            }
            else{
                changeAnimation(AnimationTypes.Rainbow);
            }

    }
    public void turnOffAnimation(){
        clearCandle=true;
        m_candle.clearAnimation(0);
        setTeamColor();
    }
    public void setTeamColor() {
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == DriverStation.Alliance.Red) {
                m_candle.setLEDs(255, 0, 0);
            } else if (ally.get() == DriverStation.Alliance.Blue) {
                m_candle.setLEDs(0, 0, 255);
            }
        }
    }
    public void turnOnAnimation(){

        clearCandle=false;
    }
    public Animation getOldAnimation(){
        return m_toAnimate;
    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}