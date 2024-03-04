// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.SubsystemHandler;
import frc.robot.util.Constants;

public class LED extends SubsystemHandler {
    private static LED m_Led;

    private CANdle m_Candle;

    int ledOffset = 8;
    int numLed = 32 + 32 + 8 - ledOffset;

    int r, g, b = 0;

    public boolean override = false;

    public boolean intake = false;

    public static LED get() {
        if (m_Led == null) {
            m_Led = new LED();
        }
        return m_Led;
    }

    private LED() {
        m_Candle = new CANdle(1, Constants.canivoreName);
        m_Candle.configAllSettings(Constants.CONFIGS.candle);
        for(int i = 0; i<m_Candle.getMaxSimultaneousAnimationCount(); i++) {
            m_Candle.clearAnimation(i);
        }
        defaultAnimate();
    }
    
    private void defaultAnimate() {
        m_Candle.animate(new LarsonAnimation( 255,  50, 60, 255, 0.3, numLed, BounceMode.Front, 9, ledOffset ), 0);
    }

    private void setLeds(int _r, int _g, int _b) {
        m_Candle.clearAnimation(0);
        m_Candle.setLEDs(_r, _g, _b, 255, ledOffset, numLed);
    }

    public void setOverride(int r, int g, int b) {
        this.r = r;
        this.b = b;
        this.g = g;
    }

    @Override
    public void periodic() {
        if (override) {
            setLeds(r,g,b);
        } else if (DriverStation.isTeleopEnabled()) {
            if (intake) {
                setLeds(30, 0 , 220);
            } else {
                setLeds(255, 90, 10);
            }
        } else {
            defaultAnimate();
        }
    }
}
