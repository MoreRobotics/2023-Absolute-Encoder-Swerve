package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMode;
import frc.robot.commands.YellowLED;

public class LEDs extends SubsystemBase{

    public static final int CANDLE_PORT = 9;  
    
    CANdle candle = new CANdle(CANDLE_PORT);
    
    public LEDs() {

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        setColor(50,50,255);

    }

    public void rainbow() {

        
        candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white
            
        // create a rainbow animation:
        // - max brightness
        // - half speed
        // - 64 LEDs
        RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 308);
        candle.animate(rainbowAnim);
    }



    public void setColor(int red, int green, int blue) {

        //System.out.println(candle.setLEDs(red, green, blue));
        ColorFlowAnimation animation = new ColorFlowAnimation(red, green, blue, 0, 1, 308, Direction.Forward);

        System.out.println(candle.animate(animation));
    }

    @Override
    public void periodic() {
        if (RobotMode.mode == RobotMode.ModeOptions.CONE) {

            setColor(255,150,0);

        } else {

            setColor(120,0,120);
        }
    }
    
}
