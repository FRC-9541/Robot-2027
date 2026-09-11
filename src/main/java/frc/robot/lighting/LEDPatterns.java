package frc.robot.lighting;

import static edu.wpi.first.units.Units.Percent;
import static frc.robot.Constants.LEDConstants.LED_BRIGHTNESS_PERCENT;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

// define all led patterns in this class
public class LEDPatterns {
    
    public static final LEDPattern RED = LEDPattern.solid(Color.kRed)
        .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); 

    public static final LEDPattern GREEN = LEDPattern.solid(Color.kGreen)
        .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); 

    public static final LEDPattern BLUE = LEDPattern.solid(Color.kBlue)
        .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); 

}
