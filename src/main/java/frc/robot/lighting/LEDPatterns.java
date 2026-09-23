package frc.robot.lighting;

import static edu.wpi.first.units.Units.Percent;
import static frc.robot.Constants.LEDConstants.LED_BRIGHTNESS_PERCENT;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

// define all led patterns in this class
public class LEDPatterns {
    
    public static final LEDPattern RED = atNormalBrightness(LEDPattern.solid(Color.kRed));
    public static final LEDPattern GREEN = atNormalBrightness(LEDPattern.solid(Color.kGreen));
    public static final LEDPattern BLUE = atNormalBrightness(LEDPattern.solid(Color.kBlue));

    public static LEDPattern atNormalBrightness(LEDPattern pattern) {
        return pattern.atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); 

    }
}
