package frc.robot.lighting;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.LEDConstants.*;
import static frc.robot.Constants.OperatingConstants.DRIVER_CONTROLLER_PORT;

import java.util.Map;
 
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

// define all led patterns in this class
public class LEDPatterns {

    // -- simple color patterns --

    public static final LEDPattern WHITE = simpleColor(Color.kWhite);
    public static final LEDPattern BLACK = simpleColor(Color.kBlack);

    public static final LEDPattern RED = simpleColor(Color.kRed);
    public static final LEDPattern GREEN = simpleColor(Color.kLime);
    public static final LEDPattern BLUE = simpleColor(Color.kBlue);

    public static final LEDPattern ORANGE = simpleColor(Color.kOrange);
    public static final LEDPattern PURPLE = simpleColor(Color.kPurple);

    // -- component patterns --

    public static final LEDPattern GOLD_BLUE = LEDPattern.gradient(GradientType.kDiscontinuous, Colors.MAIN_GOLD, Colors.MAIN_BLUE); // 3/5 stars
    public static final LEDPattern GOLD_BLUE_SCROLL = normBrightness(scroll(GOLD_BLUE)); // 4/5 stars

    public static final LEDPattern ORANGE_PURPLE = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kOrange, Color.kPurple); // 3/5 stars
    public static final LEDPattern ORANGE_PURPLE_SCROLL = normBrightness(scroll(ORANGE_PURPLE)); // 4/5 stars    

    // "snake" effect
    public static final LEDPattern SNAKE = scroll(GREEN.mask(LEDPattern.progressMaskLayer(() -> 1/3F))); // 4/5 stars

    // scrolling rainbow
    public static final LEDPattern RAINBOW_SCROLL = normBrightness(scroll(LEDPattern.rainbow(255, 255))); // 3/5 stars

    // rgb/cmy scroll
    public static final LEDPattern RGB_SCROLL = normBrightness(scroll(LEDPattern.steps(Map.of(
        0.00, Color.kRed,
        0.17, Color.kLime,
        0.33,Color.kBlue,
        0.50, Color.kRed,
        0.67, Color.kLime,
        0.83, Color.kBlue)))); // 2/5 stars
    public static final LEDPattern CMY_SCROLL = normBrightness(scroll(LEDPattern.steps(Map.of(
        0.00, Color.kCyan,
        0.17, Color.kMagenta,
        0.33, Color.kYellow,
        0.50, Color.kCyan,
        0.67, Color.kMagenta,
        0.83, Color.kYellow)))); // 4/5 stars
    public static final LEDPattern RGB_CMY_SCROLL = normBrightness(scroll(LEDPattern.steps(Map.of(
        0.00, Color.kRed,
        0.17, Color.kLime,
        0.33, Color.kBlue,
        0.50, Color.kCyan,
        0.67, Color.kMagenta,
        0.83, Color.kYellow)))); // 4/5 stars 

    // -- helper methods --

    // already at normal brightness
    public static LEDPattern simpleColor(Color color) {
        return normBrightness(LEDPattern.solid(color));
    }

    public static LEDPattern normBrightness(LEDPattern pattern) {
        return brightness(LED_BRIGHTNESS_PERCENT, pattern);
    }

    public static LEDPattern brightness(int brightnessPercent, LEDPattern pattern) {
        return pattern.atBrightness(Dimensionless.ofRelativeUnits(brightnessPercent, Percent)); 
    }

    public static LEDPattern scroll(LEDPattern pattern) {
        return scroll(LED_SCROLL_SPEED, pattern);
    }

    public static LEDPattern scroll(int percentPerSecond, LEDPattern pattern) {
        return pattern.scrollAtRelativeSpeed(Percent.per(Second).of(percentPerSecond));
    }

    public static LEDPattern blend(LEDPattern pattern1, LEDPattern pattern2) {
        return pattern1.blend(pattern2);
    }
}
