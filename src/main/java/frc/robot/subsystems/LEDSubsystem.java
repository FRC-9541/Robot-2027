package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lighting.LEDStrip;

public class LEDSubsystem extends SubsystemBase {

    private static final LEDPattern RED = LEDPattern.solid(Color.kRed)
        .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); 

    // led strip on the catching bin
    public final LEDStrip catcherLED;

    public LEDSubsystem() {
        super();
        this.catcherLED = new LEDStrip(92, LED_PWM_PORT, RED, ColorOrder.kGRB);
    }


    public void updateCatcherPattern() {
        if (RobotState.isDisabled()) { // disabled logic
            
        } else if (RobotState.isAutonomous()) { 

        }

    }
        
   
        
}