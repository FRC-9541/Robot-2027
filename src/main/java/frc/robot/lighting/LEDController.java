
package frc.robot.lighting;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Percent;
import static frc.robot.Constants.LEDConstants.*;


public class LEDController {

    // the LED strip that we use has a grb setup, so flip green and red values
	private final LEDPattern red = LEDPattern.solid(Color.kGreen)
	.atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); 

	private AddressableLEDBuffer ledBuffer;
	private AddressableLED led;
	
    public LEDController(int bufferLength) {

		led = new AddressableLED(LED_PWM_PORT);

		// Length is expensive to set, so only set it once, then just update data
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		led.setLength(ledBuffer.getLength());

		// Set the data, will not work without it being updated
		led.setData(ledBuffer);
		led.start();

        updateLEDS();
    }

    public void updateLEDS() {
        
        // set to red example
        red.applyTo(ledBuffer);
	    led.setData(ledBuffer);
    }   
}