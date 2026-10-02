
package frc.robot.lighting;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;

/**
 * note: there must only be one instance of this class or there will be a error on RoboRIO, 
 * either daisy chain or y split as there can only be one led object and port, 
 * until we update to SystemCore which does support more than one led instance
 */

public class LEDStrip {
	// buffer and led objects
	private AddressableLEDBuffer ledBuffer;
	private AddressableLED led;
	private LEDPattern pattern;
	
    public LEDStrip(int bufferLength, LEDPattern startPattern, ColorOrder order) {

		pattern = startPattern;

		led = new AddressableLED(LED_PWM_PORT);

		// Length is expensive to set, so only set it once, then just update data
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		led.setLength(ledBuffer.getLength());

		// allow for other color orders (like grb instead of rgb)
		led.setColorOrder(order);

		// Set the data, will not work without it being updated
		update();
		led.start();

    }

	public void update() {
		pattern.applyTo(ledBuffer);
		led.setData(ledBuffer);
	}

    public void setPattern(LEDPattern pattern) {
        // apply and set data
        this.pattern = pattern;
	}   

	public String getDefaultColor() {
		return ledBuffer.getLED(0).toHexString();
	}
}