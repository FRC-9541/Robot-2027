
package frc.robot.lighting;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;

// instance of this class for every different led strip
public class LEDStrip {
	// buffer and led objects
	private AddressableLEDBuffer ledBuffer;
	private AddressableLED led;
	
    public LEDStrip(int bufferLength, int ledPort, LEDPattern startPattern, ColorOrder order) {

		led = new AddressableLED(ledPort);

		// Length is expensive to set, so only set it once, then just update data
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		led.setLength(ledBuffer.getLength());

		// allow for other color orders (like grb instead of rgb)
		led.setColorOrder(order);

		// Set the data, will not work without it being updated
		led.setData(ledBuffer);
		led.start();

		setPattern(startPattern);
    }

    public void setPattern(LEDPattern pattern) {
        // apply and set data
        pattern.applyTo(ledBuffer);
	    led.setData(ledBuffer);
    }   
}