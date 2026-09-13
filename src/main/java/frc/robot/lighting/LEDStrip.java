
package frc.robot.lighting;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * note: there must only be one instance of this class or there will be a error on RoboRIO, 
 * either daisy chain or y split as there can only be one led object and port, 
 * until we update to SystemCore which does support more than one led instance
 */

public class LEDStrip {
	// buffer and led objects
	private AddressableLEDBuffer ledBuffer;
	private AddressableLED led;
	
    public LEDStrip(int bufferLength, LEDPattern startPattern, ColorOrder order) {

		led = new AddressableLED(LED_PWM_PORT);

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
		SmartDashboard.putString("LED Color", this.getColor());
    }   

	public String getColor() {
		return ledBuffer.getLED(0).toHexString();
	}
}