package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lighting.LEDPatterns;
import frc.robot.lighting.LEDStrip;

public class LEDSubsystem extends SubsystemBase {

    // led strip on the catching bin
    public final LEDStrip catcherLED;

    public LEDSubsystem() {
        super();
        this.catcherLED = new LEDStrip(LED_LENGTH, LED_PWM_PORT, LEDPatterns.RED, ColorOrder.kGRB);
    }

    public Command setCatcherPatternCommand(LEDPattern pattern) {
        return setPatternCommand(pattern, catcherLED);
    }

    private Command setPatternCommand(LEDPattern pattern, LEDStrip ledStrip) {
        return runOnce(() -> ledStrip.setPattern(pattern));
    }

    public void updateCatcherPattern() {
        if (RobotState.isDisabled()) { // disabled logic
            
        } else if (RobotState.isAutonomous()) { 

        }

    }
        
   
        
}