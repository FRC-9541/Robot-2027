package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * this class is only staying until we remove the launcher from 2026
 */
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax leftIntakeShootExpel = new SparkMax(LEFT_LAUNCH_MOTOR_ID, MotorType.kBrushed);
    private final SparkMax rightBinIntakeExpel = new SparkMax(RIGHT_LAUNCH_MOTOR_ID, MotorType.kBrushed);

    private final Timer spinUpTimer = new Timer();
    
    public ShooterSubsystem() {

        // set config for motors
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(60);
        leftConfig.inverted(false);
        leftIntakeShootExpel.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(60);
        rightConfig.inverted(false);
        rightBinIntakeExpel.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    // very poor code but its fine cus its gonna get removed
    public Command launchCommand(boolean rightBumper, boolean rightBumperPressed, boolean aButton, boolean yButton) {
        return run(() -> {
            if (rightBumper) { // press Right Bumper to launch fuel
                if (rightBumperPressed) {
                    spinUpTimer.reset();
                }
                if (spinUpTimer.get() < SPINUP_SECONDS) { // spinning up the Launcher
                    leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
                    rightBinIntakeExpel.setVoltage(SPINUP_RIGHT_VOLTAGE);
                } else {
                    leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
                    rightBinIntakeExpel.setVoltage(LAUNCHING_RIGHT_VOLTAGE);
                }
            } else if (aButton) { // press A to Intake fuel from Floor
                leftIntakeShootExpel.setVoltage(INTAKING_LEFT_VOLTAGE);
                rightBinIntakeExpel.setVoltage(INTAKING_RIGHT_VOLTAGE);
            } else if (yButton) { // press Y to Expel fuel
                leftIntakeShootExpel.setVoltage(-INTAKING_LEFT_VOLTAGE);
                rightBinIntakeExpel.setVoltage(-INTAKING_RIGHT_VOLTAGE);
            } else { // turn stuff off if nothing is pressed
                leftIntakeShootExpel.setVoltage(0);
                rightBinIntakeExpel.setVoltage(0);
            }
        });
    }

}
