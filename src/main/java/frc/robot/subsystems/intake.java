package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
    //vars here
    WPI_TalonSRX Intake_Motor;
    public Intake() {
        Intake_Motor = new WPI_TalonSRX(99);
        Intake_Motor.configPeakCurrentLimit(IntakeConstants.IntakeTalonCurrentLimit);
    }

    public Command IntakeRun() {
        return runEnd(
            () -> {
                Intake_Motor.setVoltage(IntakeConstants.IntakeTalonVoltage);
            }, () -> {
                Intake_Motor.set(0);
            }
            );
    }


};