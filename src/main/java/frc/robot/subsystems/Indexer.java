package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    WPI_TalonSRX Indexer_Motor;

    public Indexer(){
        Indexer_Motor = new WPI_TalonSRX(99);
    }

    public Command rawIndexerCommand(double voltage){
        return run(
            ( ) -> {
                Indexer_Motor.setVoltage(voltage);
            });
    }
}
