package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    WPI_TalonSRX Indexer_Motor;
    WPI_TalonSRX Mag_Motor;

    public Indexer(){
        Indexer_Motor = new WPI_TalonSRX(15);
        Mag_Motor = new WPI_TalonSRX(14);

        Mag_Motor.configPeakCurrentLimit(30,10);
        Mag_Motor.configPeakCurrentDuration(200,10);
        Mag_Motor.configContinuousCurrentLimit(20,10);

        Indexer_Motor.configPeakCurrentLimit(30,10);
        Indexer_Motor.configPeakCurrentDuration(200,10);
        Indexer_Motor.configContinuousCurrentLimit(20,10);
    }

    public Command rawIndexerCommand(double value){
        return runEnd(
            () -> {
                Indexer_Motor.set(value);
            }, () -> {
                Indexer_Motor.set(0);
            });
    }

    public Command rawMagCommand(double value){
        return runEnd(() -> {
            Mag_Motor.set(value*0.7);
        }, () -> {
            Mag_Motor.set(0);
        });
    }

    public Command shoot(double value){
        return runEnd(() -> {
            Mag_Motor.set(-value*0.7);
            Indexer_Motor.set(value);
        }, () -> {
            Mag_Motor.set(0);
            Indexer_Motor.set(0);
        });
    }

    
    public Command intake(double value){
        return runEnd(() -> {
            Mag_Motor.set(value*0.7);
            Indexer_Motor.set(value);
        }, () -> {
            Mag_Motor.set(0);
            Indexer_Motor.set(0);
        });
    }
}
