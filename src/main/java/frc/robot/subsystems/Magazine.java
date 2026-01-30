package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MagazineConstants;

public class Magazine extends SubsystemBase{    
WPI_TalonSRX Magazine_Motor;
    //constructer
}

public Magazine(){
    Magazine_Motor=new WPI_TalonSRX(99);
    Magazine_Motor.configPeakCurrentLimit(40);
}
public Command runMagazine(double voltage){
    return runEnd(
        () -> {
            Magazine_Motor.setVoltage(voltage);
        },
        () -> {
            Magazine_Motor.setVoltage(0);
        });
    }
    