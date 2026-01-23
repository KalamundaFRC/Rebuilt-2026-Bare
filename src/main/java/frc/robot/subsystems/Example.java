package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class Example extends SubsystemBase {
    WPI_TalonSRX Example_Motor;
    double number;
    RelativeEncoder encoder;
    SparkMax motor;
    
    
    public Example(){
        motor = new SparkMax(99, MotorType.kBrushless);
        Example_Motor = new WPI_TalonSRX(99);
        encoder = motor.getEncoder();

        encoder.getVelocity();

    }

    public Command rawIndexerCommand(double voltage){
        return run(() -> { Example_Motor.setVoltage(voltage);
        });
    }

    public Command IndexCommand(double volts){
        return runEnd(
        () -> {
            Example_Motor.setVoltage(volts);
        },
        () -> {
            Example_Motor.setVoltage(0);
        }
        );
    }

        // public Command movearm(double position){
        // return runEnd(
        //     () -> {
        //         //makes a calculation using ArmPidCon.calculate
        //         Double target = MathUtil.clamp(position,Constants.ArmConstants.ArmLowerBoundLimit,Constants.ArmConstants.ArmUpperBoundLimit);
        //         Double result = MathUtil.clamp(ArmPidCon.calculate(ArmDutyCycleEncode.get(),target),-1*Constants.ArmConstants.ArmVelocityLimit,Constants.ArmConstants.ArmVelocityLimit);  
        //         ArmMotor1.set(result);
        //     },
        //     () -> {
        //         ArmMotor1.set(0);
            
        //     });
    //}

    
}
