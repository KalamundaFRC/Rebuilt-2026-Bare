package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

import javax.naming.spi.DirStateFactory.Result;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
    SparkMax Shooter;
    SparkMax Shooter2;
    PIDController PIDController;
    RelativeEncoder Encoder;
    Double Velocity;
    //Timer timer;
    SparkMaxConfig sparkMaxConfig;
    SparkMaxConfig sparkMaxConfig2;
    public Shooter() {
        Shooter = new SparkMax(11,MotorType.kBrushless);
        Shooter2 = new SparkMax(12, MotorType.kBrushless);
        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig2 = new SparkMaxConfig();
        sparkMaxConfig.smartCurrentLimit(40);
        sparkMaxConfig2.smartCurrentLimit(40);
        // sparkMaxConfig2.inverted(true);
        // sparkMaxConfig.inverted(false);
        sparkMaxConfig2.follow(Shooter,true);
        Shooter.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        Shooter2.configure(sparkMaxConfig2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        
        PIDController = new PIDController(0, 0, 0);
        Encoder = Shooter.getEncoder();
        Velocity = Encoder.getVelocity();
        //timer.start();
    }

    public Command RawShooter(double value) {
        return runEnd(
            () -> {
                Shooter.set(value);
            }, 
            () -> {
                Shooter.set(0);
            });
    }


    public Command SetShooter(double value) {
        return runEnd(
            () -> {
                Double target = MathUtil.clamp(value,ShooterConstants.ShooterMinRotations, ShooterConstants.ShooterMaxRotations);
                Double result = PIDController.calculate(Velocity,target);
                Shooter.set(result);
            }, () -> {
                Shooter.set(0);
            }
        );
    }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putNumber("Shooter: ", Encoder.getVelocity());
    // }

}