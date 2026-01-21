package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;


public class Pivot extends SubsystemBase {
    WPI_TalonSRX Pivot_Motor;
    PIDController pidController;
    DutyCycleEncoder Encoder;
    double DoubleEncoderOutput;
    public Pivot () {
        Pivot_Motor = new WPI_TalonSRX(99);
        pidController = new PIDController(0, 0, 0);
        Encoder = new DutyCycleEncoder(99);
        DoubleEncoderOutput = Encoder.get();
    }
    public Command SetPivot(double value){
        return runEnd(
        () -> {
            Double target = MathUtil.clamp(value,PivotConstants.PivotLowerBoundLimit,PivotConstants.PivotUpperBoundLimit);
            Double result = MathUtil.clamp(pidController.calculate(Encoder.get(),target),-1*Constants.PivotConstants.PivotMaxVelocity,Constants.PivotConstants.PivotMaxVelocity);
            Pivot_Motor.set(result);
        }, () -> {
            Pivot_Motor.set(0);
        });
    }
}


