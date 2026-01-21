package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    SparkMax Shooter;
    PIDController PIDController;
    RelativeEncoder Encoder;
    Double Velocity;
    public Shooter() {
        Shooter = new SparkMax(99,MotorType.kBrushless);
        PIDController = new PIDController(0, 0, 0);
        Encoder = Shooter.getEncoder();
        Velocity = Encoder.getVelocity();

    }

    public Command SetShooter(double value) {
        return run(
            () -> {
                Double target = MathUtil.clamp(value,ShooterConstants.ShooterMinRotations, ShooterConstants.ShooterMaxRotations);
                Double result = PIDController.calculate(Velocity,target);
                Shooter.set(result);
            }
        );
    }
}