package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsysubsystem extends SubsystemBase {
    File swerveJsonDirectory;
    SwerveDrive swerveDrive;
    Pose2d pose;

    public SwerveSubsysubsystem(){
        try{
        pose = new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0));
        swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.SwerveConstants.maxSpeed, pose);
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }
    
}
