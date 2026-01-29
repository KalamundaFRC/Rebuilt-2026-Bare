package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
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

//pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

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

      public Pose2d getPose() {
    return swerveDrive.getPose();
    }

      public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

    public ChassisSpeeds getRobotVelocity(){
    return swerveDrive.getRobotVelocity();
  }

    
}
