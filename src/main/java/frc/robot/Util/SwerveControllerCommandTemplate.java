// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class SwerveControllerCommandTemplate {
    DrivetrainSubsystem m_drivetrainSubsystem;

    public SwerveControllerCommand SwerveControllerCommand(DrivetrainSubsystem m_drivetrainSubsystem, Trajectory trajectory) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;

        double 
            maxV = 5, 
            maxA = 3,
            p = .66,
            i = 0,
            d = .025;
        
        PIDController xController = new PIDController(p , i, d);
        PIDController yController = new PIDController(p, i, d);
        var thetaController = new ProfiledPIDController(5 , 0, 0, new TrapezoidProfile.Constraints(maxV, maxA));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_drivetrainSubsystem::getPose,
                m_drivetrainSubsystem.m_kinematics,
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);

        return swerveControllerCommand;
    }


    public SwerveControllerCommand SwerveControllerCommandPID(DrivetrainSubsystem m_drivetrainSubsystem, Trajectory trajectory, double p, double i, double d, double thetaP, double maxVel, double maxAccel) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;

        PIDController xController = new PIDController(p , i, d);
        PIDController yController = new PIDController(p, i, d);
        var thetaController = new ProfiledPIDController(thetaP , 0, 0, new TrapezoidProfile.Constraints(maxVel, maxAccel));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            m_drivetrainSubsystem::getPose, 
            m_drivetrainSubsystem.m_kinematics, 
            xController, 
            yController, 
            thetaController, 
            m_drivetrainSubsystem::setModuleStates, 
            m_drivetrainSubsystem);
        
        return swerveControllerCommand;
    }

    
    public SwerveControllerCommand SwerveControllerCommandPID(DrivetrainSubsystem m_drivetrainSubsystem, Trajectory trajectory, double p, double i, double d, double thetaP, double maxVel, double maxAccel) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        
        PIDController xController = new PIDController(p , i, d);
        PIDController yController = new PIDController(p, i, d);
        var thetaController = new ProfiledPIDController(thetaP , 0, 0, new TrapezoidProfile.Constraints(maxVel, maxAccel));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            m_drivetrainSubsystem::getPose, 
            m_drivetrainSubsystem.m_kinematics, 
            xController, 
            yController, 
            thetaController, 
            m_drivetrainSubsystem::setModuleStates, 
            m_drivetrainSubsystem);
        
        return swerveControllerCommand;
    }
}
