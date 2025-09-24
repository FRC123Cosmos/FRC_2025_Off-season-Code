package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* -----------------------------------Version 0.75--------------------------------------- */

public class AlignToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController yController;    // only strafe
    private final PIDController xController;

    private final double positionTolerance = 0.05; // Meters (5 cm)
    private double targetOffsetY;        // desired static position from center of target
    private double targetOffsetX; 
    private double targetSetpointY;      // setpoint fed to PID
    private double targetSetpointX; 
    private double targetY;             // actual position from photonvision
    private double targetX;
    private double targetErrorY;        // error input to PID
    private double targetErrorX;
    private double pidOutY;             // PID control output
    private double pidOutX; 
    private double ySpeed;          // clipped PID control output
    private double xSpeed; 
    
    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, 0.0);
        targetOffsetX = 0.33;
    }

    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetY = targetOffsetY;
        targetOffsetX = 0.33;

        this.yController = new PIDController(1.0, 0, 0.0);
        this.xController = new PIDController(1.0, 0, 0.0);
        yController.setTolerance(positionTolerance);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        yController.reset();
        xController.reset();
        targetErrorY = 0.0;
        targetErrorX = 0.0;
        pidOutY = 0.0;
        pidOutX = 0.0;
        targetSetpointY = 1;
        targetSetpointX = 0.0;
        ySpeed = 0.0;
        xSpeed = 0.0;
        // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
        // SmartDashboard.putNumber("target_y", targetY);
        // SmartDashboard.putNumber("targetError", targetError);
        // SmartDashboard.putNumber("pidOut", pidOut);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {

            targetSetpointY = targetOffsetY;      // static command
            targetSetpointX = targetOffsetX;
            targetY = VisionSubsystem.getTarget_y();
            targetX = VisionSubsystem.getTarget_x();
            pidOutY = yController.calculate(targetY, targetSetpointY);
            pidOutX = xController.calculate(targetX, targetSetpointX);
            targetErrorY = yController.getError();
            targetErrorX = xController.getError();

            ySpeed = Math.max(-0.3, Math.min(0.3, pidOutY)); // Meters/sec
            xSpeed = Math.max(-0.3, Math.min(0.3, pidOutX));

            driveSubsystem.drive(-xSpeed, -ySpeed, 0, false, true);    // try kinematic rate limit??

        } else {
            pidOutY = 0.0;
            pidOutX = 0.0;
            ySpeed = 0.0;
            xSpeed = 0.0;
            targetErrorY = 0.0;
            targetErrorX = 0.0;
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
        // mpk - comment out after verifing target values
        // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
        // SmartDashboard.putNumber("target_y", targetY);
        // SmartDashboard.putNumber("targetError", targetError);
        // SmartDashboard.putNumber("pidOut", pidOut);
        // SmartDashboard.putNumber("ySpeed", ySpeed);

    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTarget() || (yController.atSetpoint() && xController.atSetpoint())) {
            return true; 
        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
}



// /* -----------------------------------Version 0.85--------------------------------------- */

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;

//     private final PIDController yController;    // only strafe
//     private final PIDController xController;
//     private final PIDController thetaController;

//     private final double positionTolerance = 0.04; // Meters (4 cm)
//     private final double angleTolerance = 2;    // Degree 
//     private double targetOffsetY;        // desired static position from center of target
//     private double targetOffsetX; 
//     private double targetOffsetTheta; 
//     private double targetSetpointY;      // setpoint fed to PID
//     private double targetSetpointX; 
//     private double targetSetpointTheta; 
//     private double targetY;             // actual position from photonvision
//     private double targetX;
//     private double targetTheta;
//     private double targetErrorY;        // error input to PID
//     private double targetErrorX;
//     private double targetErrorTheta;
//     private double pidOutY;             // PID control output
//     private double pidOutX; 
//     private double pidOutTheta;
//     private double ySpeed;          // clipped PID control output
//     private double xSpeed; 
//     private double thetaSpeed;
    
//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this(driveSubsystem, visionSubsystem, 0.0);
//         targetOffsetX = 0.33;
//         targetOffsetTheta = 0;
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffsetY = targetOffsetY;
//         targetOffsetX = 0.33;
//         targetOffsetTheta = 0;

//         this.yController = new PIDController(1.0, 0.0, 0.0);
//         this.xController = new PIDController(1.0, 0.0, 0.0);
//         this.thetaController = new PIDController(1.0, 0.0, 0.0);
//         yController.setTolerance(positionTolerance);
//         xController.setTolerance(positionTolerance);
//         thetaController.setTolerance(angleTolerance);

//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         yController.reset();
//         xController.reset();
//         targetErrorY = 0.0;
//         targetErrorX = 0.0;
//         targetErrorTheta = 0.0;
//         pidOutY = 0.0;
//         pidOutX = 0.0;
//         pidOutTheta = 0.0;
//         targetSetpointY = 1;
//         targetSetpointX = 0.0;
//         targetSetpointTheta = 25;
//         ySpeed = 0.0;
//         xSpeed = 0.0;
//         thetaSpeed = 0.0;
//         // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
//         // SmartDashboard.putNumber("target_y", targetY);
//         // SmartDashboard.putNumber("targetError", targetError);
//         // SmartDashboard.putNumber("pidOut", pidOut);
//         // SmartDashboard.putNumber("ySpeed", ySpeed);
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {

//             targetSetpointY = targetOffsetY;      // static command
//             targetSetpointX = targetOffsetX;
//             targetSetpointTheta = targetSetpointTheta;
//             targetY = VisionSubsystem.getTarget_y();
//             targetX = VisionSubsystem.getTarget_x();
//             targetTheta = VisionSubsystem.getTarget_rawYaw();
//             pidOutY = yController.calculate(targetY, targetSetpointY);
//             pidOutX = xController.calculate(targetX, targetSetpointX);
//             pidOutTheta = thetaController.calculate(targetTheta, targetSetpointTheta);
//             targetErrorY = yController.getError();
//             targetErrorX = xController.getError();
//             targetErrorTheta = thetaController.getError();

//             ySpeed = Math.max(-0.25, Math.min(0.25, pidOutY)); // Meters/sec
//             xSpeed = Math.max(-0.25, Math.min(0.25, pidOutX));
//             thetaSpeed = Math.max(-.1, Math.min(.1, pidOutTheta)); // degrees/sec


//             driveSubsystem.drive(-xSpeed, -ySpeed, -thetaSpeed, false, true);    // try kinematic rate limit??

//         } else {
//             pidOutY = 0.0;
//             pidOutX = 0.0;
//             pidOutTheta = 0.0;
//             ySpeed = 0.0;
//             xSpeed = 0.0;
//             thetaSpeed = 0.0;
//             targetErrorY = 0.0;
//             targetErrorX = 0.0;
//             targetErrorTheta = 0.0;
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//         // mpk - comment out after verifing target values
//         // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
//         // SmartDashboard.putNumber("target_y", targetY);
//         // SmartDashboard.putNumber("targetError", targetError);
//         // SmartDashboard.putNumber("pidOut", pidOut);
//         // SmartDashboard.putNumber("ySpeed", ySpeed);

//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget() || (yController.atSetpoint() && xController.atSetpoint() && (thetaController.atSetpoint()))) {
//             return true; 
//         } else {
//             return false;
//         }

//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }
// }
