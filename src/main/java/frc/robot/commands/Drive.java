// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongToDoubleFunction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pods;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private Pods pods;
  private DoubleSupplier LX;
  private DoubleSupplier LY;
  private DoubleSupplier RX;
  private DoubleSupplier RY;

  private DoubleSupplier LT;
  private DoubleSupplier RT;

  private IntSupplier POV;

  private double TargetHeading;


  public Drive(Pods pods, DoubleSupplier LX, DoubleSupplier LY, DoubleSupplier RX, DoubleSupplier RY, DoubleSupplier RT, DoubleSupplier LT, IntSupplier POV) {
    this.pods = pods;
    this.LX = LX;
    this.LY = LY;
    this.RX = RX;
    this.RY = RY;

    this.RT = RT;
    this.LT = LT;
    
    this.POV = POV;
    addRequirements(pods);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TargetHeading = pods.gyroget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double gyroCurrent = pods.gyroget() * -1;
    SmartDashboard.putNumber("gyroCurrent", gyroCurrent);
    //right moving, left steering; field centric moving and target turning 
    double fieldMoveAngle;
    if (Math.abs(RY.getAsDouble()) > Constants.deadband|| Math.abs(RX.getAsDouble()) > Constants.deadband) {
      fieldMoveAngle = (Math.atan2(RX.getAsDouble(), -RY.getAsDouble()))*180/Math.PI; //deg, clockwise is pos b/c X and Y switched
    }
    else {
      fieldMoveAngle = gyroCurrent;
    }
      double robotMoveAngle = fieldMoveAngle - gyroCurrent;

    double robotSpeed = (Math.sqrt((RX.getAsDouble() * RX.getAsDouble()) + (RY.getAsDouble() * RY.getAsDouble()))) * Constants.moveGain;
    if (Math.abs(RY.getAsDouble()) < Constants.deadband && Math.abs(RX.getAsDouble()) < Constants.deadband) {
      robotSpeed = 0;
    }
    double robotMoveX = robotSpeed * Math.sin(robotMoveAngle*Math.PI/180);
    double robotMoveY = robotSpeed * Math.cos(robotMoveAngle*Math.PI/180);

    if (POV.getAsInt() != -1) {
        TargetHeading = POV.getAsInt();
    }
      else if (Math.abs(LY.getAsDouble()) > Constants.deadbandSteer || Math.abs(LX.getAsDouble()) > Constants.deadbandSteer) {
      TargetHeading = (Math.atan2(LX.getAsDouble(), -LY.getAsDouble()))*180/Math.PI; //deg, clockwise is pos b/c X and Y switched
      }
    TargetHeading += (RT.getAsDouble() - LT.getAsDouble()) * Constants.triggerGain;
    
    double robotMoveZ = (((TargetHeading - gyroCurrent)%360));
    robotMoveZ += ((robotMoveZ > 180) ? -360 : ((robotMoveZ < -180) ? 360 : 0));
    robotMoveZ = robotMoveZ * Constants.turnP * (1/Math.sqrt(2)); //- pods.gyrovelocity() * Constants.turnD;
    SmartDashboard.putNumber("robotmovez", robotMoveZ);
    SmartDashboard.putNumber("targetheading", TargetHeading);
    //pod 1 (BR)
    double c1current = ((pods.c1get()-Constants.c1offset));

    double BRX = robotMoveX - robotMoveZ;
    double BRY = robotMoveY - robotMoveZ;

    double BRMoveAngle = (Math.atan2(BRX, BRY))*180/Math.PI; //deg, clockwise is pos b/c X and Y switched
    double BRSpeed = (Math.sqrt((BRX*BRX) + (BRY*BRY)));

    double BRerror = BRMoveAngle - c1current;
    if (BRerror > 180) {
      BRerror -= 360;
    } else if (BRerror < -180) {
      BRerror += 360;
    }
    if (BRerror > 90) {
      BRerror -= 180;
      BRSpeed = -BRSpeed;
    } else if (BRerror < -90) {
      BRerror += 180;
      BRSpeed = -BRSpeed;
    }

    pods.s1set(pods.s1get() + (BRerror*43008/360));
    pods.d1set(BRSpeed*Constants.driveGain);


      //pod 2 (FR)
    double c2current = ((pods.c2get()-Constants.c2offset));

    double FRX = robotMoveX + robotMoveZ;
    double FRY = robotMoveY - robotMoveZ;

    double FRMoveAngle = (Math.atan2(FRX, FRY))*180/Math.PI; //deg, clockwise is pos b/c X and Y switched
    double FRSpeed = (Math.sqrt((FRX*FRX) + (FRY*FRY)));

    double FRerror = FRMoveAngle - c2current;
    if (FRerror > 180) {
      FRerror -= 360;
    } else if (FRerror < -180) {
      FRerror += 360;
    }
    if (FRerror > 90) {
      FRerror -= 180;
      FRSpeed = -FRSpeed;
    } else if (FRerror < -90) {
      FRerror += 180;
      FRSpeed = -FRSpeed;
    }

    pods.s2set(pods.s2get() + (FRerror*43008/360));
    pods.d2set(FRSpeed*Constants.driveGain);


    //pod 3 (FR)
    double c3current = ((pods.c3get()-Constants.c3offset));

    double FLX = robotMoveX + robotMoveZ;
    double FLY = robotMoveY + robotMoveZ;

    double FLMoveAngle = (Math.atan2(FLX, FLY))*180/Math.PI; //deg, clockwise is pos b/c X and Y switched
    double FLSpeed = (Math.sqrt((FLX*FLX) + (FLY*FLY)));

    double FLerror = FLMoveAngle - c3current;
    if (FLerror > 180) {
      FLerror -= 360;
    } else if (FLerror < -180) {
      FLerror += 360;
    }
    if (FLerror > 90) {
      FLerror -= 180;
      FLSpeed = -FLSpeed;
    } else if (FLerror < -90) {
      FLerror += 180;
      FLSpeed = -FLSpeed;
    }

    pods.s3set(pods.s3get() + (FLerror*43008/360));
    pods.d3set(FLSpeed*Constants.driveGain);


    //pod 4 (BL)
    double c4current = ((pods.c4get()-Constants.c4offset));

    double BLX = robotMoveX - robotMoveZ;
    double BLY = robotMoveY + robotMoveZ;

    double BLMoveAngle = (Math.atan2(BLX, BLY))*180/Math.PI; //deg, clockwise is pos b/c X and Y switched
    double BLSpeed = (Math.sqrt((BLX*BLX) + (BLY*BLY)));

    double BLerror = BLMoveAngle - c4current;
    if (BLerror > 180) {
      BLerror -= 360;
    } else if (BLerror < -180) {
      BLerror += 360;
    }
    if (BLerror > 90) {
      BLerror -= 180;
      BLSpeed = -BLSpeed;
    } else if (BLerror < -90) {
      BLerror += 180;
      BLSpeed = -BLSpeed;
    }

    pods.s4set(pods.s4get() + (BLerror*43008/360));
    pods.d4set(BLSpeed*Constants.driveGain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
