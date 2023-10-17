// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class RobotState {

    private static RobotState robotState;
    public enum GamePiece{
        CONE,
        CUBE, 
        EMPTY
    }

    public enum AutoState{
        NORMAL,
        BALANCING,
        BALANCED,
        NOAUTO
    }


    public enum SwerveState{
        MOVING,
        REST
    }

    public static Integer currentPipeline = 0;

    public static GamePiece currentGamePiece = GamePiece.EMPTY;
    public static SwerveState currentSwerveState = SwerveState.REST;
    public static AutoState currentAutoState = AutoState.NOAUTO;
    public static double balanceGyroAngle=0;

    public static boolean isElevated = false;
    public static boolean isTripping = false;
    public static boolean isGripperReverse = true;
    public static boolean isIMUalive = false;
    
    private RobotState(){
        //reset(); 
    }

    

    public static boolean isTeleop(){
        return edu.wpi.first.wpilibj.RobotState.isTeleop();
    }

    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance(){
        return DriverStation.getAlliance() == Alliance.Red;
    }

    public static void setPipeline(Integer pipline){
        currentPipeline = pipline;
    }
       
    public static void setGamePiece(GamePiece gamePiece){
        if(currentGamePiece != gamePiece) currentGamePiece = gamePiece;
    }

    public static void setSwerve(SwerveState swerveState){
        currentSwerveState = swerveState;
    }


   

 

    public static void reset(){
        currentGamePiece = null;
    }

    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }



}
