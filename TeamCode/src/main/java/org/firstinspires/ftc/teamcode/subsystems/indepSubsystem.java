package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotConstants;
public class indepSubsystem extends SubsystemBase {
    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;
    private final DriveSubsystem DRIVE;
    private final LowClawSubsystem LOWCLAW;
    private final RobotConstants ROBOTCONSTANTS=new RobotConstants();
    //private final CVSubsystem CV;

    public indepSubsystem(
            HardwareRobot hardwareRobot,
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem,
            LowClawSubsystem LowClaw
            //CVSubsystem cvSubsystem
    ) {
        HARDWARE_ROBOT = hardwareRobot;
        OP_MODE = opMode;
        DRIVE = driveSubsystem;
        LOWCLAW = LowClaw;
        //CV = cvSubsystem;
    }




    public void openClaw(){
        LOWCLAW.setClawPos(ROBOTCONSTANTS.OPENCLAW);
    }
    public void closeClaw(){
        LOWCLAW.setClawPos(ROBOTCONSTANTS.CLOSECLAW);
    }
    public void LiftUp(){
        LOWCLAW.setLeftLiftDirection(ROBOTCONSTANTS.LIFTUPCONST);
        LOWCLAW.setRightLiftDirection(ROBOTCONSTANTS.LIFTUPCONST);
    }
    public void setLiftPos(double LiftUp){
        LOWCLAW.setLeftLiftDirection(ROBOTCONSTANTS.LIFTDOWNCONST);
        LOWCLAW.setRightLiftDirection(ROBOTCONSTANTS.LIFTDOWNCONST);
    }





}