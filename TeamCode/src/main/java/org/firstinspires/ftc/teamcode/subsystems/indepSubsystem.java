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


    public void setElbowPos(double ElbowPos) {
        LOWCLAW.setElbowPos(ElbowPos);
    }
    public void adjustElbowPos(double ElbowPos){
        LOWCLAW.setElbowPos(LOWCLAW.getElbowPos()+ElbowPos);
    }

    public void openClaw(){
        LOWCLAW.setClawPos(ROBOTCONSTANTS.OPENCLAW);
    }
    public void closeClaw(){
        LOWCLAW.setClawPos(ROBOTCONSTANTS.CLOSECLAW);
    }
    public void LiftUp(){//sets lift to go up at max speed
        LOWCLAW.setLeftLiftDirection(ROBOTCONSTANTS.LIFTUPCONST);
        LOWCLAW.setRightLiftDirection(ROBOTCONSTANTS.LIFTUPCONST);
    }
    public void LiftDown(){//sets lift to go down at max speed
        LOWCLAW.setLeftLiftDirection(ROBOTCONSTANTS.LIFTDOWNCONST);
        LOWCLAW.setRightLiftDirection(ROBOTCONSTANTS.LIFTDOWNCONST);
    }
    public  void SetLiftDir(int direction){
        LOWCLAW.setRightLiftDirection(direction);
        LOWCLAW.setLeftLiftDirection(direction);
    }





}