package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class indepSubsystem extends SubsystemBase {
    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;
    private final DriveSubsystem DRIVE;
    //private final CVSubsystem CV;


    public indepSubsystem(
            HardwareRobot hardwareRobot,
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem
            //CVSubsystem cvSubsystem
    ) {
        HARDWARE_ROBOT = hardwareRobot;
        OP_MODE = opMode;
        DRIVE = driveSubsystem;
        //CV = cvSubsystem;
    }

    public void setElbowPos(double ElbowPos) {
        HARDWARE_ROBOT.elbow.setPosition(ElbowPos);
    }

    public void setClawPos(double clawPos) {
        HARDWARE_ROBOT.claw.setPosition(clawPos);
    }




}