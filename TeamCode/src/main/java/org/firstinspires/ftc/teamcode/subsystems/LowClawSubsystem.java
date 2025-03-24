package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class LowClawSubsystem extends SubsystemBase {
    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;
    private final DriveSubsystem DRIVE;


    public LowClawSubsystem(
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

    public void setLeftLiftDirection(int leftLiftUp) {
        HARDWARE_ROBOT.leftLift.setTargetPosition(leftLiftUp); //tune
    }
    public void setRightLiftDirection(int rightLiftUp) {
        HARDWARE_ROBOT.rightLift.setTargetPosition(rightLiftUp); //tune
    }
}
