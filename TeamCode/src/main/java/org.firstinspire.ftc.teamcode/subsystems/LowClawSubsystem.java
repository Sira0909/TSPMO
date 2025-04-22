package org.firstinspire.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspire.ftc.teamcode.HardwareRobot;

public class LowClawSubsystem extends SubsystemBase {
    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;
    private final DriveSubsystem DRIVE;

    private double elbowPos;
    public double getElbowPos(){return elbowPos;}
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
        elbowPos = ElbowPos;
        HARDWARE_ROBOT.elbow.setPosition(ElbowPos);
    }

    public void setClawPos(double clawPos) {
        HARDWARE_ROBOT.claw.setPosition(clawPos);
    }

    public void setRightLift(double rightLift) {
        HARDWARE_ROBOT.rightLift.set(rightLift);
    }
    public void setLeftLift(double leftLift) {
        HARDWARE_ROBOT.leftLift.set(leftLift);
    }
    //only one side is needed bc they will always be the same
    public double getLeftLift() {
        double leftliftpos = HARDWARE_ROBOT.leftLift.getCurrentPosition();
        return leftliftpos;
    }
}
