package org.firstinspire.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspire.ftc.teamcode.HardwareRobot;

public class LowClawSubsystem extends SubsystemBase {
    private final HardwareRobot HARDWARE_ROBOT;
    private double elbowPos;
    public double getElbowPos(){return elbowPos;}
    public LowClawSubsystem(
            HardwareRobot hardwareRobot,
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem
            //CVSubsystem cvSubsystem
    ) {
        HARDWARE_ROBOT = hardwareRobot;
    }
    public void setElbowPos(double ElbowPos) {
        HARDWARE_ROBOT.elbow.setPosition(ElbowPos);
    }
    public void setClawPos(double clawPos) {
        HARDWARE_ROBOT.claw.setPosition(clawPos);
    }
    public void setRotationPos(double rotationPos) {
        HARDWARE_ROBOT.clawrotation.setPosition(rotationPos);
    }
}
