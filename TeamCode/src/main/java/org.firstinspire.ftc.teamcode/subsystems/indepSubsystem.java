package org.firstinspire.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspire.ftc.teamcode.HardwareRobot;
import org.firstinspire.ftc.teamcode.RobotConstants;

import org.firstinspire.ftc.teamcode.RobotConstants;
public class indepSubsystem extends SubsystemBase {

    private final HardwareRobot HARDWAREROBOT;
    //private final CVSubsystem CV;

    public indepSubsystem(
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem,
            HardwareRobot hardwareRobot
            //CVSubsystem cvSubsystem
    ) {
        HARDWAREROBOT = hardwareRobot;
    }

    public void setElbowPosition(double Elbowppos) {
        HARDWAREROBOT.elbow.motor.setPower(Elbowppos);
        HARDWAREROBOT.elbowtwo.motor.setPower(Elbowppos);
    }
    public void setClawPosition(double clawPos) {
        HARDWAREROBOT.claw.setPosition(clawPos);
    }
    public void setRotationPosition(double rotationPos) {
        HARDWAREROBOT.clawrotation.setPosition(rotationPos);
    }
    public int getEncoder(int encoder) {
        encoder = HARDWAREROBOT.elbow.getCurrentPosition();
        return encoder;
    }
}