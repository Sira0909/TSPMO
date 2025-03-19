package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Config
@Autonomous(name="good auto (sample)")
public class basicAuto extends LinearOpMode{
    private HardwareRobot robot = new HardwareRobot(hardwareMap);
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(
                robot.rightFront,
                robot.rightBack,
                robot.leftFront,
                robot.leftBack
        );
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
