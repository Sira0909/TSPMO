package org.firstinspire.ftc.teamcode.auto;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspire.ftc.teamcode.HardwareRobot;
import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous (name = "TSPMOAuto")
public class TSPMOAuto extends LinearOpMode {
    public RobotSystem robot;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.InitAprilTags(robot.hardwareRobot.camera);
        waitForStart();
        while (opModeIsActive()) {
            /// Coordinate System Explanation:
            /// Any detected tag immediately becomes the origin.
            /// The graph consists of the 1st and 2nd quadrants.
            /// Any negative returned by the x value of a tag detected by a camera will place the robot in the 1st quadrant.
            /// Returning positive x values for tag detection places the robot in the 2nd quadrant.
            /// Y can only be positive.

        }
    }
}
