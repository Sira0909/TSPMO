/*

Note: This code uses more Fundamental FTClib. In our latest codebases,
we adopted Syncropather for macros and a slightly
different organization style. We will reach out to explain
this new system in the coming weeks, as we want to share it
with our team's newer members, as well.

 */

package org.firstinspire.ftc.teamcode.CindysMomUseThis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //Allows us to create main opmode loop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; //Notation so that this opMode appears on the driver hub.


import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspire.ftc.teamcode.RobotSystem;

/// Controls:
/// Square: rotation, dpad left: claw, right x: turn, right y: elbow, left x: strafe, left y: forward.
@TeleOp (name = "Cindy's mom: Use This.")
public class TeleOpNoAprilTags extends LinearOpMode {
    public RobotSystem robot;
    public double rotationPos;
    public double clawPos;
    public int encoderPos;
    public double elbowPower;
    private boolean clawOpen = true;
    private boolean xPressedLastLoop = false;
    private boolean rotationDown = true;
    private boolean squarePressedLastLoop = false;
    public double speed = 0.45;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        waitForStart();
        while (opModeIsActive()) {
            encoderPos = robot.inDep.getEncoder(encoderPos);
            //Has to be reduced - will break elbow if at full power.
            elbowPower = gamepad1.right_stick_y * 0.5;
            //Issues with the drive led me to invert some of the joysticks - you may need to tweak these.
            double strafe = -gamepad1.left_stick_y;
            double forward = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            //The same problems occurred with the wheels.
            if (Math.abs(turn) > 0) {
                robot.hardwareRobot.changeInversions();
            }
            robot.drive.driveRobotCentricPowers(strafe * speed,forward * speed,turn * speed);
            /// Boolean Logic Explanation:
            /// The claw and rotation do NOT work off of toggle.
            /// Initially, both are false. Since the clawPos was set to closed earlier, it stays closed.
            /// The next loop, if the dpad is pressed, the condition holds. clawOpen's value flips, opening the claw.
            /// If it is let go and pressed again, the same thing happens, closing the claw.
            /// So, Separate presses are needed to alternate the claw's position.
            boolean isDpadPressed = gamepad1.dpad_right;
            if (isDpadPressed && !xPressedLastLoop) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    clawPos = RobotConstants.OPENCLAW;
                } else {
                    clawPos = RobotConstants.CLOSECLAW;
                }
            }
            //Same logic here.
            boolean squarePressed = gamepad1.square;
            if (squarePressed && !squarePressedLastLoop) {
                rotationDown = !rotationDown;
                if (rotationDown) {
                    rotationPos = RobotConstants.CLAWROTATIONPIKINGUP;
                } else {
                    rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
                }
            }
            //Setting all updated positions
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            //Sticks are -1 to 1, by the way.
            robot.inDep.setElbowPosition(elbowPower);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.addData("Encoder Position: ", encoderPos);
            //updating variables
            xPressedLastLoop = isDpadPressed;
            squarePressedLastLoop = squarePressed;
        }
    }
}
