package org.firstinspire.ftc.teamcode;



/**
 * A static class used by the robot to reference important kinematic and algorithmic tuning values.
 */
public final class DriveConstants {


    /**
     *  Max velocity driving forward in in/s.
     */
    public static double MAX_FORWARD_SPEED = 57.5d; //TODO: TUNE
    /**
     * Max velocity driving sideways in in/s.
     */
    public static double MAX_STRAFE_SPEED = 40d; //TODO: TUNE
    /**
     * Angle of the output force vector of the robot's Mecanum wheels in radians.
     */
    public static final double THETA_WHEEL = Math.atan2(Math.abs(MAX_FORWARD_SPEED), Math.abs(MAX_STRAFE_SPEED));


    /**
     *  Max velocity of the robot in in/s.
     */
    public static final double MAX_VELOCITY = Math.min(MAX_FORWARD_SPEED, MAX_STRAFE_SPEED);
    /**
     *  Max acceleration of the robot in in/s^2.
     */
    public static double MAX_ACCELERATION = 54d; //TODO: TUNE

    /**
     *  Max angular velocity of the robot in rad/s.
     */
    public static double MAX_ANGULAR_VELOCITY = 3.6; //TODO: TUNE
    /**
     *  Max angular acceleration of the robot in rad/s^2.
     */
    public static double MAX_ANGULAR_ACCELERATION = 4; //TODO: TUNE

}