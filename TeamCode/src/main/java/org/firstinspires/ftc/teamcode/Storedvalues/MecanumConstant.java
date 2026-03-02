package org.firstinspires.ftc.teamcode.Storedvalues;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumConstant extends MecanumConstants {
    /** The Forward Velocity of the Robot - Different for each robot
     *  Default Value: 81.34056 */
    public  double xVelocity = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     *  Default Value: 65.43028 */
    public  double yVelocity = 65.43028;

    private  double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);

    /** The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     *  Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     * @implNote This vector should not be changed, but only accessed.
     */
    public  Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
    public  double maxPower = 1;
    public  String frontLeftDrive = "front_left_drive";
    public  String backLeftDrive = "back_left_drive";
    public  String frontRightDrive = "front_right_drive";
    public  String backRightDrive = "front_right_drive";
    public  DcMotorSimple.Direction frontLeftDriveMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction backLeftDriveMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction frontRightDriveMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction backRightDriveMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  double motorCachingThreshold = 0.01;
    public  boolean useBrakeModeInTeleOp = false;
    public  boolean useVoltageCompensation = false;
    public  double nominalVoltage = 12.0;
    public  double staticFrictionCoefficient = 0.1;

    public MecanumConstant() {
        defaults();
    }

    public MecanumConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public MecanumConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public MecanumConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public MecanumConstants frontLeftDrive (String frontLeftDrive ) {
        this.frontLeftDrive  = frontLeftDrive ;
        return this;
    }

    public MecanumConstants backLeftDrive (String backLeftDrive ) {
        this.backLeftDrive  = backLeftDrive ;
        return this;
    }

    public MecanumConstants frontRightDrive (String frontRightDrive ) {
        this.frontRightDrive  = frontRightDrive ;
        return this;
    }

    public MecanumConstants backRightDrive (String backRightDrive ) {
        this.backRightDrive  = backRightDrive ;
        return this;
    }

    public MecanumConstants frontLeftDriveMotorDirection(DcMotorSimple.Direction frontLeftDriveMotorDirection) {
        this.frontLeftDriveMotorDirection = frontLeftDriveMotorDirection;
        return this;
    }

    public MecanumConstants backLeftDriveMotorDirection(DcMotorSimple.Direction backLeftDriveMotorDirection) {
        this.backLeftDriveMotorDirection = backLeftDriveMotorDirection;
        return this;
    }

    public MecanumConstants frontRightDriveMotorDirection(DcMotorSimple.Direction frontRightDriveMotorDirection) {
        this.frontRightDriveMotorDirection = frontRightDriveMotorDirection;
        return this;
    }

    public MecanumConstants backRightDriveMotorDirection(DcMotorSimple.Direction backRightDriveMotorDirection) {
        this.backRightDriveMotorDirection = backRightDriveMotorDirection;
        return this;
    }

    public MecanumConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public MecanumConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public MecanumConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public MecanumConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public MecanumConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getfrontLeftDrive () {
        return frontLeftDrive ;
    }

    public void setfrontLeftDrive (String frontLeftDrive ) {
        this.frontLeftDrive  = frontLeftDrive ;
    }

    public String getbackLeftDrive () {
        return backLeftDrive ;
    }

    public void setbackLeftDrive (String backLeftDrive ) {
        this.backLeftDrive  = backLeftDrive ;
    }

    public String getfrontRightDrive () {
        return frontRightDrive ;
    }

    public void setfrontRightDrive (String frontRightDrive ) {
        this.frontRightDrive  = frontRightDrive ;
    }

    public String getbackRightDrive () {
        return backRightDrive ;
    }

    public void setbackRightDrive (String backRightDrive ) {
        this.backRightDrive  = backRightDrive ;
    }

    public DcMotorSimple.Direction getfrontLeftDriveMotorDirection() {
        return frontLeftDriveMotorDirection;
    }

    public void setfrontLeftDriveMotorDirection(DcMotorSimple.Direction frontLeftDriveMotorDirection) {
        this.frontLeftDriveMotorDirection = frontLeftDriveMotorDirection;
    }

    public DcMotorSimple.Direction getbackLeftDriveMotorDirection() {
        return backLeftDriveMotorDirection;
    }

    public void setbackLeftDriveMotorDirection(DcMotorSimple.Direction backLeftDriveMotorDirection) {
        this.backLeftDriveMotorDirection = backLeftDriveMotorDirection;
    }

    public DcMotorSimple.Direction getfrontRightDriveMotorDirection() {
        return frontRightDriveMotorDirection;
    }

    public void setfrontRightDriveMotorDirection(DcMotorSimple.Direction frontRightDriveMotorDirection) {
        this.frontRightDriveMotorDirection = frontRightDriveMotorDirection;
    }

    public DcMotorSimple.Direction getbackRightDriveMotorDirection() {
        return backRightDriveMotorDirection;
    }

    public void setbackRightDriveMotorDirection(DcMotorSimple.Direction backRightDriveMotorDirection) {
        this.backRightDriveMotorDirection = backRightDriveMotorDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public boolean isUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    /**
     * This method sets the default values for the MecanumConstants class.
     * It is called in the constructor of the MecanumConstants class.
     */
    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        frontLeftDrive  = "frontLeftDrive";
        backLeftDrive  = "backLeftDrive";
        frontRightDrive  = "frontRightDrive";
        backRightDrive  = "backRightDrive";
        frontLeftDriveMotorDirection = DcMotorSimple.Direction.REVERSE;
        backLeftDriveMotorDirection = DcMotorSimple.Direction.REVERSE;
        frontRightDriveMotorDirection = DcMotorSimple.Direction.FORWARD;
        backRightDriveMotorDirection = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }
}
