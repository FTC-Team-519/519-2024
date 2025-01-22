package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;
//HIGHEST POSITION AT 15.5051 DEGREES(0.217volts), LOWEST POSITION AT 75 DEGREES(0.93volts)
// Uses encoders
public class ArmRotation implements Event {
    private double targetVoltage;
    //private double angleDifference;
    private double startingPosition;
    private TritonRobot robot;

    private double armSpeed = 1.0;

    public ArmRotation(double targetAngle, TritonRobot robot) {
        targetVoltage = targetAngle/81.1785929;
        // init robot
        this.robot = robot;
    }

    public ArmRotation(double targetAngle, TritonRobot robot, double speed) {
        armSpeed = speed;
        targetVoltage = targetAngle/81.1785929;
        this.robot = robot;
    }


    @Override
    public void init() {
        this.robot.setArmRotateMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startingPosition = robot.readAbsoluteEncoderValue();
        //this.robot.setArmTargetPosition((int)targetVoltage);
    }

    //    @Override
    public void run() {
        if(targetVoltage< robot.readAbsoluteEncoderValue()){
            robot.setArmRotatePower(armSpeed);
        }
        else if(targetVoltage> robot.readAbsoluteEncoderValue()){
            robot.setArmRotatePower(-armSpeed);
        }
    }

    private boolean angleDifference() {
        double angleDifference = Math.abs(robot.readAbsoluteEncoderValue()-targetVoltage);
        return angleDifference < 0.1;
    }

    private boolean checkTouchSensor(){
//        return robot.touchingBack() || robot.touchingFront();
        if(targetVoltage< startingPosition){
            return robot.touchingBack();
        }
        else {
            return robot.touchingFront();
        }
    }

    @Override
    public void shutdown() {
        robot.setArmRotatePower(0);
    }

    //    @Override
    public boolean isDone() {
        return angleDifference() || checkTouchSensor();
    }
}
