package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;

public class ArmStall implements Event{
    private double targetVoltage;
    //private double angleDifference;
    private double startingPosition;
    private TritonRobot robot;

    private boolean at_position;

    public ArmStall(double targetAngle, TritonRobot robot) {
        targetVoltage = targetAngle/81.1785929;

        // init robot
        this.robot = robot;
    }

    @Override
    public void init() {
        this.robot.setArmRotateMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startingPosition = robot.readAbsoluteEncoderValue();
        at_position = false;
        //this.robot.setArmTargetPosition((int)targetVoltage);
    }

    //    @Override
    public void run() {
        if (this.checkTouchSensor()){
            //noop
        }else if (!at_position){
            if(targetVoltage< robot.readAbsoluteEncoderValue()){
                robot.setArmRotatePower(1);
            }
            else if(targetVoltage>robot.readAbsoluteEncoderValue()){
                robot.setArmRotatePower(-1);
            }
            if (angleDifference()){
                at_position = true;
            }
        }else{
            if(targetVoltage< robot.readAbsoluteEncoderValue()){
                robot.setArmRotatePower(0.1);
            }
        }
    }

    private boolean angleDifference() {
        double angleDifference = Math.abs(robot.readAbsoluteEncoderValue()-targetVoltage);
        return angleDifference < 0.1;
    }

    private boolean checkTouchSensor(){
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
        return false;
    }
}
