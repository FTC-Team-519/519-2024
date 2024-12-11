package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class OpModeBase extends OpMode {

    protected boolean isTimeToRun = true;

    protected TritonRobot robot;



    @Override
    public void init() {
        robot = new TritonRobot(hardwareMap);
    }

    @Override
    public abstract void loop();


    static protected double deadzoneAndSmoothstep(double raw_value, double limit, double steepness){
        if (Math.abs(raw_value) < limit){
            return 0.0d;
        }
        double output = (Math.abs(raw_value)-limit) / (1.0d - limit);
        double TwoPowerSteepness = Math.pow(2, steepness);
        double denom = TwoPowerSteepness*Math.pow(0.5, steepness + 1) + 0.5d;
        if (output>1.0){
            return 1.0d;
        }
        if (output<0.5){
            output = TwoPowerSteepness*Math.pow(output, steepness + 1)/denom;
        }else{
            output = 1.0d + TwoPowerSteepness*(output-1)*Math.pow(1-output, steepness)/denom;
        }
        Math.min(1.0d, Math.max(0.0d, output));
        if (raw_value<0) {
            output *= -1;
        }
        return output;
    }
}
