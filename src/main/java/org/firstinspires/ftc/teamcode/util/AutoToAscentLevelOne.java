package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name="AutoToAscentLevelOne",group="Autonomi")
public class AutoToAscentLevelOne extends OpMode {
    TritonRobot robot;
    LinearProcessor linearProcessor;

    @Override
    public void init() {
        robot = new TritonRobot(hardwareMap);
        linearProcessor = new LinearProcessor(
                new DriveForward(54,robot),
                new DriveSideways(6,robot)
        );
    }
    @Override
    public void loop() {
            linearProcessor.run();
            if(linearProcessor.isDone()){
                telemetry.addLine("Robot Is Finished With Task");
                telemetry.update();
            }
    }
}
