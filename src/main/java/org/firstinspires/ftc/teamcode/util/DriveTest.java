package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.events.ArmExtension;
import org.firstinspires.ftc.teamcode.util.events.ArmRotation;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;
import org.firstinspires.ftc.teamcode.util.events.Rotate;

@Autonomous(name="Drive Test",group="Autonomi")
public class DriveTest extends OpMode {
    TritonRobot robot;
    LinearProcessor linearProcessor;

    @Override
    public void init() {
        robot = new TritonRobot(hardwareMap);
        linearProcessor = new LinearProcessor(
                new Rotate(90, true, robot)
                //new ArmExtensionDown(0, robot)
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
