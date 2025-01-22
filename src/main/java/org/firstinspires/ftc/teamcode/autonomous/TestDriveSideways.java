package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.DriveSideways;
import org.firstinspires.ftc.teamcode.util.events.IntakePiece;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;

@Disabled
@Autonomous(name = "TestDriveSideways", group="Linear OpMode")
public class TestDriveSideways extends OpModeBase {
    private LinearProcessor linProcess = null;

    @Override
    public void init(){
        super.init();
        this.linProcess = new LinearProcessor(new DriveSideways(24.0, this.robot));
        linProcess.init();
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }
        linProcess.shutdown();
    }
}
