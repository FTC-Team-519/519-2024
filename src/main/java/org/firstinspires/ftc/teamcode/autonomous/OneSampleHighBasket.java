package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name = "OneSampleHighBasket", group="Linear OpMode")
public class OneSampleHighBasket extends OpModeBase {
    private LinearProcessor linProcess = null;

    @Override
    public void init() {
        ///robot.resetSpoolEncoders();
        super.init();
        this.linProcess = new LinearProcessor(
            new ParallelRaceProcessor(
                new ArmStall(17, this.robot),
                new LinearProcessor(
                    new DriveSideways(6.0, this.robot),
                    new DriveForward(7.0, this.robot),
                    new ParallelProcessor(
                        new Rotate(-30.0, false, 1.0, this.robot),
                        new ArmExtension(-4000, 1, this.robot)
                    ),
                    new DriveForward(2.0, this.robot),
                    new DriveSideways(5.0, this.robot),

                    new IntakePiece(this.robot,1.0, false),

                    new DriveForward(-2.0, this.robot)
                    //new ArmExtension(-50, this.robot)
                )
            ),
               new LinearProcessor(
                       new ParallelRaceProcessor(
                               new ParallelProcessor(
                                       new ArmRotation(64.0, this.robot),
                                       new Rotate(106, true, 0.75, this.robot)
                               ),
                               new ArmExtension(-50, 1, this.robot)
                    ),
                    new ParallelRaceProcessor(
                        new ArmExtension(-3512, 0.2, this.robot),
                        new IntakePiece(this.robot, 1.0, true)
                    )
            )
        );
        this.linProcess.init();
        this.robot.setSpoolMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setSpoolMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }
//        linProcess.shutdown();
        telemetry.addData("Left Spool Readout: ",robot.getCurrentLeftSpoolPosition());
        // This section should return the current voltage
        telemetry.addData("Potentiometer Output: ",robot.readAbsoluteEncoderValue());
        telemetry.addData("Degree From Back: ",robot.readAbsoluteEncoderValueAsDegrees());
    }
}
