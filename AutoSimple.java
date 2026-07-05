package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto", group="Auto")
public class AutoSimple extends LinearOpMode {

    // 声明电机
    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor indexerMotor = null;
    private DcMotor shooterDownMotor = null;
    private DcMotor shooterUpMotor = null;
    @Override
    public void runOpMode() {
        // 1. 硬件映射 (必须和手动模式的名字一样)
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "rear_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        indexerMotor = hardwareMap.get(DcMotor.class, "indexer");
        shooterDownMotor = hardwareMap.get(DcMotor.class, "shooter_down");
        shooterUpMotor = hardwareMap.get(DcMotor.class, "shooter_up");

        // 2. 设置电机方向 (这必须和你手动模式 init() 里的设置一模一样！)
        // 之前调试好的特殊设置：
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // 3. 设置刹车模式 (跑完立马停，比较准)
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        // 4. 等待开始 (按下 PLAY 键)
        waitForStart();

        intakeMotor.setPower(1);
        intakeMotor.setPower(1);
        // 5. 执行动作：向前移动 2 秒
        if (opModeIsActive()) {

            // --- 这里是关键 ---
            // 根据我们之前的调试，你的底盘似乎需要“负数功率”才会向前走
            // 所以这里我写的是 -0.5
            // 如果你发现车往后退，请把下面的 -0.5 全部改成 0.5 (正数)
            double power = -0.5;

            frontLeftDrive.setPower(power);
            rearLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            rearRightDrive.setPower(power);

            telemetry.addData("Auto", "Driving Forward...");
            telemetry.update();

            // 保持运行 2000 毫秒 (2秒)
            sleep(1000);

            // 6. 停车
            frontLeftDrive.setPower(0);
            rearLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearRightDrive.setPower(0);

            telemetry.addData("Auto", "Done");
            telemetry.update();
        }
    }
}