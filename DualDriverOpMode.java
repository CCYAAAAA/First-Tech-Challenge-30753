package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Double Driver Fixed (Logic Swapped)", group="Teleop")
public class DualDriverOpMode extends OpMode {

    // 参数设置
    private static final double MAX_SHOOTER_POWER = 0.65;
    private static final double SHOOTER_DOWN_RATIO = 1.7;

    // 底盘参数
    private static final double STRAFE_CORRECTION = 1.1;
    // 旋转灵敏度建议稍微调高，因为这种特殊结构的旋转摩擦力较大
    private static final double TURN_SENSITIVITY = 1.0;
    private static final double FINE_TURN_SENSITIVITY = 0.3;

    // 电机声明
    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;

    private DcMotor intakeMotor = null;
    private DcMotor indexerMotor = null;
    private DcMotor shooterDownMotor = null;
    private DcMotor shooterUpMotor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // 1. 硬件映射
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "rear_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right");

        intakeMotor     = hardwareMap.get(DcMotor.class, "intake");
        indexerMotor    = hardwareMap.get(DcMotor.class, "indexer");
        shooterDownMotor = hardwareMap.get(DcMotor.class, "shooter_down");
        shooterUpMotor = hardwareMap.get(DcMotor.class, "shooter_up");

        // 2. 方向设置 (保持我们在单人模式测试通过的设置)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooterDownMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterUpMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);

        // 3. 刹车模式
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - Special Logic Applied");
    }

    @Override
    public void loop() {
        // =========================================================
        //           底盘控制 (已同步单人模式的修复逻辑)
        // =========================================================

        // 1. 获取主驾驶 (Gamepad 1) 输入 - 轴向互换
        // 左摇杆 X -> 控制前后 (Axial)
        double drive  = gamepad1.left_stick_x;

        // 左摇杆 Y -> 控制左右平移 (Lateral) (注意加负号)
        double strafe = -gamepad1.left_stick_y * STRAFE_CORRECTION;

        // 右摇杆 X -> 控制旋转
        double mainTurn   = gamepad1.right_stick_x * TURN_SENSITIVITY;

        // 2. 获取副驾驶 (Gamepad 2) 输入 - 微调旋转
        // 副驾驶通常用左摇杆或者右摇杆来微调，这里沿用你之前的 left_stick_x
        // 如果方向反了（向左推变右转），把前面的负号去掉即可
        double fineTurn = -gamepad2.left_stick_x * FINE_TURN_SENSITIVITY;

        // 3. 合并旋转量
        double totalTurn = mainTurn + fineTurn;

        // 4. 运动学公式 (特殊修正版：前后分组)
        // 你的机器人结构导致前半部分(FL+FR)和后半部分(RL+RR)必须反向才能旋转

        double flPower = drive + strafe + totalTurn; // 前左：加旋转
        double frPower = drive - strafe + totalTurn; // 前右：加旋转 (注意这里变了)

        double rlPower = drive - strafe - totalTurn; // 后左：减旋转 (注意这里变了)
        double rrPower = drive + strafe - totalTurn; // 后右：减旋转

        // 5. 归一化
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(rlPower),
                Math.max(Math.abs(frPower), Math.abs(rrPower))));

        if (max > 1.0) {
            flPower /= max;
            rlPower /= max;
            frPower /= max;
            rrPower /= max;
        }

        frontLeftDrive.setPower(flPower);
        rearLeftDrive.setPower(rlPower);
        frontRightDrive.setPower(frPower);
        rearRightDrive.setPower(rrPower);

        // =========================================================
        //                 2号手柄 - 射击与机构 (保持不变)
        // =========================================================

        double rawTriggerInput = Math.max(gamepad2.left_trigger, gamepad2.right_trigger);
        double masterShooterPower = rawTriggerInput * MAX_SHOOTER_POWER;

        double pwrUp = masterShooterPower;
        double pwrDown = masterShooterPower * SHOOTER_DOWN_RATIO;

        shooterUpMotor.setPower(pwrUp);
        shooterDownMotor.setPower(pwrDown);

        // Intake / Indexer
        double intakePower = 0.0;
        if (gamepad2.a) intakePower = 1.0;
        else if (gamepad2.b) intakePower = -1.0;

        double indexerPower = 0.0;
        if (gamepad2.x) indexerPower = 1.0;
        else if (gamepad2.y) indexerPower = -1.0;

        intakeMotor.setPower(intakePower);
        indexerMotor.setPower(indexerPower);

        // =========================================================
        //                      遥测显示
        // =========================================================
        telemetry.addData("Drive Logic", "Axis Swapped + Front/Back Split");
        telemetry.addData("Inputs", "Fwd:%.2f, Str:%.2f, Turn:%.2f", drive, strafe, totalTurn);
        telemetry.update();
    }
}