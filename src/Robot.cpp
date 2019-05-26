
#include <bits/stdc++.h>
#include <wpilib.h>
#include "ctre/Phoenix.h"
#include <utility>
#include <Spark.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

using namespace frc;

class Robot : public IterativeRobot {
public:
	void RobotInit() {
			LFmotor = new WPI_TalonSRX(lfnum);
			RFmotor = new WPI_TalonSRX(rfnum);
			LRmotor = new WPI_TalonSRX(lrnum);
			RRmotor = new WPI_TalonSRX(rrnum);

			catch_leftmotor =  new Spark(catch_left_channel);
			catch_rightmotor = new Spark(catch_right_channel);

			catch_rightmotor->SetInverted(true);
			catch_leftmotor->SetInverted(false);

			LFmotor->SetInverted(false);
			LRmotor->SetInverted(false);
			RFmotor->SetInverted(true);
			RRmotor->SetInverted(true);

			LFmotor->ConfigNominalOutputForward(0,pid_timeout);
			LFmotor->ConfigNominalOutputReverse(0,pid_timeout);
			RFmotor->ConfigNominalOutputForward(0,pid_timeout);
			RFmotor->ConfigNominalOutputReverse(0,pid_timeout);
			LRmotor->ConfigNominalOutputForward(0,pid_timeout);
			LRmotor->ConfigNominalOutputReverse(0,pid_timeout);
			RRmotor->ConfigNominalOutputForward(0,pid_timeout);
			RRmotor->ConfigNominalOutputReverse(0,pid_timeout);

			LFmotor->ConfigPeakOutputForward(motor_threshold, pid_timeout);
			LFmotor->ConfigPeakOutputReverse(-motor_threshold,pid_timeout);
			RFmotor->ConfigPeakOutputForward(motor_threshold, pid_timeout);
			RFmotor->ConfigPeakOutputReverse(-motor_threshold,pid_timeout);
			LRmotor->ConfigPeakOutputForward(motor_threshold, pid_timeout);
			LRmotor->ConfigPeakOutputReverse(-motor_threshold,pid_timeout);
			RRmotor->ConfigPeakOutputForward(motor_threshold, pid_timeout);
			RRmotor->ConfigPeakOutputReverse(-motor_threshold,pid_timeout);

			elevator_motor1=new WPI_TalonSRX(elevator_num1);
			elevator_motor2=new WPI_TalonSRX(elevator_num2);

			xbox = new XboxController(0);

			timer = new Timer();

			rightmotors = new SpeedControllerGroup(*RFmotor, *RRmotor);
			leftmotors = new SpeedControllerGroup(*LFmotor,*LRmotor);

			catchmotors = new SpeedControllerGroup(*catch_leftmotor , *catch_rightmotor);
			elevatormotors = new SpeedControllerGroup(*elevator_motor1,*elevator_motor2);

			RRmotor->ConfigSelectedFeedbackSensor(FeedbackDevice :: CTRE_MagEncoder_Absolute, slotid_right, pid_timeout);
			LRmotor->ConfigSelectedFeedbackSensor(FeedbackDevice :: CTRE_MagEncoder_Absolute, slotid_left , pid_timeout);

			RRmotor->SetSensorPhase(true);
			LRmotor->SetSensorPhase(false);

			RRmotor->SelectProfileSlot(slotid_right, 0);
			RRmotor->Config_kP(slotid_right, p, pid_timeout);
			RRmotor->Config_kI(slotid_right, i, pid_timeout);
			RRmotor->Config_kD(slotid_right, d, pid_timeout);

			LRmotor->SelectProfileSlot(slotid_left, 0);
			LRmotor->Config_kP(slotid_left, p, pid_timeout);
			LRmotor->Config_kI(slotid_left, i, pid_timeout);
			LRmotor->Config_kD(slotid_left, d, pid_timeout);

			cameraserver = CameraServer::GetInstance();
			cameraserver->StartAutomaticCapture();
			cameraserver->StartAutomaticCapture();

			accelerometer = new BuiltInAccelerometer();

			memset(drop_speed, false, sizeof(drop_speed));
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		timer->Reset();
		timer->Start();
		gamedata = DriverStation::GetInstance().GetGameSpecificMessage();
		if(gamedata[0]=='L'){

		}else if(gamedata[0]=='R'){

		}
	}

	void AutonomousPeriodic() {
		now_time = timer->Get();

		switch(now_time){
			case 0:
				before_data[0] = 0; // Xspeed
				before_data[1] = 0; // Zrotate
				PidDrive_Pos(before_data[0],before_data[1]);
				break;
			default:
				PidDrive_Pos(before_data[0],before_data[1]);
				break;
		}
	}

	void TeleopInit() {
		timer->Reset();
		timer->Start();
		LRmotor->SetSelectedSensorPosition(0, 0, 0);
		RRmotor->SetSelectedSensorPosition(0, 0, 0);
	}

	void TeleopPeriodic() {
		if(xbox->GetXButton())drop_speed[0] = true;
		else if(xbox->GetYButton())drop_speed[1] = true;
		//else if(xbox->GetAButton())Catch_box();

		MoveMotor(catchmotors, xbox->GetAButton(), xbox->GetBButton());
		MoveMotor(elevatormotors, xbox->GetBumper(XboxController::kLeftHand), xbox->GetBumper(XboxController::kRightHand));

		SmartDashboard::PutNumber("leftmotor_velocity", LRmotor->GetSensorCollection().GetPulseWidthVelocity());
		SmartDashboard::PutNumber("leftmotor_position", LRmotor->GetSensorCollection().GetQuadraturePosition());
		SmartDashboard::PutNumber("rightmotor_velocity",RRmotor->GetSensorCollection().GetPulseWidthVelocity());
		SmartDashboard::PutNumber("rightmotor_position",RRmotor->GetSensorCollection().GetQuadraturePosition());

		SmartDashboard::PutNumber("accel_X",accelerometer->GetX());
		SmartDashboard::PutNumber("accel_Y",accelerometer->GetY());
		SmartDashboard::PutNumber("accel_Z",accelerometer->GetZ());

		SmartDashboard::PutNumber("teleop",timer->Get());
	}

	void TestPeriodic() {
		MoveMotor(LFmotor,xbox->GetAButton());
		MoveMotor(RFmotor,xbox->GetAButton());

		SmartDashboard::PutNumber("test",1);
	}

/*------------------------------------------------------------------------------------------------------------------*/

	double Limitmotor(double value){
		if(value < 0)return std::max(value,-motor_threshold);
		else if(value >= 0)return std::min(value,motor_threshold);}

	void MoveMotor(SpeedController *motor , bool button){
		motor_speed = drop_speed[0]==1?slow_motor_speed:motor_speed;
		if (button)motor->Set(Limitmotor(motor_speed));
		else motor->Set(0);}

	void MoveMotor(SpeedController *motor, bool button, double speed){
		speed = drop_speed[0]==1?slow_motor_speed:speed;
		if (button)motor->Set(Limitmotor(speed));
		else motor->Set(0);}

	void MoveMotor(SpeedController *motor, bool plusbutton, bool minusbutton){
		motor_speed = drop_speed[0]==1?slow_motor_speed:motor_speed;
		if (plusbutton)motor->Set(Limitmotor(motor_speed));
		else if (minusbutton)motor->Set(Limitmotor(-motor_speed));
		else motor->Set(0);}

	void Catch_box(){
		//Catch Box.
	}

	double neutral(double value){
		if((value < 0.2 && value > 0) || (value > -0.2 && value < 0))return 0;
		return value;}


	void PidDrive_Vel(double xspeed , double zrotate ){
		double right_speed = Limitmotor(xspeed-zrotate);
		double left_speed =  Limitmotor(xspeed+zrotate);

		RRmotor->Set(ControlMode::Velocity,right_speed*pid_vel_const);
		RFmotor->Set(ControlMode::Follower,rrnum);
		LRmotor->Set(ControlMode::Velocity,left_speed* pid_vel_const);
		LFmotor->Set(ControlMode::Follower,lrnum);
	}

	void PidDrive_Pos(double xspeed , double zrotate ){
		double right_pos = Limitmotor(xspeed-zrotate);
		double left_pos =  Limitmotor(xspeed+zrotate);

		RRmotor->Set(ControlMode::Position,right_pos*pid_pos_const);
		RFmotor->Set(ControlMode::Follower,rrnum);
		LRmotor->Set(ControlMode::Position,left_pos* pid_pos_const);
		LFmotor->Set(ControlMode::Follower,lrnum);
	}

private:
	//drop_speed: 0 is for drive, 1 is for elevator. "catched" shows the status of the gripper.
	bool drop_speed[2], catched = 0;
	double slow_motor_speed = 0.3;			//Change this value for the lowest boost speed.
	double drive_speed[3] = { 0. };

	WPI_TalonSRX *LFmotor,*RFmotor,*LRmotor,*RRmotor, *elevator_motor1,*elevator_motor2;;
	PWMSpeedController *catch_leftmotor,*catch_rightmotor;
	SpeedControllerGroup *rightmotors,*leftmotors, *catchmotors, *elevatormotors;
	XboxController* xbox;
	MotorSafetyHelper* drivesafetyhelper;
	Timer* timer;
	BuiltInAccelerometer* accelerometer;
	SmartDashboard* sdb;
	CameraServer* cameraserver;
	std::string gamedata;

	const int lfnum = 0;
	const int rfnum = 1;
	const int lrnum = 2;
	const int rrnum = 3;
	const int elevator_num1 = 4;
	const int elevator_num2 = 5;
	const int catch_left_channel = 0;
	const int catch_right_channel = 1;

	const double motor_threshold = 0.6;
	const double motor_speed = 0.6;
	const double input_threshold = 0.2;

	double drive_speed_x, drive_speed_y, drive_speed_z;

	const int pid_timeout = 0;
	const int slotid_right = 0;
	const int slotid_left = 1;

	const double p = 0.1;
	const double i = 0;
	const double d = 0;
	const int pid_vel_const = 4096;
	const int pid_pos_const = 4096;

	int now_time;
	int before_data[2];
};

START_ROBOT_CLASS(Robot)
