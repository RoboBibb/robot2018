/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include "WPILib.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot : public frc::IterativeRobot {
public:

	// left side of drivetrain
	frc::PWMVictorSPX left0Mot{0}, left1Mot{1}, left2Mot{2};
	frc::SpeedControllerGroup lMots{ left0Mot, left1Mot, left2Mot };

	// right side of drivetrain
	frc::PWMVictorSPX right0Mot{3}, right1Mot{4}, right2Mot{5};
	frc::SpeedControllerGroup rMots{ right0Mot, right1Mot, right2Mot };

	// drive train object
	frc::DifferentialDrive drive{lMots, rMots};

	// xbox ctlr for main driver
	frc::Joystick driveCtl{0};
	frc::Joystick fxnCtl{1};

	frc::Talon armCtl{7};

	// gyro
	frc::ADXRS450_Gyro gyro;

	// pneumatics

	// claw to grab boxes
	frc::DoubleSolenoid grabber{0, 1};
	// launch cube from front
	frc::DoubleSolenoid flipper{7, 6};


	// autonomous chooser on DS
	frc::LiveWindow* m_lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoDriveStraight = "Drive Straight (no dump)";	  // drive forwards
	const std::string kAutoDriveStraightLeft = "drive straight (left)";   // starting on left side of field
	const std::string kAutoDriveStraightRight = "drive straight (right)"; // starting on right side of field
	std::string m_autoSelected;

	const std::string autos[4] = { kAutoNameDefault,
								kAutoDriveStraight,
								kAutoDriveStraightLeft,
								kAutoDriveStraightRight };



	Robot(){
		drive.SetExpiration(0.1);


	}

	void RobotInit(){

		std::cout <<"initializing...";
		gyro.Calibrate();

		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoDriveStraight, kAutoDriveStraight);
		m_chooser.AddObject(kAutoDriveStraightLeft, kAutoDriveStraightLeft);
		m_chooser.AddObject(kAutoDriveStraightRight, kAutoDriveStraightRight);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


		frc::SmartDashboard::PutNumber("auto mode\n"
				"<0> default (do nothing)\n"
				"<1> drive straight (no dump)\n"
				"<2> drive straight (left)\n"
				"<3> drive straight (right)", 0);

		std::cout <<"done\n";

	}

	// should be more accurate, but untested
	void driveStraight(frc::ADXRS450_Gyro& gyro, frc::DifferentialDrive& mots, const double time, const double speed = 0.5){
		const double DS_kP = 0.03, DS_CYCLETIME = 0.004;

		// get angle to maintain as zero
		gyro.Reset();

		// drive forward for the set ammount of time
		for (int i = (int) (time / (DS_CYCLETIME / abs(speed))); i > 0; i--) {
			// turn to correct heading
			mots.ArcadeDrive(speed, -gyro.GetAngle() * DS_kP); // add negatives for inverted steering/drive
			Wait(DS_CYCLETIME);
		}

		mots.ArcadeDrive(0.0, 0.0);

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

		// get chosen auto
		m_autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected << std::endl; // doesn't work (empty string)

		m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl; // doesn't work("default")



		// doesn't work anymore
		m_autoSelected = autos[(int) SmartDashboard::GetNumber("auto mode\n"
				"0 - default (do nothing)\n"
				"1 - drive straight (no dump)\n"
				"2 - drive straight (left)\n"
				"3 - drive straight (right)", 0)];
		std::cout << "Auto selected: " << m_autoSelected << std::endl; // default


		// enable motor controllers
		drive.SetSafetyEnabled(false);

		// drive straight dont dump
		if (m_autoSelected == kAutoDriveStraight) {
			driveStraight(gyro, drive, 4, -0.5);

		} else if (m_autoSelected == kAutoDriveStraightLeft) {
			driveStraight(gyro, drive, 4, -0.5);
			if(startLeft()) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}

		} else if (m_autoSelected == kAutoDriveStraightRight) {
			driveStraight(gyro, drive, 4, -0.5);
			if(!startLeft()) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}


		} else {


		}
		// Stop
		drive.ArcadeDrive(0, 0);
	}

	bool startLeft(){

		std::cout <<"checking if starting left...";
		std::string msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		while(msg.length() == 0){
			msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}

		std::cout <<"done\n";
		return msg[0] == 'L';

	}

	void AutonomousPeriodic(){
		if (m_autoSelected == kAutoDriveStraight) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}



	void TeleopInit(){}
	void TeleopPeriodic(){



		// reverse button
		static bool switchable = true;
		static float dir = 1;
		if (switchable && driveCtl.GetRawButton(1)) {
			dir = -dir;
			switchable = false;
		} else if (!switchable && !driveCtl.GetRawButton(1)) {
			switchable = true;
		}


		// arcade drive with 2 sticks & 80% turn speed
		drive.ArcadeDrive(
				(driveCtl.GetRawButton(2) ? 0.4 : 1 ) *
				dir * driveCtl.GetRawAxis(1),
				(driveCtl.GetRawButton(2) ? 0.8 : 1 ) *
				driveCtl.GetRawAxis(4) * 0.8);

		// flipper control
		if (fxnCtl.GetRawAxis(3) > 0.75) {
			flipper.Set(frc::DoubleSolenoid::kForward);
		} else {
			flipper.Set(frc::DoubleSolenoid::kReverse);
		}

		// grabber control
		if (fxnCtl.GetRawButton(1)) {
			grabber.Set(frc::DoubleSolenoid::kForward);

		} else if (fxnCtl.GetRawButton(2)) {
			grabber.Set(frc::DoubleSolenoid::kReverse);
		}


		// elevator ctl
		if (fxnCtl.GetRawButton(6)) {
			armCtl.Set(-.5);
		} else {
			armCtl.Set(fxnCtl.GetRawAxis(2) - 0.5);
		}




	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
