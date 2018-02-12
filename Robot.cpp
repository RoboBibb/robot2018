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

	frc::ADXRS450_Gyro gyro;



	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoDriveStraight = "My Auto";
	std::string m_autoSelected;



	void RobotInit() {
		gyro.Calibrate();
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoDriveStraight, kAutoDriveStraight);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoDriveStraight) {
			double center = gyro.GetAngle();
			// drive in direction of base angle 1000 times
			for(unsigned i = 0; i < 1000; i++){
				// redirect towards base angle
				drive.ArcadeDrive(.5, (gyro.GetAngle() - center) / 45);
				// drive straight 2 seconds / 1000 times
				Wait(2 / 1000);

			}
		} else {
			// Default Auto goes here
		}
		// Stop
		drive.ArcadeDrive(0, 0);
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoDriveStraight) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}



	void TeleopInit() {}
	void TeleopPeriodic() {


		// arcade drive with 2 sticks & 80% turn speed
		drive.ArcadeDrive(driveCtl.GetRawAxis(1), driveCtl.GetRawAxis(4) * 0.8);


	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
