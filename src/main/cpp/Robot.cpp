/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */
#define _USE_MATH_DEFINES

// prototyping test definitions

// Careful: enabling the next define will turn on solenoid on auto climb switch
//#define TEST_CLIMB_SOLENOIDS

// Automation states
enum
{
	ARM_WRIST_MODE_MANUAL,
	ARM_WRIST_AUTO_LOW_HATCH,
	ARM_WRIST_AUTO_MID_HATCH,
	ARM_WRIST_AUTO_HIGH_HATCH,
	ARM_WRIST_AUTO_LOW_BALL,
	ARM_WRIST_AUTO_MID_BALL,
	ARM_WRIST_AUTO_HIGH_BALL
};

#define ARM_LOW_HATCH_SETPOINT 0.9
#define ARM_MID_HATCH_SETPOINT 0.0
#define ARM_HIGH_HATCH_SETPOINT -0.6
#define ARM_LOW_BALL_SETPOINT 0.5
#define ARM_MID_BALL_SETPOINT -0.2
#define ARM_HIGH_BALL_SETPOINT -0.8
#define WRIST_HATCH_SETPOINT 1.57
#define WRIST_BALL_SETPOINT 0.0
#define WRIST_HIGH_BALL_SETPOINT 0.7
#define DEAD_BAND 0.02 



#include <iostream>
#include <string>
#include <cmath>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot: public TimedRobot
{
private:
	// Declaring classes or variable we can use inside this class

	// state variables
	bool _f_startupok;
	bool _previousautoclimbswitch;

  // calibration variables
	double arm_sensor_calibration;
	double wrist_sensor_calibration;

	// These are all the sensors, joysticks and actuators for this robot
	WPI_VictorSPX * _leftFront = new WPI_VictorSPX(0);
	WPI_VictorSPX * _leftFollower = new WPI_VictorSPX(1);
	WPI_VictorSPX * _arm2 = new WPI_VictorSPX(2);
	WPI_VictorSPX * _arm1 = new WPI_VictorSPX(3);
	WPI_VictorSPX * _intake2 = new WPI_VictorSPX(4);
	WPI_VictorSPX * _intake1 = new WPI_VictorSPX(5);
	WPI_VictorSPX * _wrist = new WPI_VictorSPX(6);
	WPI_VictorSPX * _rightFollower = new WPI_VictorSPX(7);
	WPI_VictorSPX * _rightFront = new WPI_VictorSPX(8);
	WPI_VictorSPX * _climb = new WPI_VictorSPX(9);
	
	DifferentialDrive * _diffDrive = new DifferentialDrive(*_leftFront,	*_rightFront);

	Joystick * _joystick1 = new Joystick(0);
	Joystick * _joystick2 = new Joystick(1);

	AnalogPotentiometer * _wristangle = new AnalogPotentiometer(0, 2 * M_PI);
	AnalogPotentiometer * _armangle = new AnalogPotentiometer(1, 2 * M_PI);                                                                 

	Compressor *_compressor1 = new Compressor(0);

	frc::Solenoid _climb1 {7};
	frc::Solenoid _climb2 {6};
	frc::Solenoid _suction {5};
	frc::Solenoid _armsolenoid {4};

	DigitalInput * _climblimit = new DigitalInput(8);
	Encoder * _leftwheeldistance = new Encoder(0, 1);
	Encoder * _rightwheeldistance = new Encoder(2, 3);

  // timer resources	
	frc::Timer m_timer;
	frc::Timer _climbtimer;

	// diagnostic resources
	Faults _faults_L;
	Faults _faults_R;

public:
	void RobotInit() {
		/* factory default values */
		_rightFront->ConfigFactoryDefault();
		_rightFollower->ConfigFactoryDefault();
		_leftFront->ConfigFactoryDefault();
		_leftFollower->ConfigFactoryDefault();
		_arm2->ConfigFactoryDefault();
		_arm1->ConfigFactoryDefault();
		_intake1->ConfigFactoryDefault();
		_intake2->ConfigFactoryDefault();
		_wrist->ConfigFactoryDefault();
		_climb->ConfigFactoryDefault();

		/* set up followers */
		_rightFollower->Follow(*_rightFront);
		_leftFollower->Follow(*_leftFront);
		_arm2->Follow(*_arm1);
		_intake2->Follow(*_intake1);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		_rightFront->SetInverted(true);
		_rightFollower->SetInverted(true);
		//_leftFront->SetInverted(true);
		//_leftFollower->SetInverted(true);
		_intake2->SetInverted(true);

		_arm1->SetNeutralMode(Brake);
		_arm2->SetNeutralMode(Brake);

		/* [4] adjust sensor phase so sensor moves
		 * positive when LEDs are green */
		//Do we use this?
		_rightFront->SetSensorPhase(true);
		_leftFront->SetSensorPhase(true);

		/*
		* WPI drivetrain classes defaultly assume left and right are opposite. call
		* this so we can apply + to both sides when moving forward. DO NOT CHANGE
		*/
		_diffDrive->SetRightSideInverted(false);

		_compressor1->SetClosedLoopControl(true);

		// Setting sensor offset to a constant
//		arm_sensor_calibration = 1.68;
//		wrist_sensor_calibration = 2.07;

		arm_sensor_calibration = 1.5;
		wrist_sensor_calibration = 2.10;

		CameraServer::GetInstance()->StartAutomaticCapture();

		std::cout << " Robot Initialized " << std::endl;
	}

	void TeleopInit()
	{
		// This is to stop the robot from climbing when switch is on before it is enabled
		if (_joystick1->GetRawButton(1) ||
		    _joystick1->GetRawButton(4) ||
			_joystick1->GetRawButton(5))
		{
			_f_startupok = false;
			std::cout << " Dude! Check the switches! " << std::endl;
		}
		else
		{
			_f_startupok = true;
		}

		_previousautoclimbswitch = false;
	}

	void TeleopPeriodic()
	{
		std::stringstream work;

		if (!_f_startupok)
			return;

		/* get gamepad stick values */
		double _js_left_cmd = -1*  _joystick1->GetRawAxis(2); /* positive is forward */
		double _js_right_cmd =  -1* _joystick1->GetRawAxis(1); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(_js_left_cmd) < 0.10)
			_js_left_cmd = 0;
		if (fabs(_js_right_cmd) < 0.10)
			_js_right_cmd = 0;

		/* drive robot */
		_diffDrive->TankDrive(_js_left_cmd, _js_right_cmd, false);

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		//work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = _leftFront->GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront->GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = _leftFront->GetSelectedSensorVelocity(0);
		double rightVelUnitsPer100ms = _rightFront->GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rightVelUnitsPer100ms;

		
		_leftFront->GetFaults(_faults_L);
		_rightFront->GetFaults(_faults_R);

		if (_faults_L.SensorOutOfPhase) {
			work << " L sensor is out of phase";
		}
		if (_faults_R.SensorOutOfPhase) {
			work << " R sensor is out of phase";
		}

		/* print to console */
		std::cout << work.str() << std::endl;
	
		/* Intake/Outake */
		if (_joystick2->GetRawButton(7))
		{
			_intake1->Set (1.0);
		}
		else if (_joystick2->GetRawButton(8))
		{
			_intake1->Set (-1.0);
		}
		else 
		{
			_intake1->Set (0);
		}

		bool pressureSwitch = _compressor1->GetPressureSwitchValue();
		std::cout << " pressureswitch " << pressureSwitch << std::endl;

		/* Controlling solenoids */
	
		if (_joystick1->GetRawButton(2))
			_suction.Set(true);
		else
			 _suction.Set(false);

#ifdef TEST_CLIMB_SOLENOIDS
		if (_joystick1->GetRawButton(1))
		{
			_climb1.Set(true);
			_climb2.Set(true);
		}
		else
		{
			_climb1.Set(false);
			_climb2.Set(false);
		}
#endif		

		if (_joystick2->GetRawButton(10))
			_armsolenoid.Set(true);
		else 
			_armsolenoid.Set(false);

		bool climbLimitReached = !_climblimit->Get();
		std::cout << " climblimit " << climbLimitReached << std::endl;

		if (_joystick1->GetRawButton(4))
		{
			// raise the robot manually
			_climb->Set(1.0);
		}
		else if (_joystick1->GetRawButton(5))
		{
			// lower the robot manually
			_climb->Set(-0.5);
		}
		else
		{
			// automatic climbing
			if (_joystick1->GetRawButton(1) && !_previousautoclimbswitch)
			{
				// this is the transition from off to on for the climb auto switch
				
				// start timer for climb
				_climbtimer.Reset();
   				_climbtimer.Start();

				_climb->Set(1.0);
			}
			else if (_joystick1->GetRawButton(1) && _previousautoclimbswitch)
			{
				// this is where we are still climbing

				// check the timers, climber motor has to run from 0 to 3 seconds
				if (_climbtimer.Get() < 3.5) 
				{
  				// check the end switch to prevent serious damage to the bot
          if (climbLimitReached)
          {
       			// Stop the climber motor
     				_climb->Set(0.0);

            // retract the pneumatic cylinders, just in case we got here before the timer timed out
            _climb1.Set(false);
            _climb2.Set(false);
          }
          else
          {
       			// Full power on the climber motor
     				_climb->Set(1.0);
          }
   			} 
				else 
				{
     			// Stop the climber motor
     			_climb->Set(0);
				}

        // climb pneumatics need to be active from 0.7 to 1.9 seconds
        if ((_climbtimer.Get() > 0.7) && (_climbtimer.Get() < 1.9))
				{
          // extend the pneumatic cylinders
          _climb1.Set(true);
			    _climb2.Set(true);
   			} 
				else 
				{
          // retract the pneumatic cylinders
     		  _climb1.Set(false);
			    _climb2.Set(false);
				}

        // Drive backwards from 2 seconds to 2.4
        if ((_climbtimer.Get() > 2.0) && (_climbtimer.Get() < 2.4))
				{
          // Drive backward
          _diffDrive->TankDrive(-3.0, -3.0);
   			} 
				else 
				{
          // Stop driving
     		  _diffDrive->TankDrive(0.0, 0.0);
				}
			}
			else
			{
				// safety above all, default catch is turn the climber off
				// stop the climber unless automatic climbing was started
				_climb->Set(0);
        _climb1.Set(false);
			  _climb2.Set(false);
			}



			_previousautoclimbswitch = _joystick1->GetRawButton(1);
		}

		
		
		int leftwheeldistance = _leftwheeldistance->Get();
		std::cout << " leftwheeldistance " << leftwheeldistance << std::endl;
		int rightwheeldistance = _rightwheeldistance->Get();
		std::cout << " rightwheeldistance " << rightwheeldistance << std::endl;

		// determine how want to control the arm and the wrist	
		int auto_arm_wrist_ctrl_mode = ARM_WRIST_MODE_MANUAL;

		if  (_joystick2->GetRawButton(9) && _joystick2->GetRawButton(2))
		 	auto_arm_wrist_ctrl_mode =	ARM_WRIST_AUTO_LOW_HATCH;
		else if  (_joystick2->GetRawButton(9) && _joystick2->GetRawButton(3))
			auto_arm_wrist_ctrl_mode = ARM_WRIST_AUTO_MID_HATCH;
		else if (_joystick2->GetRawButton(9) && _joystick2->GetRawButton(4))
			auto_arm_wrist_ctrl_mode = ARM_WRIST_AUTO_HIGH_HATCH;
		else if (_joystick2->GetRawButton(2))
			auto_arm_wrist_ctrl_mode = ARM_WRIST_AUTO_LOW_BALL;
		else if (_joystick2->GetRawButton(3))
			auto_arm_wrist_ctrl_mode = ARM_WRIST_AUTO_MID_BALL;
		else if (_joystick2->GetRawButton(4))
			auto_arm_wrist_ctrl_mode = ARM_WRIST_AUTO_HIGH_BALL;

		// execute arm and wrist control
	  AutoControlWrist(auto_arm_wrist_ctrl_mode);
	}

  void AutonomousInit() override
	{
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
	{
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0)
		{
      // Drive forwards half speed
      _diffDrive->TankDrive(0.5, 0.5);
    }
		else
		{
      // Stop robot
      _diffDrive->TankDrive(0.0, 0.0);

			// and place the arm and wrist in mid hatch position
			AutoControlWrist(ARM_WRIST_AUTO_MID_HATCH);
    }
  }

private:
  void AutoControlWrist(int ctrl_mode)
	{
		// Read the Arm and Wrist Potentiometers
		double arm_sensor_angle = _armangle->Get();
		double wrist_sensor_angle = _wristangle->Get();
		std::cout << "arm_sensor_angle " << arm_sensor_angle;
		std::cout << " wrist_sensor_angle " << wrist_sensor_angle << std::endl;

		// Make the arm and js_wrist_command angle 0 when it is horizontal 
		double arm_angle = arm_sensor_angle - arm_sensor_calibration;
		double wrist_angle = wrist_sensor_angle - wrist_sensor_calibration;
		
		// Control the js_wrist_command
		std::cout << "arm_angle " << arm_angle;
		std::cout << " wrist_angle " << wrist_angle << std::endl;

		double _arm_error = 0.0;
		double _wrist_error = 0.0;

		switch (ctrl_mode)
		{
		case ARM_WRIST_MODE_MANUAL:
		  {
				// Joystick based Arm & Wrist control
				double js_arm_command = -1 * _joystick2->GetRawAxis(1);
				double js_wrist_command = -1 * _joystick2->GetRawAxis(3);

				// Apply deadband gamepad 10%
				if (fabs(js_arm_command) < 0.10)
					js_arm_command = 0;
				if (fabs(js_wrist_command) < 0.10)
					js_wrist_command = 0;

				// Control the motors
				_arm1->Set(js_arm_command);
				_wrist->Set(js_wrist_command);
				return;
			}
			break;
		case ARM_WRIST_AUTO_LOW_HATCH:
			// calculate the actual arm error
			_arm_error = arm_angle - ARM_LOW_HATCH_SETPOINT;
			// calculate the actual wrist error
			_wrist_error = wrist_angle - (arm_angle + WRIST_HATCH_SETPOINT);
			// adjust the arm and wrist
			AdjustArmAndWrist(_arm_error, _wrist_error);
			break;
		case ARM_WRIST_AUTO_MID_HATCH:
		// This takes it to the mid hatch position
			// calculate the actual arm error
			_arm_error = arm_angle - ARM_MID_HATCH_SETPOINT;
			// calculate the actual wrist error
			_wrist_error = wrist_angle - (arm_angle + WRIST_HATCH_SETPOINT);
			// adjust the arm and wrist
			AdjustArmAndWrist(_arm_error, _wrist_error);

			break;
		case ARM_WRIST_AUTO_HIGH_HATCH:
		// This takes it to the high hatch position
			// calculate the actual arm error
			_arm_error = arm_angle - ARM_HIGH_HATCH_SETPOINT;
			// calculate the actual wrist error
			_wrist_error = wrist_angle - (arm_angle + WRIST_HATCH_SETPOINT);
			// adjust the arm and wrist
			AdjustArmAndWrist(_arm_error, _wrist_error);

			break;
		case ARM_WRIST_AUTO_LOW_BALL:
			// calculate the actual arm error
			_arm_error = arm_angle - ARM_LOW_BALL_SETPOINT;
			// calculate the actual wrist error
			_wrist_error = wrist_angle - (arm_angle + WRIST_BALL_SETPOINT);
			// adjust the arm and wrist
			AdjustArmAndWrist(_arm_error, _wrist_error);

			break;
		case ARM_WRIST_AUTO_MID_BALL:
			// calculate the actual arm error
			_arm_error = arm_angle - ARM_MID_BALL_SETPOINT;
			// calculate the actual wrist error
			_wrist_error = wrist_angle - (arm_angle + WRIST_BALL_SETPOINT);
			// adjust the arm and wrist
			AdjustArmAndWrist(_arm_error, _wrist_error);

			break;
		case ARM_WRIST_AUTO_HIGH_BALL:
			// calculate the actual arm error
			_arm_error = arm_angle - ARM_HIGH_BALL_SETPOINT;
			// calculate the actual wrist error
			_wrist_error = wrist_angle - (arm_angle + WRIST_HIGH_BALL_SETPOINT);
			// adjust the arm and wrist
			AdjustArmAndWrist(_arm_error, _wrist_error);

			break;
		default:
  		// safe state is to turn the motors off
			_arm1->Set(0);
			_wrist->Set(0);
		  break;
		}

		//_arm1->Set(0);
		//_wrist->Set(0);
	}


  void AdjustArmAndWrist(double _arm_error, double _wrist_error)
	{
		// adjust the arm
		if (_arm_error > (0.0 + DEAD_BAND)) 
		{
			_arm1->Set(0.5);
		}
		else if (_arm_error < (0.0 - DEAD_BAND))
		{
			_arm1->Set(-0.5);
		}
		else 
		{
			_arm1->Set(0.0);
		}

		// adjust the wrist
		if (_wrist_error > (0.0 + DEAD_BAND)) 
		{
			_wrist->Set(-0.5);
		}
		else if (_wrist_error < (0.0 - DEAD_BAND))
		{
			_wrist->Set(0.5);
		}
		else 
		{
			_wrist->Set(0.0);
		}
	}

};

START_ROBOT_CLASS(Robot)