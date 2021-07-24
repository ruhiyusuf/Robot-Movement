// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>
#include <rev/CANEncoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <math.h>

class Robot : public frc::TimedRobot {
 public:
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4; // replace with actual motor IDs

  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  frc::DifferentialDrive* m_robotDrive = new frc::DifferentialDrive(*m_leftLeadMotor, *m_rightLeadMotor);

  double max_speed = 0.0;
  double left_y = 0.0, right_x = 0.0;
  frc::XboxController* controller = new frc::XboxController{0}; // replace with USB port number on driver station
  frc::GenericHID::JoystickHand left_analog {frc::GenericHID::kLeftHand};
  frc::GenericHID::JoystickHand right_analog {frc::GenericHID::kRightHand};

  double setpoint = 0.0;
  const double l_wheel_circum = 5.7 * M_PI, r_wheel_circum = 5.7 * M_PI;    
  double l_wheel_rots = 0.0, r_wheel_rots = 0.0, l_wheel_dist = 0.0, r_wheel_dist = 0.0;

  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

};
