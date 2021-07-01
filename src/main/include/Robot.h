// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>
#include <rev/CANEncoder.h>

class Robot : public frc::TimedRobot {
 public:
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4; // replace with actual motor IDs
  rev::CANSparkMax* m_leftLeadMotor = nullptr; 
  rev::CANSparkMax* m_rightLeadMotor = nullptr; 
  rev::CANSparkMax* m_leftFollowMotor = nullptr; 
  rev::CANSparkMax* m_rightFollowMotor = nullptr; 

  double left_y = 0.0;
  frc::XboxController* controller = nullptr;

  double distance = 0;
  double l_motor_rots = 0.0, r_motor_rots = 0.0, l_motor_circum = 0.0, r_motor_circum = 0.0, l_motor_dist = 0.0, r_motor_dist = 0.0;
  rev::CANEncoder* m_encoderSensor_left_motor = nullptr;
  rev::CANEncoder* m_encoderSensor_right_motor = nullptr;

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
