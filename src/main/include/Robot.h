// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include "SFDrive.h"

class Robot : public frc::TimedRobot {
 public:
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

  static const int rightLeadDeviceID = 1;
  static const int rightFollowDeviceID = 2;
  static const int leftLeadDeviceID = 3;
  static const int leftFollowDeviceID = 4;

  double m_P = 0, m_I = 0, m_D = 0;
  double kMinOutput = 0, kMaxOutput = 0;

  double errorL = 0, errorR = 0, setpoint = 0;

  double left_y = 0.0;
  double right_x = 0.0;

  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANEncoder m_leftEncoder = m_leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);

  rev::CANPIDController m_leftPIDController = m_leftLeadMotor->GetPIDController();
  rev::CANPIDController m_rightPIDController = m_rightLeadMotor->GetPIDController();

  frc::Joystick* m_stick = new frc::Joystick{0};

  SFDrive* m_robotDrive = new SFDrive(m_leftLeadMotor, m_rightLeadMotor);

};