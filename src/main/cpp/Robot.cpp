// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  std::cout << "test" << std::endl;
  compressor = new frc::Spark(1);
}
void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  int test = 76;
  m_robotDrive->ArcadeDrive(-left_y, right_x);

  analog_input->GetVoltage();
  frc::SmartDashboard::PutNumber("analogInput", analog_input->GetVoltage());
  frc::SmartDashboard::PutNumber("testNum", test);

  var_input = frc::SmartDashboard::GetNumber("varInput", 1);
  frc::SmartDashboard::PutNumber("varInput", var_input);

  maxPSI = frc::SmartDashboard::GetNumber("maxPSI", 50);
  frc::SmartDashboard::PutNumber("maxPSI", maxPSI);

  // maxPSI = 100; 

  PSI = (analog_input->GetVoltage()) * 100 + 10; // transfer function
  if (!reached_max_pressure) {
    if (PSI < maxPSI) {
      frc::SmartDashboard::PutNumber("currPSI", PSI);
      compressor->Set(1);
    } else {
      compressor->Set(0);
      reached_max_pressure = true; 
    }
  }

  
  // if button pressed,
  // include if statement
  if (m_stick->GetTopPressed()) {
    if (PSI > maxPSI) {
      compressor->Set(1);
    } else {
      compressor->Set(0);
    }
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
