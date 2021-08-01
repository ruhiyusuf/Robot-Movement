// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

void Robot::RobotInit() {
  this->m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  this->m_leftLeadMotor->RestoreFactoryDefaults();
  this->m_rightLeadMotor->RestoreFactoryDefaults();
  this->m_leftFollowMotor->RestoreFactoryDefaults();
  this->m_rightFollowMotor->RestoreFactoryDefaults();

  this->m_leftLeadMotor->SetInverted(true);
  this->m_leftFollowMotor->Follow(*this->m_leftLeadMotor);
  this->m_rightLeadMotor->SetInverted(false);
  this->m_rightFollowMotor->Follow(*this->m_rightLeadMotor);

  this->m_leftLeadMotor->SetSmartCurrentLimit(40);
  this->m_rightLeadMotor->SetSmartCurrentLimit(40);
  this->m_leftFollowMotor->SetSmartCurrentLimit(40);
  this->m_rightFollowMotor->SetSmartCurrentLimit(40);

  this->m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  this->m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  this->m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  this->m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  this->m_leftLeadMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  this->m_rightLeadMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  this->m_leftFollowMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  this->m_rightFollowMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);

  this->m_leftLeadMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  this->m_rightLeadMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  this->m_leftFollowMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  this->m_rightFollowMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);

  this->m_leftLeadMotor->GetEncoder().SetPositionConversionFactor(wheel2GearR);
  this->m_rightLeadMotor->GetEncoder().SetPositionConversionFactor(wheel2GearR);

  // this->m_robotDrive = new frc::DifferentialDrive(*this->m_leftLeadMotor, *this->m_rightLeadMotor);

  this->controller = new frc::XboxController{0}; // replace with USB port number on driver station
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", this->controller->GetY(left_analog));
  frc::SmartDashboard::PutNumber("right x: ", this->controller->GetX(right_analog));
  frc::SmartDashboard::PutNumber("left wheel rotations: ", this->m_leftLeadMotor->GetEncoder().GetPosition());
  frc::SmartDashboard::PutNumber("right wheel rotations: ", this->m_rightLeadMotor->GetEncoder().GetPosition());
  frc::SmartDashboard::PutNumber("left wheel distance (ft): ", this->m_leftLeadMotor->GetEncoder().GetPosition() * wheel_circum);
  frc::SmartDashboard::PutNumber("right wheel distance (ft): ", this->m_rightLeadMotor->GetEncoder().GetPosition() * wheel_circum);
}

void Robot::AutonomousInit() {
  kP = 0.1;                             // needs tuning
  setpoint = lError = rError = 0.0;
  lSpeed = rSpeed = 0.0;
}
void Robot::AutonomousPeriodic() {
  if (this->controller->GetAButton())
    setpoint = 5.0;
  else if (this->controller->GetBButton()) {
    kP = 0.0;
    // this->m_robotDrive->StopMotor();
  }

  l_wheel_rots = this->m_leftLeadMotor->GetEncoder().GetPosition();
  r_wheel_rots = this->m_rightLeadMotor->GetEncoder().GetPosition();

  l_wheel_dist = l_wheel_rots * wheel_circum;
  r_wheel_dist = r_wheel_rots * wheel_circum;

  lError = setpoint - l_wheel_dist;
  rError = setpoint - r_wheel_dist;

  lSpeed = kP * lError;
  rSpeed = kP * rError;

  // this->m_robotDrive->TankDrive(lSpeed, rSpeed);
  
  this->m_leftLeadMotor->Set(lSpeed);
  this->m_rightLeadMotor->Set(rSpeed);
}

void Robot::TeleopInit() {
  // this->m_robotDrive->SetDeadband(0.05);
}
void Robot::TeleopPeriodic() {
  left_y = this->controller->GetY(left_analog);
  right_x = this->controller->GetX(right_analog);
  
  // this->m_robotDrive->ArcadeDrive(-left_y, right_x, true);

  this->m_leftLeadMotor->Set(-left_y);
  this->m_rightLeadMotor->Set(-left_y);
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
