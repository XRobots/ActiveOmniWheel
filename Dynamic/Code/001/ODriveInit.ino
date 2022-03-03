

void OdriveInit1() {

      Serial.println("ODrive 1");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 1000.0f << '\n';
          Serial1 << "w axis" << axis << ".motor.config.current_lim " << 25.0f << '\n';

          // motor & encoder are pre-calibration through ODrive tool

          //requested_state = AXIS_STATE_MOTOR_CALIBRATION;
          //Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          //odrive1.run_state(axis, requested_state, true);
    
          //requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          //Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          //odrive1.run_state(axis, requested_state, true);
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(axis, requested_state, false); // don't wait 

          Serial1 << "w axis" << 0 << ".controller.config.vel_gain " << 0.2 << '\n';
          Serial1 << "w axis" << 1 << ".controller.config.vel_gain " << 0.2 << '\n';

          Serial1 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << 10 << '\n';
          Serial1 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << 10 << '\n';
      }   

}



