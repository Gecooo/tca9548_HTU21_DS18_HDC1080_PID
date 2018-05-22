void Pidregulator() {
  Setpoint = 6.00;
   Input = Seredtemp;
    double gap = abs(Setpoint-Input);  //определение агрессивной настройки PID- регулятора
    if(gap<1.50)
     {                                            
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
    if (Hottemp >=50.0)
     {
      myPID.SetMode(MANUAL);
      pwmWriteHR(INT1, 50);
      digitalWrite(INT2, LOW);                                                       
      }
     else {
      myPID.SetMode(AUTOMATIC); 
      pwmWriteHR(INT1, Output);
      digitalWrite(INT2, LOW);
  }
 // wdt_reset();
  
 // else {myPID.SetMode(MANUAL); pwmWriteHR(INT1, 0); digitalWrite(fan_in, LOW);}
   
}

