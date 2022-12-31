//PID Control Template

// *********************************************************************************************************************** //
// Variables                                                                                                               //
// *********************************************************************************************************************** //

//pin definition
#define ENA 9
#define IN1 21
#define IN2 20
#define SENSOR_PIN A0

// ******** <TODO> **********************
// ******** define interval between recomputing error and adjusting feedback (in milliseconds) ********************** 
const int INTERVAL = 10; 

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double cumError, rateError;

int motorSpeed = 100; // speed of the motor, values between 0 and 255
int target = 512; // position (as read by potentiometer) to move the motor to, default value 512

// ******** <TODO> **********************
// ******** define the different gains **********************
float kp = 0.0; // proportional gain
float ki = 0.0; // integral gain
float kd = 0.0; // derivative gain

int pos = 0; // current position for plotting
//serial communication variables
float PID_values[4];
byte i = 0;
char record[100];
char recvchar;
byte indx = 0;

// setup code, setting pin modes and initialising the serial connection
void setup() 
{
    Serial.begin(115200);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);    
    pinMode(SENSOR_PIN, INPUT);   
}

void loop() 
{
        //  ******** <TODO> **********************
        //  ******** implement your code  here **********************
        readInput();
        
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
      if(elapsedTime >= INTERVAL) 
      {
        double output = calcPID(analogRead(A0));
        setMovement(output, output);
          
         previousTime = currentTime; 
        //print actual motor position and target value to serial-monitor/plotter
        Serial.print(analogRead(A0));
        Serial.print(" ");   
        Serial.println(target);
    
        }
          
}

// method to set direction and speed of the motor
void setMovement(int dir, int speed1) 
{
        //  ******** <TODO> **********************
        //  ******** implement your code  here **********************
        //set direction
        if(dir < 0) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          }
         else {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
         }
         if(abs(speed1) > 255) { speed1 = 255;}

         //set speed
         analogWrite(ENA, abs(speed1));
         motorSpeed = abs(speed1);

        
}



double calcPID(double inp){     
  
        error = target - inp; // determine error
        // accept errors less than 5, return 0
        if(abs(error) <= 5) {return 0;}

        // only integrate when error is between 10 and 100
        if(abs(error) >= 10 && abs(error) <= 100) {
          cumError += error * elapsedTime; // compute integral
          }
        
        rateError = (error - lastError)/elapsedTime; // compute derivative            
 
        lastError = error;
        
        return kp*error + ki*cumError + kd*rateError;;
}
// *********************************************************************************************************************** //
// method for receiving commands over the serial port
void readInput() 
{
      if (Serial.available())
    {
        recvchar = Serial.read();
        //reset all errors
        error = 0;
        cumError = 0;
        rateError = 0;
        lastError = 0;
        
        if (recvchar != '\n')
        { 
            record[indx++] = recvchar;
        }
        else if (recvchar == '\n')
        {
          record[indx] = '\0';
          indx = 0;
          
          convertData(record);
          if(i==4){
          target = PID_values[0];
          kp = PID_values[1];
          ki = PID_values[2];
          kd = PID_values[3];
          Serial.print("Entered Values:");
          printData(PID_values);
          }
          else
          {
          Serial.println("Enter correct number of values separated by commas!!");            
          }
        }
    }
}
// *********************************************************************************************************************** //
//method for reading/interpreting serial input 
void convertData(char record[])
{
    i = 0;
    char *index = strtok(record, ",");
    while(index != NULL)
    {
       PID_values[i++] = atof(index); 
        index = strtok(NULL, ",");
    }
}
// *********************************************************************************************************************** //
//method for printing values entered via the serial monitor
void printData(float data[])
{
    for (byte j = 0 ; j < 4 ; j++)
    {
      Serial.print(data[j]);
      Serial.print('\t');
    }
    Serial.println(); 
}
