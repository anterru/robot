/***************************************************************************************************
    Author: Antonio Terrada Rubio
    
    Descripcion: The aim of this program is to control a mobile and manipulator robot. To control it, 
                 the orders are received from the Ethernet port. Once processed an order (move the car,
                 move an articulation or open/close the gripper), it answers indicating if the action
                 could be completed.  
    
****************************************************************************************************/     

#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Ultrasonic.h>

 
/************************************************************************************************/
/*                                DEFINE PINS                                                   */
/************************************************************************************************/

/**/                    #define LEFT_WHEEL_FRONT    46                                        /**/
/**/                    #define LEFT_WHEEL_BACK     47                                        /**/
/**/                    #define RIGHT_WHEEL_FRONT   48                                        /**/
/**/                    #define RIGHT_WHEEL_BACK    49                                        /**/
/**/                                                                                          /**/
/**/                    #define SERVO_ROTATE        13                                        /**/
/**/                    #define SERVO_BASE_L        12                                        /**/
/**/                    #define SERVO_BASE_R        11                                        /**/
/**/                                                                                          /**/
/**/                    //Do not use 10, it is used by the Ethernet shield                                                         
/**/                                                                                          /**/
/**/                    #define SERVO_SHOULDER       9                                        /**/
/**/                    #define SERVO_ELBOW          8                                        /**/
/**/                    #define SERVO_WRIST          7                                        /**/
/**/                                                                                          /**/
                                           
/**/                    #define SERVO_WRIST_ROTA     5                                        /**/
/**/                    #define SERVO_CLAW           4                                        /**/

/**/                    #define SERVO_SENSOR_DIST    3                                        /**/

/**/                    #define PIN_LASER           40  /*Not in use*/                        /**/
/**/                    #define TRIGGER_PIN         42                                        /**/
/**/                    #define ECHO_PIN            41                                        /**/

                
/*************************************************************************************************/
/*                              DEFINE CONSTANTS                                                 */
/*************************************************************************************************/

/**/                   #define MOTOR               'm'                                         /**/
/**/                   #define ARM                 'b'                                         /**/
    
/**/                   #define STRAIGHT            'w'                                         /**/
/**/                   #define BACK                's'                                         /**/
/**/                   #define LEFT                'a'                                         /**/
/**/                   #define RIGHT               'd'                                         /**/
/**/                   #define STOP                'e'                                         /**/

/**/                   #define S_ROTATE            '0'                                         /**/
/**/                   #define S_BASE              '1'                                         /**/
/**/                   #define S_SHOULDER          '2'                                         /**/
/**/                   #define S_ELBOW             '3'                                         /**/
/**/                   #define S_WRIST             '4'                                         /**/
/**/                   #define S_WRIST_ROT         '5'                                         /**/
/**/                   #define S_CLAW              '6'                                         /**/

/**/                   #define NULL_ARTICULATION   '*'                                         /**/
/**/                   #define ADD                 '+'                                         /**/
/**/                   #define SUB                 '-'                                         /**/
/**/                   #define NULL_OPERATION      '/'                                         /**/

/**/                   #define MIN_DIST            10.0                                        /**/


/*************************************************************************************************/
/*                              DEFINE RETURN VALUES                                             */
/*************************************************************************************************/

/**/                   #define MOV_ROB_UNKNOWN      -1                                         /**/
/**/                   #define ARTIC_UNKNOWN        -2                                         /**/
/**/                   #define OUT_OF_RANGE_VALUE   -3                                         /**/
/**/                   #define MOV_TYPE_UNKNOWN     -4                                         /**/

/**/                   #define DONE                  1                                         /**/
/**/                   #define ACK                   "ACK"                                     /**/
/**/                   #define NACK                  "NACK"                                    /**/

/**/                   #define INSUF_DISTANCE       -1                                         /**/
/**/                   #define CORRECT_DIST          1                                         /**/
 


/*************************************************************************************************/
/*                           DEFINE GLOBAL VARIABLES                                             */
/*************************************************************************************************/
          
            //Servos
/**/                   Servo servoRotate,                                                      /**/
/**/                         servoBaseL,                                                       /**/
/**/                         servoBaseR,                                                       /**/
/**/                         servoShoulder,                                                    /**/
/**/                         servoElbow,                                                       /**/
/**/                         servoWrist,                                                       /**/
/**/                         servoWristRotate,                                                 /**/
/**/                         servoClaw;                                                        /**/

                      Servo *servos [] = { &servoRotate,                                       /**/
/**/                                       &servoBaseL,                                        /**/
/**/                                       &servoBaseR,                                        /**/
/**/                                       &servoShoulder,                                     /**/
/**/                                       &servoElbow,                                        /**/
/**/                                       &servoWrist,                                        /**/
/**/                                       &servoWristRotate,                                  /**/
/**/                                       &servoClaw
                                        };

           //Current position of each servo 
                     
/**/                   int   posRotate   = 0,                                                  /**/
/**/                         posBaseL   = 0,                                                   /**/
/**/                         posBaseR   = 0,                                                   /**/
/**/                         posShoulder  = 0,                                                 /**/
/**/                         posElbow    = 0,                                                  /**/
/**/                         posWrist  = 0,                                                    /**/
/**/                         posWristRotate = 0,                                               /**/
/**/                         posClaw   = 0;                                                    /**/

/**/                   int   *pos [] = { &posRotate,                                           /**/
/**/                                            &posBaseL,                                     /**/
/**/                                            &posBaseR,                                     /**/
/**/                                            &posShoulder,                                  /**/
/**/                                            &posElbow,                                     /**/
/**/                                            &posWrist,                                     /**/
/**/                                            &posWristRotate,                               /**/
/**/                                            &posClaw                                       /**/
                                            };                                                 /**/

          //Default position of each servo

/**/                  int    defaultRotate   = 180,                                            /**/
/**/                         defaultBaseL    = 180,                                            /**/
/**/                         defaultBaseR    = 180,                                            /**/
/**/                         defaultShoulder = 180,                                            /**/
/**/                         defaultElbow    = 180,                                            /**/
/**/                         defaultWrist    = 180,                                            /**/
/**/                         defaultWristRot = 180,                                            /**/
/**/                         defaultClaw     = 180;                                            /**/

/**/                  int *posDefault [] = { &defaultRotate,                                   /**/
/**/                                               &defaultBaseL,                              /**/
/**/                                               &defaultBaseR,                              /**/
/**/                                               &defaultShoulder,                           /**/
/**/                                               &defaultElbow,                              /**/
/**/                                               &defaultWrist,                              /**/
/**/                                               &defaultWristRot,                           /**/
/**/                                               &defaultClaw                                /**/
                                               };                                              /**/
                          
           //Connection management
                                                                                                                                    
/**/                  EthernetUDP Udp;                                                         /**/
/**/                  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };                     /**/
/**/                  IPAddress ip(192, 2, 1, 177);                                            /**/
/**/                  int   localPort = 8888;                                                  /**/
/**/                  char  bufferRecv[UDP_TX_PACKET_MAX_SIZE];                                /**/ 
/**/                  char  ackSend[] = "\tack\n",                                             /**/
/**/                        nackSend[]= "\tnack\n";                                            /**/
/**/                  int   receivedSize = -1;                                                 /**/

           //Needed to read input values

/**/                  char  articulation    = NULL_ARTICULATION,                               /**/  
/**/                        operation       = NULL_OPERATION,                                  /**/ 
/**/                        typeMovement    = '0',                                             /**/
/**/                        movement        = '0';                                             /**/
/**/                  int   movValue        =  0;                                              /**/

          //Needed to read the distance to the closest object
          
/**/                  Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);                            /**/
/**/                  float distance = 11.0;                                                   /**/
/**/                  long  microsec;                                                          /**/

          //Wheels movement

/**/                  uint8_t wheelL  = 0,                                                     /**/
/**/                          wheelLb = 0,                                                     /**/
/**/                          wheelR  = 0,                                                     /**/
/**/                          wheelRb = 0;                                                     /**/


/**/                 int retValue   = 0;                                                       /**/
                      

/*************************************************************************************************/                 
                

void setup() 
{ 
  Serial.begin(9600);
  
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  
  pinMode(LEFT_WHEEL_FRONT,  OUTPUT);
  pinMode(LEFT_WHEEL_BACK,   OUTPUT);
  pinMode(RIGHT_WHEEL_FRONT, OUTPUT);
  pinMode(RIGHT_WHEEL_BACK,  OUTPUT);
  
  digitalWrite(LEFT_WHEEL_FRONT,  LOW);
  digitalWrite(LEFT_WHEEL_BACK,   LOW);
  digitalWrite(RIGHT_WHEEL_FRONT, LOW);
  digitalWrite(RIGHT_WHEEL_BACK,  LOW);
  
  servoRotate.     attach (SERVO_ROTATE);
  servoBaseL.      attach (SERVO_BASE_L);
  servoBaseR.      attach (SERVO_BASE_R);
  servoShoulder.   attach (SERVO_SHOULDER);
  servoElbow.      attach (SERVO_ELBOW);
  servoWrist.      attach (SERVO_WRIST);
  servoWristRotate.attach (SERVO_WRIST_ROTA);
  servoClaw.       attach (SERVO_CLAW);
  
  defaultPosServos();
  Serial.print("Started");
  
}

void defaultPosServos(){
    for(int i = 0; i<sizeof(servos)/sizeof(Servo*) ; i++){
        articulation = (char)(i + 48);    //
        operation        = ADD;
        movValue     = *posDefault[i];
        
        moveArticulation();
                                                                                                                                                                                      
        resetArticValues();
    }
}

void loop() 
{ 
  checkDistance();
  receiveCommand(); 
}

void receiveCommand()
{
  receivedSize = Udp.parsePacket();
  if(receivedSize)
  {
      Udp.read(bufferRecv, UDP_TX_PACKET_MAX_SIZE);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      
      retValue = tratarComando();
      
      if(retValue == DONE)
      {
          Udp.write(ACK);
      }
      else
      {
          Udp.write(NACK);
          Serial.println(retValue);
      }
      Udp.endPacket();
  }
}

int tratarComando () //Formato: {Tipo, movement, [operation, grados]}
{
        typeMovement = bufferRecv[0];
        movement     = bufferRecv[1];  
        
        switch (typeMovement)
        {
            case MOTOR:
                motor(movement);
                break;     
            case ARM:
                operation = bufferRecv[2];
                movValue  = (((int)bufferRecv[3])-48)*100 + 
                            (((int)bufferRecv[4])-48)* 10 + 
                            (((int)bufferRecv[5])-48);
                
                if(movement >= S_ROTATE && movement <= S_CLAW)
                {
                    if((operation == ADD || operation == SUB) && (movValue > 0 && movValue <= 180))                     
                    {
                         articulation = movement;      
                         moveArticulation();
                    }
                    else
                    {
                        resetArticValues();
                        return OUT_OF_RANGE_VALUE;
                    }
                }
                else
                {
                    resetArticValues();
                    return ARTIC_UNKNOWN;
                }
                
                break;
            default:
                return MOV_TYPE_UNKNOWN;
        }
        return DONE;
}

void resetArticValues()
{
    articulation = NULL_ARTICULATION;
    operation    = NULL_OPERATION;
    movValue     = 0;
}

void moveArticulation(){
    
    /*Serial.print("Moviendo ");
    Serial.print(articulation);
    Serial.print(" ");
    Serial.print(operation);
    Serial.print(movValue);
    Serial.println(" º"); */
  
    int articAux = ((int)articulation) - 48;
    
    if(operation = ADD)
    {      
        while(movValue > 0)
        {
            *pos[articAux]++ ;
            servos[articAux]-> write(*pos[articAux]);
            movValue--;
            delay(15);
        }
    }
    else
    {
       while(movValue > 0)
       {
            *pos[articAux]-- ;
            servos[articAux]-> write(*pos[articAux]);
            movValue--;
            delay(10);
       }  
    }
    
}

int checkDistance(){
  microsec  = ultrasonic.timing();
  distance = ultrasonic.convert(microsec, Ultrasonic::CM);
  /*Serial.print("distance medida = "); 
  Serial.println(distance);*/
  
  if(distance < MIN_DIST)
  {
      motor(STOP);
      return INSUF_DISTANCE;
  }
  return CORRECT_DIST;
}

void motor(char movement){
  
   switch(movement)
   {
      case STRAIGHT:
            wheelL  = HIGH; 
            wheelLb = LOW;
         
            wheelR  = HIGH;
            wheelRb = LOW;
            //Serial.println("STRAIGHT");
            break;
      case RIGHT:
            wheelL  = LOW; 
            wheelLb = HIGH;
         
            wheelR  = HIGH;
            wheelRb = LOW;
            //Serial.println("RIGHT");
            break;
      case LEFT:
            wheelL  = HIGH; 
            wheelLb = LOW;
         
            wheelR  = LOW;
            wheelRb = HIGH;
            //Serial.println("LEFT");
            break;
      case BACK:
            wheelL  = LOW; 
            wheelLb = HIGH;
            
            wheelR  = LOW;
            wheelRb = HIGH;
            //Serial.println("BACK");
            break;
      default:
            wheelL  = LOW; 
            wheelLb = LOW;
            
            wheelR  = LOW;
            wheelRb = LOW;
            //Serial.println("Stop");
            break;
   } 
   
   if(checkDistance() == CORRECT_DIST)
   {
       digitalWrite(LEFT_WHEEL_FRONT , wheelL);
       digitalWrite(LEFT_WHEEL_BACK  , wheelLb);
       digitalWrite(RIGHT_WHEEL_FRONT, wheelR);
       digitalWrite(RIGHT_WHEEL_BACK , wheelRb);
   }
   else
   {
       digitalWrite(LEFT_WHEEL_FRONT , LOW);
       digitalWrite(LEFT_WHEEL_BACK  , LOW);  
       digitalWrite(RIGHT_WHEEL_FRONT, LOW);
       digitalWrite(RIGHT_WHEEL_BACK , LOW);
   }
}

