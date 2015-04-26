#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Ultrasonic.h>

 
/************************************************************************************************/
/*                              DEFINIR PINES                                                   */
/************************************************************************************************/

/**/                    #define LEFT_WHEEL_FRONT    46                                        /**/
/**/                    #define LEFT_WHEEL_BACK     47                                        /**/
/**/                    #define RIGHT_WHEEL_FRONT   48                                        /**/
/**/                    #define RIGHT_WHEEL_BACK    49                                        /**/
/**/                                                                                          /**/
/**/                    #define SERVO_ROTAR         13                                        /**/
/**/                    #define SERVO_BASE_I        12                                        /**/
/**/                    #define SERVO_BASE_D        11                                        /**/
/**/                                                                                          /**/
/**/                    //10 no utilizar, utilizado por Ethernet                                                         
/**/                                                                                          /**/
/**/                    #define SERVO_HOMBRO        9                                         /**/
/**/                    #define SERVO_CODO          8                                         /**/
/**/                    #define SERVO_MUNECA        7                                         /**/
/**/                                                                                          /**/
/**/                    //SE ALIMENTAN DIRECTAMENTE DE ARDUINO                                            
/**/                    #define SERVO_MUNECA_ROTA   5                                         /**/
/**/                    #define SERVO_PINZA         4                                         /**/

/**/                    #define SERVO_SENSOR_DIST   3                                         /**/

/**/                    #define PIN_LASER           40  /*No en uso*/                         /**/
/**/                    #define TRIGGER_PIN         42                                        /**/
/**/                    #define ECHO_PIN            41                                        /**/

                
/*************************************************************************************************/
/*                              DEFINIR CONSTANTES                                               */
/*************************************************************************************************/

/**/                   #define MOTOR               'm'                                         /**/
/**/                   #define BRAZO               'b'                                         /**/
    
/**/                   #define ALANTE              'w'                                         /**/
/**/                   #define DETRAS              's'                                         /**/
/**/                   #define IZQUIERDA           'a'                                         /**/
/**/                   #define DERECHA             'd'                                         /**/
/**/                   #define STOP                'e'                                         /**/

/**/                   #define S_ROTAR             '0'                                         /**/
/**/                   #define S_BASE              '1'                                         /**/
/**/                   #define S_HOMBRO            '2'                                         /**/
/**/                   #define S_CODO              '3'                                         /**/
/**/                   #define S_MUNECA            '4'                                         /**/
/**/                   #define S_MUN_ROTAR         '5'                                         /**/
/**/                   #define S_PINZA             '6'                                         /**/

/**/                   #define ARTICULACION_NULA   '*'                                         /**/
/**/                   #define SUMA                '+'                                         /**/
/**/                   #define RESTA               '-'                                         /**/
/**/                   #define SIGNO_NULO      '/'                                             /**/

/**/                   #define DISTANCIA_MINIMA    10.0


/*************************************************************************************************/
/*                          DEFINIR VALORES DE RETORNO                                           */
/*************************************************************************************************/

/**/                   #define MOV_ROB_DESCONOCIDO  -1                                         /**/
/**/                   #define ARTIC_DESCONOCIDA    -2                                         /**/
/**/                   #define VALOR_FUERA_DE_RANGO -3                                         /**/
/**/                   #define TIPO_MOV_DESCONOCIDO   -4                                       /**/

/**/                   #define HECHO                 1                                         /**/
/**/                   #define ACK                   "ACK"                                     /**/
/**/                   #define NACK                  "NACK"                                    /**/

/**/                   #define DIST_INSUFICIENTE     -1                                        /**/
/**/                   #define DIST_CORRECTA          1                                        /**/



/*************************************************************************************************/
/*                         DEFINIR VARIABLES GLOBALES                                            */
/*************************************************************************************************/
          
            //Servos
/**/                   Servo servoRotar,                                                       /**/
/**/                         servoBaseI,                                                       /**/
/**/                         servoBaseD,                                                       /**/
/**/                         servoHombro,                                                      /**/
/**/                         servoCodo,                                                        /**/
/**/                         servoMuneca,                                                      /**/
/**/                         servoMunRota,                                                     /**/
/**/                         servoPinza;                                                       /**/

                      Servo *servos [] = { &servoRotar,                                        /**/
/**/                                       &servoBaseI,                                        /**/
/**/                                       &servoBaseD,                                        /**/
/**/                                       &servoHombro,                                       /**/
/**/                                       &servoCodo,                                         /**/
/**/                                       &servoMuneca,                                       /**/
/**/                                       &servoMunRota,                                      /**/
/**/                                       &servoPinza
                                        };

           //Posicion actual de los servos  
                     
/**/                   int   posRotar   = 0,                                                   /**/
/**/                         posBaseI   = 0,                                                   /**/
/**/                         posBaseD   = 0,                                                   /**/
/**/                         posHombro  = 0,                                                   /**/
/**/                         posCodo    = 0,                                                   /**/
/**/                         posMuneca  = 0,                                                   /**/
/**/                         posMunRota = 0,                                                   /**/
/**/                         posPinza   = 0;                                                   /**/

/**/                   int   *posiciones [] = { &posRotar,                                     /**/
/**/                                            &posBaseI,                                     /**/
/**/                                            &posBaseD,                                     /**/
/**/                                            &posHombro,                                    /**/
/**/                                            &posCodo,                                      /**/
/**/                                            &posMuneca,                                    /**/
/**/                                            &posMunRota,                                   /**/
/**/                                            &posPinza                                      /**/
                                            };                                                 /**/

          //Posicion de reposo de los servos

/**/                  int    reposoRotar  = 180,                                               /**/
/**/                         reposoBaseI  = 180,                                               /**/
/**/                         reposoBaseD  = 180,                                               /**/
/**/                         reposoHombro = 180,                                               /**/
/**/                         reposoCodo   = 180,                                               /**/
/**/                         reposoMun    = 180,                                               /**/
/**/                         reposoMunRot = 180,                                               /**/
/**/                         reposoPinza  = 180;                                               /**/

/**/                  int *posicionesReposo [] = { &reposoRotar,                               /**/
/**/                                               &reposoBaseI,                               /**/
/**/                                               &reposoBaseD,                               /**/
/**/                                               &reposoHombro,                              /**/
/**/                                               &reposoCodo,                                /**/
/**/                                               &reposoMun,                                 /**/
/**/                                               &reposoMunRot,                              /**/
/**/                                               &reposoPinza                                /**/
                                               };                                              /**/
                          
           //Manejo de conexion 
                                                                                                                                    
/**/                  EthernetUDP Udp;                                                         /**/
/**/                  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };                     /**/
/**/                  IPAddress ip(192, 2, 1, 177);                                            /**/
/**/                  unsigned int localPort = 8888;                                           /**/
/**/                  char  bufferRecibir[UDP_TX_PACKET_MAX_SIZE];                             /**/ 
/**/                  char  ackEnviar[] = "\tack\n",                                           /**/
/**/                        nackEnviar[]= "\tnack\n";                                          /**/
/**/                  int   tamRecibido = -1;                                                  /**/

           //Lectura de valores entrantes

/**/                  char  articulacion    = ARTICULACION_NULA,                               /**/  
/**/                        signo           = SIGNO_NULO,                                      /**/ 
/**/                        tipoMovimiento  = '0',                                             /**/
/**/                        movimiento      = '0';                                             /**/
/**/                  int   valorMov        =  0;                                              /**/

          //Lectura distancia a objeto mas proximo
          
/**/                  Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);                            /**/
/**/                  float distancia = 11.0;                                                  /**/
/**/                  long  microsec;                                                          /**/

          //Movimiento de las ruedas

/**/                  uint8_t ruedaI  = 0,                                                     /**/
/**/                          ruedaIb = 0,                                                     /**/
/**/                          ruedaD  = 0,                                                     /**/
/**/                          ruedaDb = 0;                                                     /**/

          //Tratar mensaje recibido 

/**/                 int valRetorno   = 0;                                                     /**/
                      

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
  
  servoRotar.  attach (SERVO_ROTAR);
  servoBaseI.  attach (SERVO_BASE_I);
  servoBaseD.  attach (SERVO_BASE_D);
  servoHombro. attach (SERVO_HOMBRO);
  servoCodo.   attach (SERVO_CODO);
  servoMuneca. attach (SERVO_MUNECA);
  servoMunRota.attach (SERVO_MUNECA_ROTA);
  servoPinza.  attach (SERVO_PINZA);
  
  servoReposo();
  Serial.print("Started");
  
}

void servoReposo(){
    for(int i = 0; i<sizeof(servos)/sizeof(Servo*) ; i++){
        articulacion = (char)(i + 48);    //
        signo        = SUMA;
        valorMov     = *posicionesReposo[i];
        
        mueveArticulacion();
                                                                                                                                                                                      
        resetValoresArticulacion();
    }
    Serial.println("Servos en posicion de reposo");
}

void loop() 
{ 
  comprobarDistancia();
  recibirComando(); 
}

void recibirComando()
{
  tamRecibido = Udp.parsePacket();
  if(tamRecibido)
  {
      Udp.read(bufferRecibir, UDP_TX_PACKET_MAX_SIZE);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      
      valRetorno = tratarComando();
      
      if(valRetorno == HECHO)
      {
          Udp.write(ACK);
      }
      else
      {
          Udp.write(NACK);
          Serial.println(valRetorno);
      }
      Udp.endPacket();
  }
}

int tratarComando () //Formato: {Tipo, movimiento, [signo, grados]}
{
        tipoMovimiento = bufferRecibir[0];
        movimiento     = bufferRecibir[1];  
        
        switch (tipoMovimiento)
        {
            case MOTOR:
                motor(movimiento);
                break;     
            case BRAZO:
                signo = bufferRecibir[2];
                valorMov  = (((int)bufferRecibir[3])-48)*100 + 
                            (((int)bufferRecibir[4])-48)* 10 + 
                            (((int)bufferRecibir[5])-48);
                
                if(movimiento >= S_ROTAR && movimiento <= S_PINZA)
                {
                    if((signo == SUMA || signo == RESTA) && (valorMov > 0 && valorMov <= 999))                     
                    {
                         articulacion = movimiento;      
                         mueveArticulacion();
                    }
                    else
                    {
                        resetValoresArticulacion();
                        return VALOR_FUERA_DE_RANGO;
                    }
                }
                else
                {
                    resetValoresArticulacion();
                    return ARTIC_DESCONOCIDA;
                }
                
                break;
            default:
                return TIPO_MOV_DESCONOCIDO;
        }
        return HECHO;
}

void resetValoresArticulacion()
{
    articulacion = ARTICULACION_NULA;
    signo    = SIGNO_NULO;
    valorMov     = 0;
}

void mueveArticulacion(){
    
    /*Serial.print("Moviendo ");
    Serial.print(articulacion);
    Serial.print(" ");
    Serial.print(signo);
    Serial.print(valorMov);
    Serial.println(" º"); */
  
    int articAux = ((int)articulacion) - 48;
    
    if(signo = SUMA)
    {      
        while(valorMov > 0)
        {
            *posiciones[articAux]++ ;
            servos[articAux]-> write(*posiciones[articAux]);
            valorMov--;
            delay(15);
        }
    }
    else
    {
       while(valorMov > 0)
       {
            *posiciones[articAux]-- ;
            servos[articAux]-> write(*posiciones[articAux]);
            valorMov--;
            delay(10);
       }  
    }
    
}

int comprobarDistancia(){
  microsec  = ultrasonic.timing();
  distancia = ultrasonic.convert(microsec, Ultrasonic::CM);
  /*Serial.print("Distancia medida = "); 
  Serial.println(distancia);*/
  
  if(distancia < DISTANCIA_MINIMA)
  {
      motor(STOP);
      return DIST_INSUFICIENTE;
  }
  return DIST_CORRECTA;
}

void motor(char movimiento){
  
   switch(movimiento)
   {
      case ALANTE:
            ruedaI  = HIGH; 
            ruedaIb = LOW;
         
            ruedaD  = HIGH;
            ruedaDb = LOW;
            //Serial.println("Alante");
            break;
      case DERECHA:
            ruedaI  = LOW; 
            ruedaIb = HIGH;
         
            ruedaD  = HIGH;
            ruedaDb = LOW;
            //Serial.println("Derecha");
            break;
      case IZQUIERDA:
            ruedaI  = HIGH; 
            ruedaIb = LOW;
         
            ruedaD  = LOW;
            ruedaDb = HIGH;
            //Serial.println("Izquierda");
            break;
      case DETRAS:
            ruedaI  = LOW; 
            ruedaIb = HIGH;
            
            ruedaD  = LOW;
            ruedaDb = HIGH;
            //Serial.println("Detras");
            break;
      default:
            ruedaI  = LOW; 
            ruedaIb = LOW;
            
            ruedaD  = LOW;
            ruedaDb = LOW;
            //Serial.println("Stop");
            break;
   } 
   
   if(comprobarDistancia() == DIST_CORRECTA)
   {
       digitalWrite(LEFT_WHEEL_FRONT , ruedaI);
       digitalWrite(LEFT_WHEEL_BACK  , ruedaIb);
       digitalWrite(RIGHT_WHEEL_FRONT, ruedaD);
       digitalWrite(RIGHT_WHEEL_BACK , ruedaDb);
   }
   else
   {
       digitalWrite(LEFT_WHEEL_FRONT , LOW);
       digitalWrite(LEFT_WHEEL_BACK  , LOW);  
       digitalWrite(RIGHT_WHEEL_FRONT, LOW);
       digitalWrite(RIGHT_WHEEL_BACK , LOW);
   }
}

