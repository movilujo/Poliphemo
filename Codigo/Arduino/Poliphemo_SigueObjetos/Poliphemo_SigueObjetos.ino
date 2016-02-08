/*
 *  Author: Jose Luis Villarejo Muñoz
 *  Date  : Enero 2016
 *  Robot diseñado por Jose Luis Villarejo (http://robotstyles.blogspot.com.es)
 *  Licencia: GNU General Public License v3 or later
 *
 *  Programa sigue objetos para Poliphemo (Robot educacional)
 *  Idea original:https://learn.adafruit.com/pixy-pet-robot-color-vision-follower-using-pixycam/the-code
 *    y programas de ejemplo de Pixy CMUcam5 adaptado para sistema motriz 
 *    basado en servomotores de rotacion continua
 *
 */
 
#include <Servo.h>
#include <SPI.h>  
#include <Pixy.h>

#define PT_MIN_X        0L
#define PT_MAX_X        320L
#define PT_MIN_Y        0L
#define PT_MAX_Y        200L

#define X_CENTER        ((PT_MAX_X-PT_MIN_X)/2)       
#define Y_CENTER        ((PT_MAX_Y-PT_MIN_Y)/2)

#define SERVO_MIN_POS     0L
#define SERVO_MAX_POS     1000L
#define SERVO_CENTER_POS  ((SERVO_MAX_POS-SERVO_MIN_POS)/2)

/*--------------------------------------------------------------
 * Clase ServoLoop
 *  Se utiliza par el control del sistema pan/tilt que mueve la
 *  camara 
 *  
 *  Esta clase se ha obtenido de uno de los ejemplos proporcionados por
 *  Pixy CMUcam 5, en concreto pantil.ino
 */


class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};

/* Constructor */
ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = SERVO_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  if (m_prevError!=0x80000000)
  {  
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    m_pos += vel;
    if (m_pos>SERVO_MAX_POS) 
      m_pos = SERVO_MAX_POS; 
    else if (m_pos<SERVO_MIN_POS) 
      m_pos = SERVO_MIN_POS;
  }
  m_prevError = error;
}

/*--------------------------------------------------------------
*  Definición de Servos utilizados en el sistema motriz del robot
*/

Pixy pixy;

Servo motrizD;
Servo motrizI;

ServoLoop panLoop(350, 700);
ServoLoop tiltLoop(500, 700);

/* Definición de Pines */
int pMotrizD = 5;     // conexión del servo motriz derecho
int pMotrizI = 6;     // conexión del servo motriz izquierdo

uint32_t lastBlockTime = 0;
int32_t size = 400;
int msParado = 1510;

void setup() {
  
  motrizD.attach(pMotrizD);
  motrizI.attach(pMotrizI);
  Serial.begin(9600);
  Serial.println("En Setup...\n");
  
  pixy.init();

}

void loop() {
  uint16_t bloques;
  bloques = pixy.getBlocks();
  
  // Si se han detectado bloques
  if (bloques)
  {
    int trackedBlock = TrackBlock(bloques);
    SigueObjeto(trackedBlock);
    lastBlockTime = millis();
   }  
  else if (millis() - lastBlockTime > 1000)
  {
    motrizD.writeMicroseconds(msParado);
    motrizI.writeMicroseconds(msParado);
    ScanForBlocks();
  }
}

int oldX, oldY, oldSignature;

//-----------------------------------------------------------
// Funcion basada en el ejemplo pantilt de la libreria Pixy
// posiciona la camara apuntando al centro del objeto
//-----------------------------------------------------------
int TrackBlock(int blockCount)
{
	int trackedBlock = 0;
	long maxSize = 0;

	for (int i = 0; i < blockCount; i++)
	{
		if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
		{
			long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
			if (newSize > maxSize)
			{
				trackedBlock = i;
				maxSize = newSize;
			}
		}
	}

	int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
	int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;

	panLoop.update(panError);
	tiltLoop.update(-tiltError);

	pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

	oldX = pixy.blocks[trackedBlock].x;
	oldY = pixy.blocks[trackedBlock].y;
	oldSignature = pixy.blocks[trackedBlock].signature;
	return trackedBlock;
}

//-----------------------------------------------------------
// Dirige el robot hacia la direccion que le marca la camara
// La velocidad a la que se acerca el robot al objeto esta 
// determinada por la proximidad a este
//-----------------------------------------------------------

void SigueObjeto(int trackedBlock)
{
	int32_t followError = SERVO_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
       
	size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
	size -= size >> 3;


	// Forward speed decreases as we approach the object (size is larger)
	int forwardSpeed = constrain(400 - (size/150), -100, 400);  

	// Steering differential is proportional to the error times the forward speed
	int32_t differential = (followError + (followError * forwardSpeed))>>8;

	// Adjust the left and right speeds by the steering differential.
	int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
	int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
        
        leftSpeed =  map(leftSpeed, -400, 400, 1750, 1250);
        rightSpeed = map(rightSpeed, -400, 400, 1250, 1750);

        motrizD.writeMicroseconds(rightSpeed);
        motrizI.writeMicroseconds(leftSpeed);

}

//---------------------------------------
// El robot gira sobre su eje en busqueda
// del objeto
// 
//---------------------------------------
uint32_t lastMove = 0;
     
void ScanForBlocks()
{
  Serial.println("Buscando bloques...");
  if (millis() - lastMove > 20)
     {
      lastMove = millis();
      tiltLoop.m_pos = 300;
      panLoop.m_pos = 300;
     
      motrizD.writeMicroseconds(1400);
      motrizI.writeMicroseconds(1400);
      delay(20);
      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
     }
}



