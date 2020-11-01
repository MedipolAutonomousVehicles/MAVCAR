#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "JHPWMPCA9685.h"


// Calibrated for a Robot Geek RGS-13 Servo
// Make sure these are appropriate for the servo being used!

int servoMin = 120 ;
int servoMax = 720 ;
int sagileri=0;
int saggeri=1;
int solileri=2;
int solgeri=3;

PCA9685 *pca9685 = new PCA9685() ;
int err = pca9685->openPCA9685();

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}
void ileri(int hiz){
pca9685->setPWM(sagileri,0,hiz) ;
pca9685->setPWM(solileri,0,hiz) ;

pca9685->setPWM(saggeri,0,0) ;
pca9685->setPWM(solgeri,0,0) ;
}
void geri(int hiz){
pca9685->setPWM(saggeri,0,hiz) ;
pca9685->setPWM(solgeri,0,hiz) ;

pca9685->setPWM(sagileri,0,0) ;
pca9685->setPWM(solileri,0,0) ;
}
int sag(int hiz){
pca9685->setPWM(solileri,0,hiz) ;
pca9685->setPWM(saggeri,0,hiz) ;

pca9685->setPWM(sagileri,0,0) ;
pca9685->setPWM(solgeri,0,0) ;
}
int sol(int hiz){
pca9685->setPWM(solgeri,0,hiz) ;
pca9685->setPWM(sagileri,0,hiz) ;

pca9685->setPWM(saggeri,0,0) ;
pca9685->setPWM(solileri,0,0) ;
}



int main() {
    
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60) ;
        // 27 is the ESC key
        printf("Hit ESC key to exit\n");
        while(pca9685->error >= 0 && getkey() != 27){
            
            if(getkey() == 119){
            printf("ileri");
            ileri(700);
            }
            if(getkey() == 97){
            printf("sol");
            sol(700);
            }
            if(getkey() == 100){
            printf("sag");
            sag(700);
            }
            if(getkey() == 115){
            printf("geri");
            geri(700);
            }
            if (getkey() == 98){
            printf("fren");
            pca9685->setPWM(sagileri,0,0) ;
            pca9685->setPWM(solileri,0,0) ;
            pca9685->setPWM(saggeri,0,0) ;
            pca9685->setPWM(solgeri,0,0) ;
            }
            

        }
        
        pca9685->setAllPWM(0,0) ;
        sleep(1);
    }
    pca9685->closePCA9685();
}
