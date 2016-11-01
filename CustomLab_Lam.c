/*
 * Controller_Matrix.c
 *
 * Created: 5/25/2014 11:35:11 AM
 *  Author: Asayake
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <bit.h>
#include <timer.h>
#include <stdio.h>

unsigned char controllerStatus = 0x00;
unsigned char controllerStatus2 = 0x00;
unsigned char bodyCount = 13;
unsigned char MatrixArrays[8][8];
unsigned char reset = 0;
unsigned char obsCount = 0;

unsigned long int findGCD(unsigned long int a, unsigned long int b)
{
	unsigned long int c;
	while(1){
		c = a%b;
		if(c==0){return b;}
		a = b;
		b = c;
	}
	return 0;
}

typedef struct _body{
	unsigned char snakeRow;
	unsigned char snakeCol;
	} body;

typedef struct _previous{
	unsigned char previousRow;
	unsigned char previousCol;
} previous;

typedef struct _obstacle{
	unsigned char obstacleRow;
	unsigned char obstacleCol;
	unsigned char obstacleState;
} obstacle;

previous previousBody [32];
obstacle obstacles [4];
body snakeBody [32];

void update(void){
	for(unsigned char y = 0; y <= bodyCount; ++y){
		previousBody[y].previousCol = snakeBody[y].snakeCol;
		previousBody[y].previousRow = snakeBody[y].snakeRow;
	}
	for(unsigned char x = bodyCount; x > 0; --x){
		snakeBody[x].snakeCol = snakeBody[x-1].snakeCol;
		snakeBody[x].snakeRow = snakeBody[x-1].snakeRow;
	}
}

unsigned char snakeCollide(body *z){
	for(unsigned char x = 1; x <= bodyCount; ++x){
		if(z[0].snakeCol == z[x].snakeCol && z[0].snakeRow == z[x].snakeRow){
			return 1;
		}
	}
	return 0;
}

unsigned char objectCollide(body *z, obstacle *a){
	for(unsigned char x = 0; x <= 4; ++x){
		if(a[x].obstacleState != 0){
			if(z[0].snakeCol == a[x].obstacleCol && z[0].snakeRow == a[x].obstacleRow){
				return 1;
			}
		}
	}
	return 0;
}

typedef struct _task {
	/*Tasks should have members that include: state, period,
		a measurement of elapsed time, and a function pointer.*/
	signed char state; //Task's current state
	unsigned long int period; //Task period
	unsigned long int elapsedTime; //Time elapsed since last task tick
	int (*TickFct)(int); //Task tick function
} task;

//Tick Function to Sample SNES controller
enum SNES_States {SNES_Init, latchOn ,latchOff, clkOn1, clkOff1, clkOn2, clkOff2};
int SNES_Tick(int state)
{
	unsigned char tmpC = PINC;
	static unsigned char i = 0;
	
	switch (state)
	{
		case -1:
		{
			state = SNES_Init;
			break;
		}
		case SNES_Init:
		{
			state = latchOn;
			
			break;
		}
		case latchOn:
		{
			state = latchOff;
			break;
		}
		case latchOff:
		{
			state = clkOn1;
			break;
		}
		case clkOn1:
		{
			state = clkOff1;
			break;
		}
		case clkOff1:
		{
			if (i <= 7)
			{
				state = clkOn1;
			}
			else
			{
				state = clkOn2;
				i = 0;
			}
			break;
		}
		case clkOn2:
		{
			state = clkOff2;
			break;
		}
		case clkOff2:
		{
			if (i <= 7)
			{
				state = clkOn2;
			}
			else
			{
				state = SNES_Init;
				i = 0;
			}
			break;
		}
		default:
		{
			state = -1;
			break;
		}
	}
	
	switch (state)
	{
		case SNES_Init:
		{
			tmpC = SetBit(tmpC,3,0); //Turn latch off
			tmpC = SetBit(tmpC,4,0); //Turn CLK off
			break;
		}
		case latchOn:
		{
			tmpC = SetBit(tmpC,3,1);
			break;
		}
		case latchOff:
		{
			tmpC = SetBit(tmpC,3,0);
			controllerStatus = SetBit(controllerStatus,i, !GetBit(tmpC,2));
			i++;
			break;
		}
		case clkOn1:
		{
			tmpC = SetBit(tmpC,4,1);
			break;
		}
		case clkOff1:
		{
			tmpC = SetBit(tmpC,4,0);
			controllerStatus = SetBit(controllerStatus,i, !GetBit(tmpC,2));
			i++;
			break;
		}
		case clkOn2:
		{
			tmpC = SetBit(tmpC,4,1);
			break;
		}
		case clkOff2:
		{
			tmpC = SetBit(tmpC,4,0);
			controllerStatus2 = SetBit(controllerStatus2,i, !GetBit(tmpC,2));
			i++;
			break;
		}
		default:
		{
			break;
		}
	}
	
	PORTC = tmpC;
	
	return state;
}

enum ctrl_States{init, check, ctrl_Up, ctrl_Down, ctrl_Left, ctrl_Right, ctrl_Up_Release, ctrl_Down_Release, ctrl_Left_Release, ctrl_Right_Release, resetNow, gameOver};// ctrl_State;
int ctrl_Tick(int state){
	
	switch(state){
		case init:
			reset = 0;
			update();
			snakeBody[0].snakeCol = 4;
			snakeBody[0].snakeRow = 3;
			state = check;
			break;
		
		case check:
			if(GetBit(controllerStatus, 4) && !GetBit(controllerStatus,5)){
				state = ctrl_Up;
			}
			else if(GetBit(controllerStatus, 5) && !GetBit(controllerStatus,4)){
				state = ctrl_Down;
			}
			else if(GetBit(controllerStatus, 6) && !GetBit(controllerStatus,7)){
				state = ctrl_Left;
			}
		
			else if(GetBit(controllerStatus, 7) && !GetBit(controllerStatus,6)){
				state = ctrl_Right;
			}
			else{
				state = check;
			}
			break;
		
		case ctrl_Up:
			if(snakeBody[0].snakeRow >= 7 || snakeCollide(snakeBody) == 1 || objectCollide(snakeBody, obstacles) == 1){
				state = resetNow;
			}
			else{
				state =  ctrl_Up_Release;
			}
			break;
		
		case ctrl_Down:
			if(snakeBody[0].snakeRow <= 0 || snakeCollide(snakeBody) == 1 || objectCollide(snakeBody, obstacles) == 1){
				state = resetNow;
			}
			else{
				state =  ctrl_Down_Release;
			}
			break;
		
		case ctrl_Left:
			if(snakeBody[0].snakeCol <= 0 || snakeCollide(snakeBody) == 1 || objectCollide(snakeBody, obstacles) == 1){
				state = resetNow;
			}
			else{
				state =  ctrl_Left_Release;
			}
			break;
		
		case ctrl_Right:
			if(snakeBody[0].snakeCol >= 7 || snakeCollide(snakeBody) == 1 || objectCollide(snakeBody, obstacles) == 1){
				state = resetNow;
			}
			else{
				state =  ctrl_Right_Release;
			}
			break;
		
		case ctrl_Up_Release:
			if(GetBit(controllerStatus, 5) && !GetBit(controllerStatus,4)){
				state = ctrl_Down;
			}
			else if(GetBit(controllerStatus, 6) && !GetBit(controllerStatus,7)){
				state = ctrl_Left;
			}
			else if(GetBit(controllerStatus, 7) && !GetBit(controllerStatus,6)){
				state = ctrl_Right;
			}
			else{
				state = ctrl_Up;
			}
			break;
		
		case ctrl_Down_Release:
			if(GetBit(controllerStatus, 4) && !GetBit(controllerStatus,5)){
				state = ctrl_Up;
			}
			else if(GetBit(controllerStatus, 6) && !GetBit(controllerStatus,7)){
				state = ctrl_Left;
			}
			else if(GetBit(controllerStatus, 7) && !GetBit(controllerStatus,6)){
				state = ctrl_Right;
			}
			else{
				state = ctrl_Down;
			}
			break;
		
		case ctrl_Left_Release:
			if(GetBit(controllerStatus, 4) && !GetBit(controllerStatus,5)){
				state = ctrl_Up;
			}
			else if(GetBit(controllerStatus, 5) && !GetBit(controllerStatus,4)){
				state = ctrl_Down;
			}
			
			else if(GetBit(controllerStatus, 7) && !GetBit(controllerStatus,6)){
				state = ctrl_Right;
			}
			else{
				state = ctrl_Left;
			}
			break;
		
		case ctrl_Right_Release:
			if(GetBit(controllerStatus, 4) && !GetBit(controllerStatus,5)){
				state = ctrl_Up;
			}
			else if(GetBit(controllerStatus, 5) && !GetBit(controllerStatus,4)){
				state = ctrl_Down;
			}
			
			else if(GetBit(controllerStatus, 6)  && !GetBit(controllerStatus,7)){
				state = ctrl_Left;
			}
			else{
				state = ctrl_Right;
			}
			break;
			
		case resetNow:
			reset = 1;
			update();
			for(unsigned char x = 0; x <= bodyCount; ++x){
				snakeBody[x].snakeRow = 0x00;
				snakeBody[x].snakeCol = 0x00;
			}
			state = gameOver;
			break;
			
		case gameOver:
			if(GetBit(controllerStatus, 3)){
				state = init;
			}
			else{
				state = gameOver;
			}
			break;
			
		default:
			state = init;
			break;
	}
	
	switch(state){
		case init:
			break;
		
		case check:
			break;
		
		case ctrl_Up:
			update();
			snakeBody[0].snakeRow = snakeBody[0].snakeRow + 1;
			break;
		
		case ctrl_Down:
			update();
			snakeBody[0].snakeRow = snakeBody[0].snakeRow - 1;
			break;
		
		case ctrl_Left:
			update();
			snakeBody[0].snakeCol = snakeBody[0].snakeCol -1;
			break;
		
		case ctrl_Right:
			update();
			snakeBody[0].snakeCol = snakeBody[0].snakeCol + 1;
			break;
		
		case ctrl_Up_Release:
			break;
		
		case ctrl_Down_Release:
			break;
		
		case ctrl_Left_Release:
			break;
		
		case ctrl_Right_Release:
			break;
			
		case resetNow:	
			break;
			
		case gameOver:
			break;
			
		default:
			break;
		
	}
	return state;
}

enum MAT_States{MAT_Init, MAT_Row1, MAT_Row2, MAT_Row3,MAT_Row4, MAT_Row5,
MAT_Row6, MAT_Row7, MAT_Row8};
int TickFct_MatrixOutput (int state)
{
	unsigned char tmpA = 0x00;
	unsigned char tmpC = 0xFF;
	
	unsigned char col = 0;
	
	for(unsigned char y = 0; y <= bodyCount; ++y){
		MatrixArrays[previousBody[y].previousRow][previousBody[y].previousCol] = 0x00;
	}

	for(unsigned char b = 0; b < 4; ++b){
		if(obstacles[b].obstacleState == 0){
			MatrixArrays[obstacles[b].obstacleRow][obstacles[b].obstacleCol] = 0x00;
		}
	}

	if(reset == 0){
		for(unsigned char x = 0; x <= bodyCount; ++x){
			MatrixArrays[snakeBody[x].snakeRow][snakeBody[x].snakeCol] = 0x01;
		}
		for(unsigned char z = 0; z < 4; ++z){
			if(obstacles[z].obstacleState != 0){
				MatrixArrays[obstacles[z].obstacleRow][obstacles[z].obstacleCol] = 0x01;
			}
		}
	}
	
	switch (state)
	{
		case -1:
		{
			state = MAT_Init;
			break;
		}
		case MAT_Init:
		{
			state = MAT_Row1;
			break;
		}
		case MAT_Row1:
		{
			state = MAT_Row2;
			break;
		}
		case MAT_Row2:
		{
			state = MAT_Row3;
			break;
		}
		case MAT_Row3:
		{
			state = MAT_Row4;
			break;
		}
		case MAT_Row4:
		{
			state = MAT_Row5;
			break;
		}
		case MAT_Row5:
		{
			state = MAT_Row6;
			break;
		}
		case MAT_Row6:
		{
			state = MAT_Row7;
			break;
		}
		case MAT_Row7:
		{
			state = MAT_Row8;
			break;
		}
		case MAT_Row8:
		{
			state = MAT_Row1;
			break;
		}
		default:
		{
			state = -1;
			break;
		}
	}
	
	switch (state)
	{
		case MAT_Init:
		{
			MatrixArrays[0][0] = 0x00;
			MatrixArrays[0][1] = 0x00;
			MatrixArrays[0][2] = 0x00;
			MatrixArrays[0][3] = 0x00;
			MatrixArrays[0][4] = 0x00;
			MatrixArrays[0][5] = 0x00;
			MatrixArrays[0][6] = 0x00;
			MatrixArrays[0][7] = 0x00;
			MatrixArrays[1][0] = 0x00;
			MatrixArrays[1][1] = 0x00;
			MatrixArrays[1][2] = 0x00;
			MatrixArrays[1][3] = 0x00;
			MatrixArrays[1][4] = 0x00;
			MatrixArrays[1][5] = 0x00;
			MatrixArrays[1][6] = 0x00;
			MatrixArrays[1][7] = 0x00;
			MatrixArrays[2][0] = 0x00;
			MatrixArrays[2][1] = 0x00;
			MatrixArrays[2][2] = 0x00;
			MatrixArrays[2][3] = 0x00;
			MatrixArrays[2][4] = 0x00;
			MatrixArrays[2][5] = 0x00;
			MatrixArrays[2][6] = 0x00;
			MatrixArrays[2][7] = 0x00;
			MatrixArrays[3][0] = 0x00;
			MatrixArrays[3][1] = 0x00;
			MatrixArrays[3][2] = 0x00;
			MatrixArrays[3][3] = 0x00;
			MatrixArrays[3][4] = 0x00;
			MatrixArrays[3][5] = 0x00;
			MatrixArrays[3][6] = 0x00;
			MatrixArrays[3][7] = 0x00;
			MatrixArrays[4][0] = 0x00;
			MatrixArrays[4][1] = 0x00;
			MatrixArrays[4][2] = 0x00;
			MatrixArrays[4][3] = 0x00;
			MatrixArrays[4][4] = 0x00;
			MatrixArrays[4][5] = 0x00;
			MatrixArrays[4][6] = 0x00;
			MatrixArrays[4][7] = 0x00;
			MatrixArrays[5][0] = 0x00;
			MatrixArrays[5][1] = 0x00;
			MatrixArrays[5][2] = 0x00;
			MatrixArrays[5][3] = 0x00;
			MatrixArrays[5][4] = 0x00;
			MatrixArrays[5][5] = 0x00;
			MatrixArrays[5][6] = 0x00;
			MatrixArrays[5][7] = 0x00;
			MatrixArrays[6][0] = 0x00;
			MatrixArrays[6][1] = 0x00;
			MatrixArrays[6][2] = 0x00;
			MatrixArrays[6][3] = 0x00;
			MatrixArrays[6][4] = 0x00;
			MatrixArrays[6][5] = 0x00;
			MatrixArrays[6][6] = 0x00;
			MatrixArrays[6][7] = 0x00;
			MatrixArrays[7][0] = 0x00;
			MatrixArrays[7][1] = 0x00;
			MatrixArrays[7][2] = 0x00;
			MatrixArrays[7][3] = 0x00;
			MatrixArrays[7][4] = 0x00;
			MatrixArrays[7][5] = 0x00;
			MatrixArrays[7][6] = 0x00;
			MatrixArrays[7][7] = 0x00;
			
			break;
		}
		case MAT_Row1:
		{
			tmpA = SetBit(tmpA,0,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[0][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row2:
		{

			tmpA = SetBit(tmpA,1,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[1][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row3:
		{
			tmpA = SetBit(tmpA,2,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[2][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row4:
		{
			tmpA = SetBit(tmpA,3,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[3][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row5:
		{
			tmpA = SetBit(tmpA,4,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[4][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row6:
		{
			tmpA = SetBit(tmpA,5,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[5][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row7:
		{
			tmpA = SetBit(tmpA,6,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[6][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		case MAT_Row8:
		{
			tmpA = SetBit(tmpA,7,1);
			for (col = 0; col < 8; col++)
			{
				if (MatrixArrays[7][col] == 0x01)
				{
					tmpC = SetBit(tmpC, col, 0);
				}
			}
			break;
		}
		default:
		{
			break;
		}
		
	}
	
	PORTA = tmpA;
	PORTB = tmpC;
	
	return state;
}

enum obstacle_States{obsInit, obs0_P1,  obs0_P2, obs1_P1, obs1_P2, obs2_P1, obs2_P2, obs3_P1, obs3_P2, obs4_P1, obs4_P2};
int obstacle_Tick(int state){
	switch(state){
		case -1:
		{
			state = obsInit;
			break;
		}
		case init:
		{
			state = obs0_P1;
			break;
		}
		case obs0_P1:
		{
			state = obs0_P2;
			break;
		}
		case obs0_P2:
		{
			if(obsCount < 4){
				++obsCount;
				state = obs0_P2;
			}
			else{
				state = obs1_P1;
			}
			break;
		}
		case obs1_P1:
		{
			state = obs1_P2;
			break;
		}
		case obs1_P2:
		{
			if(obsCount < 4){
				++obsCount;
				state = obs1_P2;
			}
			else{
				state = obs2_P1;
			}
			break;
		}
		case obs2_P1:
		{
			state = obs2_P2;
			break;
		}
		case obs2_P2:
		{
			if(obsCount < 4){
				++obsCount;
				state = obs2_P2;
			}
			else{
				state = obs3_P1;
			}
			break;
		}
		case obs3_P1:
		{
			state = obs3_P2;
			break;
		}
		case obs3_P2:
		{
			if(obsCount < 4){
				++obsCount;
				state = obs3_P2;
			}
			else{
				state = obs0_P1;
			}
			break;
		}
		
	}
	switch(state){
		case obsInit:
		{
			obsCount = 0;
			obstacles[0].obstacleCol = 5;
			obstacles[0].obstacleRow = 4;
			obstacles[1].obstacleCol = 1;
			obstacles[1].obstacleRow = 1;
			obstacles[2].obstacleCol = 3;
			obstacles[2].obstacleRow = 5;
			obstacles[3].obstacleCol = 4;
			obstacles[3].obstacleRow = 2;
			obstacles[4].obstacleCol = 2;
			obstacles[4].obstacleRow = 3;
			break;
		}
		case obs0_P1:
		{
			obstacles[3].obstacleState = 0;
			obstacles[0].obstacleState = 1;
			obsCount = 0;
			break;
		}
		case obs0_P2:
		{
			break;
		}
		case obs1_P1:
		{
			obstacles[0].obstacleState = 0;
			obstacles[1].obstacleState = 1;
			obsCount = 0;
			break;
		}
		case obs1_P2:
		{
			break;
		}
		case obs2_P1:
		{
			obstacles[1].obstacleState = 0;
			obstacles[2].obstacleState = 1;
			obsCount = 0;
			break;
		}
		case obs2_P2:
		{
			break;
		}
		case obs3_P1:
		{
			obstacles[2].obstacleState = 0;
			obstacles[3].obstacleState = 1;
			obsCount = 0;
			break;
		}
		case obs3_P2:
		{
			break;
		}
	}
	return state;
}

int main(void)
{
	DDRA = 0xFF; PORTA = 0x00;
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xF8; PORTC = 0x07;
	
	unsigned long int SNES_Period_calc = 1;
	unsigned long int ctrl_Period_calc = 75;
	unsigned long int TickFct_MatrixOutput_calc = 1;
	unsigned long int obstacle_Period_calc = 750;
	
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(SNES_Period_calc, ctrl_Period_calc);
	tmpGCD = findGCD(tmpGCD, TickFct_MatrixOutput_calc);
	tmpGCD = findGCD(tmpGCD, obstacle_Period_calc);
	
	
	unsigned long int GCD = tmpGCD;
	
	unsigned long int SNES_Period = SNES_Period_calc/GCD;
	unsigned long int ctrl_Period = ctrl_Period_calc/GCD;
	unsigned long int TickFct_MatrixOutput_period = TickFct_MatrixOutput_calc/GCD;
	unsigned long int obstacle_period = obstacle_Period_calc/GCD;

	
	static task task1,task2,task3,task4;
	task *tasks[] = { &task1,&task2,&task3,&task4};
	const unsigned short numTasks = 4;//sizeof(tasks)/sizeof(task*);

	// Task 1
	task1.state = -1;//Task initial state.
	task1.period = SNES_Period;//Task Period.
	task1.elapsedTime = SNES_Period;//Task current elapsed time.
	task1.TickFct = &SNES_Tick;//Function pointer for the tick.

	// Task 2
	task2.state = -1;//Task initial state.
	task2.period = ctrl_Period;//Task Period.
	task2.elapsedTime = ctrl_Period;//Task current elapsed time.
	task2.TickFct = &ctrl_Tick;//Function pointer for the tick.
	
	// Task 3
	task3.state = -1;//Task initial state.
	task3.period = TickFct_MatrixOutput_period;//Task Period.
	task3.elapsedTime = TickFct_MatrixOutput_period;//Task current elapsed time.
	task3.TickFct = &TickFct_MatrixOutput;//Function pointer for the tick.
	
	task4.state = -1;//Task initial state.
	task4.period = obstacle_period;//Task Period.
	task4.elapsedTime = obstacle_period;//Task current elapsed time.
	task4.TickFct = &obstacle_Tick;//Function pointer for the tick.

	// Set the timer and turn it on
	TimerSet(GCD);
	TimerOn();

	unsigned short i; // Scheduler for-loop iterator

    while(1)
    {
		for ( i = 0; i < numTasks; i++ ) {
			// Task is ready to tick
			if ( tasks[i]->elapsedTime == tasks[i]->period ) {
				// Setting next state for task
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
				// Reset the elapsed time for next tick.
				tasks[i]->elapsedTime = 0;
			}
			tasks[i]->elapsedTime += 1;
		}
		while(!TimerFlag);
		TimerFlag = 0;

        //TODO:: Please write your application code 
    }
	return 0;
}