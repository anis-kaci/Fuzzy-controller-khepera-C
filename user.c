/*****************************************************************************/
/* File:        user.c (Khepera Simulator)                                   */
/* Author:      Olivier MICHEL <om@alto.unice.fr>                            */
/* Date:        Thu Sep 21 14:39:05 1995                                     */
/* Description: example of user.c file                                       */
/*                                                                           */
/* Copyright (c) 1995                                                        */
/* Olivier MICHEL                                                            */
/* MAGE team, i3S laboratory,                                                */
/* CNRS, University of Nice - Sophia Antipolis, FRANCE                       */
/*                                                                           */
/* Permission is hereby granted to copy this package for free distribution.  */
/* The author's name and this copyright notice must be included in any copy. */
/* Commercial use is forbidden.                                              */
/*****************************************************************************/


#include "../SRC/include.h"
#include "user_info.h"
#include "user.h"

#define FORWARD_SPEED   5                    /* normal (slow) forward speed*/
#define TURN_SPEED      4                    /* normal (slow) turn speed */
#define COLLISION_TH    900                  /* value of IR sensors to be 
                                                  considered as collision */
int pas=0;


int max(int a, int b){
  if(a >=b){
    return a;
  }
  else{
    return b;
  }
}
void DrawStep()
{
  char text[256];

  sprintf(text,"step = %d",pas);
  Color(GREY);
  UndrawText(200,100,"step = 500");
  Color(BLUE);
  DrawText(200,100,text);
}

void UserInit(struct Robot *robot)
{
}

void UserClose(struct Robot *robot)
{
}

void NewRobot(struct Robot *robot)
{
  pas = 0;
}

void LoadRobot(struct Robot *robot,FILE *file)
{
}

void SaveRobot(struct Robot *robot,FILE *file)
{
}

void RunRobotStart(struct Robot *robot)
{
  ShowUserInfo(2,1);
}

void RunRobotStop(struct Robot *robot)
{
  ShowUserInfo(1,1);
}


//set the parameters 
int param_loin[] = {0, 400, 700, 1023};
int param_proche[] = {300, 600, 900, 1023};
int param_moyen[] = {250, 500, 750};







/*
* Fonction loin
*/
float loin(int IR_value, int *params){
  if(IR_value < params[0]){
    printf("Valeur inexistante");
    return 0.0;
  } else if(IR_value >= params[0] && IR_value <= params[1]){
    return 1.0; 
  } else if(IR_value >= params[1] && IR_value <= params[2]){


    	float slope = -1.0 / (params[2] - params[1]);
    	float intercept = 1.0 - slope * params[1];

    	// Calcul de la distance en pourcentage
    	float distance_percentage = slope * IR_value + intercept;

    	// Retourne la distance en pourcentage
    	return distance_percentage;
  } else if(IR_value >= params[2] && IR_value <= params[3]) {
    return 0.0; 
  } else {
    printf("Valeur inexistante");
    return 0.0;
  }
}

/*
 * Fonction proche
 */
float proche(int IR_value, int *params)
{
  if (IR_value < params[0])
  {
    printf("Valeur inexistante");
    return 0.0;
  }
  else if (IR_value >= params[0] && IR_value <= params[1])
  {
    return 0.0; 
  }
  else if (IR_value >= params[1] && IR_value <= params[2])
  {
   

    float slope = 1.0 / (params[2] - params[1]);
    float intercept = -slope * params[1];

    float distance_percentage = slope * IR_value + intercept;

    return distance_percentage;
  }
  else if (IR_value >= params[2] && IR_value <= params[3])
  {
    return 1.0; 
  }
  else
  {
    printf("Valeur inexistante");
    return 0.0;
  }
}


/*
    Fonction moyen
*/

float moyen(int IR_value, int *params){
  if (IR_value < params[0]){
      printf("Valeur inexistante");
      return 0.0;
   } else if(IR_value >= params[0] && IR_value <= params[1]){

     float slope = 1.0 / (params[1] - params[0]);
     float intercept = -slope * params[0];
     return slope * IR_value + intercept;
   } else if(IR_value >= params[1] && IR_value <= params[2]){

     float slope = -1.0 / (params[2] - params[1]);
     float intercept = 1.0 - slope * params[1];
     return slope * IR_value + intercept;
   } else {
    printf("Valeur inexistante");
    return 0.0;
   }
}



// Fuzzification pour chaque capteur IR
void fuzzification(int IR_value, float *degrees)
{
  // Calcul des degrés d'appartenance aux ensembles flous
  degrees[0]= loin(IR_value, param_loin);
  degrees[1] = proche(IR_value, param_proche);
  degrees[2] = moyen(IR_value, param_moyen);

  // Vous pouvez afficher ces degrés d'appartenance si nécessaire
  printf("Degree d'appartenance:\n");
  printf("Loin: %.2f\n", degrees[0]);
  printf("Proche: %.2f\n", degrees[1]);
  printf("Moyen: %.2f\n", degrees[2]);
}




// Rule evaluation using minimum operator
float rule_evaluation(float a, float b, float c) {
    return fmin(a, fmin(b,c));
}



/*Rules : 

    - if right sensors feel something close → move left
    - if right sensors feel something far → move right
    - if right sensors feel something on mid distance → move a bit right
    - if left sensors feel something close → move right
    - if left sensors feel something far → move left
    - if left sensors feel something on mid distance → move a bit left



  combinations : 
     - if (rs close) OR (ls far) -> move left 
     - if (ls close) OR (rs far) -> move right



*/


/*
  Fonction pour defuzzification en utilisant méthode centre de gravité 
*/

float centroid_defuzzification(float turn_right_strength, float turn_left_strength) {
    int i;
    float centroid = 0.0;
    float sum_of_moments = 0.0;
    float sum_of_weights = 0.0;

    // Define the weights for turning right and left
    float weights[] = { 1.0, -1.0 }; // Assuming 1.0 for turning right, -1.0 for turning left

    float rule_strengths[] = { turn_right_strength, turn_left_strength }; // Obtained from rule evaluation

    for (i = 0; i < 2; ++i) { // Loop through turning right and left rules
        sum_of_moments += rule_strengths[i] * weights[i];
        sum_of_weights += rule_strengths[i];
    }

    // Avoid division by zero
    if (sum_of_weights != 0.0) {
        centroid = sum_of_moments / sum_of_weights;
    }

    return centroid;
}




boolean StepRobot(struct Robot *robot){

  int flago = 0;
  pas++;
  DrawStep();

     float degreesTruths0[3];
     float degreesTruths1[3];
     float degreesTruths2[3];


    //left sensors process

    float irs0_value = robot->IRSensor[0].DistanceValue; // Input value to be fuzzified

    float irs1_value = robot->IRSensor[1].DistanceValue;

    float irs2_value = robot->IRSensor[2].DistanceValue; 

    // Fuzzification phase
    fuzzification(irs0_value, degreesTruths0); // Example input range is 0 to 10
    
    fuzzification(irs1_value, degreesTruths1); // Example input range is 0 to 10
    
    fuzzification(irs2_value, degreesTruths2); // Example input range is 0 to 10

    //taking degrees of truth of each
    float degree_left_far_s0 = degreesTruths0[0];
    float degree_left_close_s0 = degreesTruths0[1];
    float degree_left_mid_s0 = degreesTruths0[2];


    float degree_left_far_s1 = degreesTruths1[0];
    float degree_left_close_s1 = degreesTruths1[1];
    float degree_left_mid_s1 = degreesTruths1[2];


    float degree_left_far_s2 = degreesTruths2[0];
    float degree_left_close_s2 = degreesTruths2[1];
    float degree_left_mid_s2 = degreesTruths2[2];






    //Right sensors process

    float irs3_value = robot->IRSensor[3].DistanceValue; // Input value to be fuzzified

    float irs4_value = robot->IRSensor[4].DistanceValue;

    float irs5_value = robot->IRSensor[5].DistanceValue; 

    // Fuzzification phase
    fuzzification(irs3_value, degreesTruths0); // Example input range is 0 to 10
    
    fuzzification(irs4_value, degreesTruths1); // Example input range is 0 to 10
    
    fuzzification(irs5_value, degreesTruths2); // Example input range is 0 to 10

    //taking degrees of truth of each
    float degree_right_far_s3 = degreesTruths0[0];
    float degree_right_close_s3 = degreesTruths0[1];
    float degree_right_mid_s3 = degreesTruths0[2];


    float degree_right_far_s4 = degreesTruths1[0];
    float degree_right_close_s4 = degreesTruths1[1];
    float degree_right_mid_s4 = degreesTruths1[2];


    float degree_right_far_s5 = degreesTruths2[0];
    float degree_right_close_s5 = degreesTruths2[1];
    float degree_right_mid_s5 = degreesTruths2[2];





    /*
      Rule Evaluation Phase 
      (inference phase using Mamdani min max)
    */
    //left sensors
    float rule_left_close = rule_evaluation(degree_left_close_s0, degree_left_close_s1, degree_left_close_s2);
    float rule_left_mid = rule_evaluation(degree_left_mid_s0, degree_left_mid_s1, degree_left_mid_s2);
    float rule_left_far = rule_evaluation(degree_left_far_s0, degree_left_far_s1, degree_left_far_s2);



    //right sensors 
    float rule_right_close = rule_evaluation(degree_right_close_s3, degree_right_close_s4, degree_right_close_s5);
    float rule_right_mid = rule_evaluation(degree_right_mid_s3, degree_right_mid_s4, degree_right_mid_s5);
    float rule_right_far = rule_evaluation(degree_right_far_s3, degree_right_far_s4, degree_right_far_s5);


    

    //code for right sensors (combining the rules)
    float turn_right_strength_right = fmax(rule_right_mid, rule_right_far);
    float turn_left_strength_right = rule_right_close;
    //deffuzify using centroid method 
    float defuzzOutputRight = fmax(turn_right_strength_right, turn_left_strength_right);


    //Decision making 
   if (defuzzOutputRight == turn_left_strength_right){

      robot->Motor[RIGHT].Value  = TURN_SPEED ;
      robot->Motor[LEFT].Value = - TURN_SPEED ;  
      flago = 1;
      //Turn left 
      

    }
    else{
      //default action go forward
      robot->Motor[LEFT].Value  = FORWARD_SPEED;
      robot->Motor[RIGHT].Value = FORWARD_SPEED;  /* else go forward (default) */
      printf("robot will go forward");
    }


    //code for left sensors
    float turn_left_strength_left = fmax(rule_left_mid, rule_left_far);
    float turn_right_strength_left = rule_left_close;
    //deffuzify using centroid method 
    float defuzzOutputLeft = fmax(turn_right_strength_left, turn_left_strength_left);


    //Decision making (if danger act if not go forward)
    if (defuzzOutputLeft == turn_right_strength_left){
      //Turn right
      robot->Motor[RIGHT].Value  =  - TURN_SPEED ;
      robot->Motor[LEFT].Value =  TURN_SPEED ; 
      flago=1;  
      
    }
    else{
      //default action go forward
      robot->Motor[LEFT].Value  = FORWARD_SPEED;
      robot->Motor[RIGHT].Value = FORWARD_SPEED;  /* else go forward (default) */
      printf("robot will go forward");
    }



    /*back sensors
      if ((robot->IRSensor[6].DistanceValue > COLLISION_TH)||  
      (robot->IRSensor[7].DistanceValue > COLLISION_TH))   
   return(FALSE);*/
  //else
   return(TRUE);



}

void FastStepRobot(struct Robot *robot)
{
}

void ResetRobot(struct Robot *robot)
{
  pas = 0;
}

void UserCommand(struct Robot *robot,char *text)
{
  WriteComment("unknown command"); /* no commands */
}

void DrawUserInfo(struct Robot *robot,u_char info,u_char page)
{
  char text[256];

  switch(info)
  {
    case 1:
      switch(page)
      {
        case 1: Color(MAGENTA);
                FillRectangle(0,0,40,40);
                Color(BLUE);
                DrawLine(100,100,160,180);
                Color(WHITE);
                DrawPoint(200,200);
                Color(YELLOW);
                DrawRectangle(240,100,80,40);
                Color(GREEN);
                DrawText(240,230,"hello world");
                break;
        case 2: Color(RED);
                DrawArc(200,50,100,100,0,360*64);
                Color(YELLOW);
                FillArc(225,75,50,50,0,360*64);
                Color(BLACK);
                DrawText(140,170,"This is the brain of the robot");
      }
      break;
    case 2:     DrawStep();
  }
}



