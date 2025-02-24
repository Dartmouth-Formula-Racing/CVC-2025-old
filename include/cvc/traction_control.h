/* 
Author: Sasha Ries
Date: 2/10/25
File: traction_control.h
Decription: Header file for traction_control.c
*/



/* Function to calculate slip ratio given the front and rear filtered wheel speeds
Takes in no arguments as data is accessed from the master array: CVC_data */
void calculate_slip_ratio();