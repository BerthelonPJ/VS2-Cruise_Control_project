Created By Pierre-Jean Bertehlon
15 Oct. 2018.

This repository is supposed to be a project I realized with another colleague during the course Vehicle Systems 2 i followed when I was studying as an exchange student at the LTU.

We used an AVR board, AT90CAN128.

Purpose of the projet:
  -The Objective of this project is to build ( or simulate ) a cruise control manager on a car. 
  -The program should allow the user to enable or disable the system whenever he wants.
  -It should also disable the cruise control when the brake lever is pushed (represented by a push button).
  -The wheel speed will be simulated by a potentiometer available through the ADC of the board.
  -The desired speed can be increased or decreased by pushing two buttons. 
  -Also all the informations will be displayed on the board display, but if needed we can send it to a computer using the USART connector.
  -With a last button we can cycle through the menu, e.g choose wether to display temperature, accelerometer, speed or steering angle.
  
