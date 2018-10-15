# AS2p
E0009E Automotive Systems 2: Parallel version

A car is not anymore just a mechanical system anymore.
It is a mechatronic system of systems with electrical and electonic systems on a chassis.
(With connections to the Internet, it becomes a cyber phyrical system (CPS).)
The advantage added by the electronics is that several inputs can be used to make the correct decision about a process.
For example, an engine air fuel mixture does not depend only on RPM but also ambiant air temperature, pressure, engine temperature,...

The aim of this course is to enpower a student to understand and maybe design both the hardware and software in these mechatronic systems.
We use an old processor to flaten the learning curve.
It is only at the end of the course that we introduce micro controllers and systems on chip.

The text below this line in this README file is updated with each commit to enable a student to navigate rapidly between the diffent branch and commits.

# Version's aim
In this commit, we introduce Serial Communication with the USART peripheral and enable communication with a PC via USB.
In the new state (USART) one can stream the ADC values, echo what is typed on the PC or send the text (input) to the PC.
The receiving of character is done with an internal interrupt.

## Exercises
- Log the ADC values and plot them in MATLAB or Microsoft Excel.
- Clean the second line of the display of previous text.
- In the USAST uECHO mode, clear the second line if the user sends a new line (\n) or a carriage return (\r).
- Connect two boards and communicate using USART0.
- Can you build a null modem with USART0?