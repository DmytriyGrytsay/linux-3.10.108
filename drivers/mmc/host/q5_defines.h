/*
 * (C) Copyright 2010
 * Ricado Ribalda-Universidad Autonoma de Madrid-ricardo.ribalda@uam.es
 * This work has been supported by: QTechnology  http://qtec.com/
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef Q5_DEFINES
#define Q5_DEFINES

/*Bit macros*/
#define MASK(X) ((1<<X)-1)
#define SETREG(var,reg,value) (var&~(reg##_MASK<<reg##_SHIFT))|(((value)&reg##_MASK)<<reg##_SHIFT)
#define GETREG(var,reg) ((var>>reg##_SHIFT)&reg##_MASK)
#define REG_ON(reg) (reg##_MASK<<reg##_SHIFT)

enum channels {RED=0,GREEN,BLUE,IR1,IR2,ALL_CHANNELS=32};
#define LAST_CHANNEL IR2
#define MAX_CHANNEL (IR2+1)

/*Image list*/
enum {LAST=0,PREVIOUS};

#endif
