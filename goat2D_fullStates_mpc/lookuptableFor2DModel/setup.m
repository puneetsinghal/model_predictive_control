% By; Hadi Salman
% Date: 09/27/2016
%% Run this code at the beginning always
clear all; close all;
global leg 
%initialize leg;
leg.l1 = 5.93;%[inch]
leg.l2 = 13.8;%[inch]

%% hip 1, 2, and 3 of the leg

[leg.h1, leg.h2] = goat_initialize();

