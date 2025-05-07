%PID Control
P = 3.504;
I = 0.001;
D = 10;
slip_ref = 0.2;

%ABS Control
inc_ratio  = 1;
drop_ratio = 0.8;
% EV
Eeta_EV = 0.9;

%Vehicle Body
M_body      = 1200;      %kg %vehicle body mass
CG_Front    = 1.4;       %m  %Horizontal distance from CG to front axle
CG_Rear     = 1.6;       %m  %Horizontal distance from CG to Rear axle
CG_height   = 0.5;       %m  %CG height above ground
Front_Area  = 3;         %m^2%Frontal Area
Drag_Cof    = 0.4;       %   %Drag Coefficient
Intial_vel  = 30;        %m/s   %initial velocity

%Tire
RVL         = 3000;      %N  %Rated Vertical Load
PLF         = 3500;      %N  %Peak Longitudinal Force at rated Load
Slip        = slip_ref;      %per%Slip at peak force at rated load (percent)
RR          = 0.3;       %m  %Rolling raduis
TI          = 1;       %kg.m^2    %Tire Inertia

%Disk Brake
MPR         = 150;       %mm %Mean Pad Raduis
CB          = 18;        %mm %Cylinder Bore
SFC         = 0.9;       %   %Static Friction Coefficient
CFC         = 0.7;       %   %Coulomb Friction Coefficient
BFV         = 0.1;       %rad/s  %Breakaway friction velocity
VFC         = 0;         %N*m*s/read %Viscous Friction Coefficient

%Braking Pressure
A_master    = 0.0003;    %m^2%Area of master cylinder
P_master    = 10^6;      %Pa %Pressure of master cylinder
A_booster   = 0.025;     %m^2%Area of booster
P_booster   = 8*10^5;    %Pa %Pressure of booster



