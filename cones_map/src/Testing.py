# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 18:21:25 2019

@author: Alon
"""

import OrderConesDT
import matplotlib.pyplot as plt
import PurePursuit

Json=OrderConesDT.JsonExpi('Json_1.txt')
Fig=plt.figure(figsize=(18,16))
Ax=Fig.add_subplot(111)
Ax.axis('equal')
MovieFreq=20.0 #Hz

StopFlag=True
for Itr in range(400,Json.FrameAmount):
    [Cones,CarCG,CarDir]=Json.GetFrame(Itr)

    MapTrack = OrderConesDT.MapTrack(Cones, CarCG, CarDir, StandardDistance=1,SphereRFactor=1,CarLengthFactor=0.1)  # Build class
    MapTrack.OrderCones(MaxItrAmnt=20, CostThreshold=-0.2, ColorCostWeight=0.4, \
               RRatioThreshold=10)
    MidPoints=MapTrack.FindMidPoints()
    MapTrack.PlotMap(Ax) #Map of cones and all related

    if MidPoints.size>0:
        PP=PurePursuit.PPAckerman(CarCG,CarDir,MidPoints,Lb=0.1,SymSteeringAngleBounds=12*3.1415/180,SphereRFactor=1,StandardDistance=1)
        PP.PlotMap(Ax) #plot PurePersuit related

    #last movie stuff
    Ax.set_title('frame number %g, steering angle %g' %(Itr,round((180/3.1415)*PP.SteeringAngle)),fontsize=24)

    plt.draw()
    plt.pause(1.0/MovieFreq)
    #raw_input("Press Enter to continue...") #activate for one by one
    plt.cla()