# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt #only for running examples
from scipy.spatial import Delaunay

def ProjPnt2Line(P,q):
#==============================================================================
#Input: 
#q - [x,y] point to be project onto line P
#P - 2x2 [[x1,y1], points representing a line
#         [x2,y2]]  
    
#Output:
#projq  [x,y] np array representing point resulting in projecting q on P

#Example:
#P=np.array([[0,0],
#           [1,0]])
#q=np.array([0.5,4])
#projq=ProjPnt2Line(P,q)
#plt.scatter(P[:,0],P[:,1],color=[1,0,0])
#plt.scatter(q[0],q[1],color=[0,0,0])
#plt.scatter(projq[0],projq[1],color=[0,0,1])
#==============================================================================
    p1=P[0,:]; p2=P[1,:]
    t=(p2-p1)/np.linalg.norm(p2-p1) #unit vector in p1-p2
    projq=np.dot((q-p1),t)*t+p1; #find project point
    return projq

def SphereFilter(Cones,CarCG,R):
# =============================================================================
#Input:
#Cones - mx3 matrix of cones [x,y,color]
#CarCG - car center [x,y]
#CarDir - car direction [x,y] - normalized
#R - radius of hemisphere

#Output:
#FilteredCones - same format as cones
#only cones within sphere with radius R starting at car
#FInd - bool array FilteredCones=Cones(FInd,:)
    
#Example:
#=np.sqrt(2)
#Cones=np.array([[-1,0.5,98],
#                 [1,1,121],
#                 [1,-0.6,98]])
#CarCG=np.array([0,0.5])
#CarDir=np.array([0,1])
#SphereFilter(Cones,CarCG,R)
#PassCones, FInd=SphereFilter(Cones,CarCG,R) #run function on data
#FailCones=Cones[~FInd,:] #obtain fail cones
#t=np.linspace(0,2*np.pi,100) #for circle plotting
#plt.scatter(PassCones[:,0],PassCones[:,1],color=[0,1,0]) #plot pass cones
#plt.scatter(FailCones[:,0],FailCones[:,1],color=[1,0,0]) #plot fail cones
#plt.scatter(CarCG[0],CarCG[1],color=[0.5,0,0.5]) #plot CarCG
#plt.plot(CarCG[0]+R*np.cos(t),CarCG[1]+R*np.sin(t),ls='--',lw=2,color=[0,0,0]) #plot circle
#plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid
# =============================================================================
    
    ConesR=Cones[:,:2]-CarCG #vectors of cones relative to car
    SqDistance=np.diag(np.matmul(ConesR,np.transpose(ConesR)))
    FInd=SqDistance < R**2
    FilteredCones=Cones[FInd,:]
    return FilteredCones,FInd

def InCenter(P):
# =============================================================================
#Input:
#P - 3x2 [[x1,y1], points of a triangle
#          [x2,y2],  
#          [x3,y3]]
  
#Output:
#InCenter - [x,y] coordinates 
#InR - radius of InCircle

#From: 
#https://www.mathopenref.com/coordincenter.html
#https://keisan.casio.com/

#Example:
#t1=np.radians([-30,90,210])
#P=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1))])) #equilateral triangle
#InCenter, InR=InCenter(P)
#plt.plot(np.hstack([P[:,0],P[0,0]]),np.hstack([P[:,1],P[0,1]]),color=[0.5,0,0]) #plot triangle edges
#plt.scatter(P[:,0],P[:,1],color=[1,0,0]) #plot triangle points
#plt.scatter(InCenter[0],InCenter[1],color=[0,0,1]) #plot InCenter
#t2=np.linspace(0,2*np.pi,100) #for circle plotting
#plt.plot(InCenter[0]+InR*np.cos(t2),InCenter[1]+InR*np.sin(t2),ls='--',lw=2,color=[0,0,0]) #plot InCircle
#q=InCenter+InR*(P[0,:]-InCenter)/np.linalg.norm((P[0,:]-InCenter)) #find projection of point 1 on InCircle
#plt.plot([InCenter[0],q[0]],[InCenter[1],q[1]],lw=2,color=[0,0.7,0]) #plot in radius in the direction of point 1
#plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid
# =============================================================================

    A=P[0,:]; B=P[1,:]; C=P[2,:]
    a=np.linalg.norm(B-C); b=np.linalg.norm(A-C); c=np.linalg.norm(A-B);
    p=a+b+c
    InCenter=(a*A+b*B+c*C)/p
    s=p/2
    InR=np.sqrt(s*(s-a)*(s-b)*(s-c))/s
    return InCenter,InR

def CircumRadius(P):
# =============================================================================
#Input:
#P - 3x2 [[x1,y1], points of a triangle
#          [x2,y2],  
#          [x3,y3]]
    
#Output:
#CircumR=radius of CircumCircle
    
#From:
#http://mathworld.wolfram.com/Circumradius.html
    
#Example:
#t1=np.radians([-30,90,210])
#P=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1))])) #equilateral triangle
#CircumCenter=[0,0] #symmetry
#CircumR=CircumRadius(P)
#plt.plot(np.hstack([P[:,0],P[0,0]]),np.hstack([P[:,1],P[0,1]]),color=[0.5,0,0]) #plot triangle edges
#plt.scatter(P[:,0],P[:,1],color=[1,0,0]) #plot triangle points
#plt.scatter(CircumCenter[0],CircumCenter[1],color=[0,0,1]) #plot InCenter
#t2=np.linspace(0,2*np.pi,100) #for circle plotting
#plt.plot(CircumCenter[0]+CircumR*np.cos(t2),CircumCenter[1]+CircumR*np.sin(t2),ls='--',lw=2,color=[0,0,0]) #plot InCircle
#plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid    
# =============================================================================
    A=P[0,:]; B=P[1,:]; C=P[2,:]
    a=np.linalg.norm(B-C); b=np.linalg.norm(A-C); c=np.linalg.norm(A-B);
    CircumR=a*b*c/np.sqrt((a+b+c)*(a+b-c)*(a+c-b)*(b+c-a))
    return CircumR

def OutFacingNormal(P,InCenter):
# =============================================================================
#Input:
#InCenter [x,y] of a triangle incenter
#P - 2x2 [[x1,y1], points on triangle of the same edge
#          [x2,y2]]  
    
#Output:
#n - unit vector [x,y] normal to the edge specified by P, facing out of the triangle

#Example:
#t1=np.radians([-30,90,210])
#Ptri=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1))])) #equilateral triangle
#Pedge=Ptri[:2,:]
#InCenter,InR=InCenter(Ptri)
#q=np.mean(Pedge,axis=0) #mean by column
#n=OutFacingNormal(Pedge,InCenter)
#plt.plot(np.hstack([Ptri[:,0],Ptri[0,0]]),np.hstack([Ptri[:,1],Ptri[0,1]]),color=[0.5,0,0]) #plot triangle
#plt.scatter(Pedge[:,0],Pedge[:,1],color=[1,0,0]) #plot edge points
#plt.scatter(InCenter[0],InCenter[1],color=[0,0,1]) #plot InCenter
#plt.quiver(q[0],q[1],n[0],n[1])
#plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid  
# =============================================================================
    q=ProjPnt2Line(P,InCenter);
    n=(q-InCenter)/np.linalg.norm(q-InCenter);
    return n

def EdgeCost(InCenter,EdgeCones,u):
# =============================================================================
#Input:
#InCenter - [x,y] in center of triangle
#EdgeCones - [x,y,color] 2x3 of edge vertcies 
#u - [x,y] normalized direction of enterance to triangle

#Output:
#J - cost of passing through edge. |J|<2
    
#Example:
#t1=np.radians([-30,90,210])
#ConesColors=np.array([98,121,98])
#TriCones=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1)),ConesColors])) #equilateral triangle
#EdgeCones=TriCones[:2,:] #edge cones have different colors
#InCenter=InCenter(TriCones[:,:2])[0] #[0] after function call - returns the  first value
#u=OutFacingNormal(EdgeCones[:,:2],InCenter) #enterance direction - same direction as out facing normal of edge
#J=EdgeCost(InCenter,EdgeCones,u)
#print(J) #expect to be -2
# =============================================================================
        J=0 #initalize
        if EdgeCones[0,2]==EdgeCones[1,2]: #if the same color
            J=J+1
        else: #not the same color
            J=J-1
        J=J-np.dot(u,OutFacingNormal(EdgeCones[:,:2],InCenter)) #+bad points for difference in direction
        return J

def TriOne(DT,ConesColors,ID,CarDir):
# =============================================================================
# Input:
# DT - DelaunayTriangulation containing DT.points and DT.simplices
# ConesColors - colors of cones (98 for blue, 121 for yellow) with same
# indexing as DT.Points
# ID - number of triangle in DT.simplices (row index)
# CarDir - car direction [x,y] normalized
# 
# Output:
# NewID - number of new triangle in DT.ConnectivityList (row index)
# Newu - direction of enterance to new triangle (NewID)
# NewCrossEdge - [V1,V2] of edge that we crossed to get from ID->NewID
# V1 and V2 refer to vertex indcies (row) of DT.points
# =============================================================================

    V=DT.simplices[ID,:] #find vertex indcies of ID
    P=np.hstack([(DT.points[V,:]),ConesColors[V].reshape(3,1)]) #Triangle cones
    IC=InCenter(P[:,:2])[0] #find incenter of ID
    EdgesInd=np.array([[0,1], #Edge1 #build indcies in V mapping to EdgesV by V[EdgesInd]
                       [0,2], #Edge2 
                       [1,2]])#Edge3
    J1=EdgeCost(IC,P[EdgesInd[0,:],:],CarDir)
    J2=EdgeCost(IC,P[EdgesInd[1,:],:],CarDir)
    J3=EdgeCost(IC,P[EdgesInd[2,:],:],CarDir)
    J=np.hstack([J1,J2,J3]); JminInd=np.argmin(J)
    NewCrossEdge=V[EdgesInd[JminInd,:]] #obtain Edge Vertices to cross (point indcies [V1,V2])
    Newu=OutFacingNormal(P[EdgesInd[JminInd,:],:2],IC) #calculate normal to cross edge
    Attachments=np.where(np.any(DT.simplices==NewCrossEdge[0],axis=1) &\
                         np.any(DT.simplices==NewCrossEdge[1],axis=1)) #find IDs that are connected to edge
    NewID=np.squeeze(np.setdiff1d(Attachments,ID)) #NewID - New Triangle
    return NewID,Newu,NewCrossEdge

def FindNextTriangle(DT,ConesColors,ID,Dir,CrossEdge):
# =============================================================================
# Input:
#DT - DelaunayTriangulation containing DT.points and DT.simplices
#ConesColors - colors of cones (98 for blue, 121 for yellow) with same
#indexing as DT.Points
#ID - number of triangle in DT.ConnectivityList (row index)
#Dir - direction of enterance to triangle ID
#CrossEdge - [V1,V2] of edge that was crossed to enter triangle ID
#V1 and V2 refer to vertex indcies (row) in DT.Points
    
#Output:
#NewID - number of new triangle in DT.ConnectivityList (row index)
#NewDir - direction of enterance to new triangle (NewID)
#NewCrossEdge - [V1,V2] of edge that we crossed to get from ID->NewID
#V1 and V2 refer to vertex indcies (row) in DT.Points
#MinCost - price in cost function with which algorithm decided to go
#RRatio - ratio between Circumcenter/InCenter radii
# =============================================================================

    V=DT.simplices[ID,:] #find vertex indcies of ID
    P=np.hstack([DT.points[V,:],ConesColors[V].reshape(3,1)]) #Triangle cones
    IC=InCenter(P[:,:2])[0] #find incenter of ID
    EdgesInd=np.array([[0,1], #Edge1 #build indcies in V mapping to EdgesV by V[EdgesInd]
                       [0,2], #Edge2 
                       [1,2]])#Edge3
    EdgesV=V[EdgesInd]
    EdgesInd=EdgesInd[~(np.any(EdgesV==CrossEdge[0],axis=1) &\
                        np.any(EdgesV==CrossEdge[1],axis=1)),:] #find the two edges to calculate for (not going backwards).
    J1=EdgeCost(IC,P[EdgesInd[0,:],:],Dir) #calculate costs
    J2=EdgeCost(IC,P[EdgesInd[1,:],:],Dir)
    J=np.hstack([J1,J2]); MinCost=np.min(J); JminInd=np.argmin(J) #find Edge to cross by row index in Edges
    NewCrossEdge=V[EdgesInd[JminInd,:]] #obtain Edge Vertices to cross (point indcies [V1,V2])
    NewDir=OutFacingNormal(P[EdgesInd[JminInd,:],:2],IC) #calculate normal to cross edge
    Attachments=np.where(np.any(DT.simplices==NewCrossEdge[0],axis=1) &\
                         np.any(DT.simplices==NewCrossEdge[1],axis=1)) #find IDs that are connected to edge
    NewID=np.squeeze(np.setdiff1d(Attachments,ID)) #NewID - New Triangle
    
    NewV=DT.simplices[NewID,:] #find vertex indcies of NewID (=NewTriangle)
#    NewV=NewV.reshape(3)
    NewP=DT.points[NewV,:] #find points correlating to new vertex indcies
    RRatio=CircumRadius(NewP)/InCenter(NewP)[1] #calculate RRatio of new triangle
    return NewID,NewDir,NewCrossEdge,MinCost,RRatio

def FindMiddle(OBCones,OYCones):
# =============================================================================
#Input:
    #BCones - blue cones in order for interpolation [x,y]
    #Ycones - yellow cones in order for interpolation [x,y]
#Output:
    #MiddlePoints - ordered points of middle of track [x,y]
# =============================================================================

    #decide on inner and outer cones by amount (infantile)
    Bamnt=OBCones.shape[0]; Yamnt=OYCones.shape[0] 
    Mamnt=np.minimum(Bamnt,Yamnt) #amount of middle points - as inner track cones
    Ind=np.argmin([Bamnt,Yamnt]) #index of minimum between [Bamnt,Yamnt]
    if Ind==0:
        Inxy=OBCones; Outxy=OYCones
    else:
        Inxy=OYCones; Outxy=OBCones
    
    #Build the middle points    
    OMxy=np.zeros([Mamnt,2])    
    for k in range(0,Mamnt):
        InCone=Inxy[k,:]
        Ind=np.argmin(np.diag(np.matmul(Outxy-InCone,np.transpose(Outxy-InCone)))) #find closest OutCone to k-th InCone
        OutCone=Outxy[Ind,:]
        OMxy[k,:]=(InCone+OutCone)/2
        
    return OMxy

def RotateVector(n,Theta):
    # =========================================================================
#Input:
    #n - row vector [x,y]
    #Theta - angle in radians to rotate by 
#Output:
    #v - row rotated vector [x,y]
# =============================================================================    
    Q=np.array([[np.cos(Theta),-np.sin(Theta)],
               [np.sin(Theta),np.cos(Theta)]]) #Rotation matrix
    v=np.transpose(np.matmul(Q,np.transpose(n)))
    return v
  

def OrderCones(Cones,CarCG,CarVel,MaxItrAmnt=50,CostThreshold=-0.2,\
               RRatioThreshold=8,SphereR=25000,CarLength=5,Angle4FakeCone=np.pi/3):
# =============================================================================
#Input:
#Cones - double matrix mx3 [x,y,color]. color = 98(blue)/121(yellow)
#CarCG - double [x,y] of geometric center of car
#CarDir - double [x,y] of car velocity (direcational)

#Output:
#BCones - blue cones in order for interpolation [x,y]
#Ycones - yellow cones in order for interpolation [x,y]
# =============================================================================
    #initalize
    Yind=np.full([MaxItrAmnt,1],np.nan)
    Bind=np.full([MaxItrAmnt,1],np.nan)
    
    #Preprocessing
    CarDir=CarVel/np.linalg.norm(CarVel); #normalize velocity to unit direction
    Cones=SphereFilter(Cones,CarCG,SphereR)[0] #filtering for radius around car
    
    #Fix new points - fake cones - if no blue/yellow cones are found
    BaddFlag=~np.any(Cones[:,2]==98);  YaddFlag=~np.any(Cones[:,2]==121); 
    if BaddFlag:
        BNew=CarCG+CarLength*RotateVector(CarDir,+Angle4FakeCone); #add cone to the left
        Cones=np.vstack([Cones,np.hstack([BNew,98])]) #add fake yellow cone @ the buttom of matrix
    if YaddFlag: #no yellow cones exist
        YNew=CarCG+CarLength*RotateVector(CarDir,-Angle4FakeCone); #add cone to the right
        Cones=np.vstack([Cones,np.hstack([YNew,121])]) #add fake yellow cone @ the buttom of matrix
    
    #Triangulate
    DT=Delaunay(Cones[:,:2]) #Triangulate only /w Cones
    ID=DT.find_simplex(CarCG) #attempt to find triangle which contains CarCG
    if (ID==-1): #if CarCG isoutside of convex hull
        Cones=np.vstack([Cones,np.hstack([CarCG,0])]) #add Car to Cones as a fake cone
        DT=Delaunay(Cones[:,:2])#Triangulate /w Cones+CarCG
        ID=DT.find_simplex(DT,CarCG+0.5*CarLength*CarDir) #find first triangle to work with  
    
    NewID,Newu,NewCrossEdge=TriOne(DT,Cones[:,2],ID,CarDir)
    if NewID==-1: return #crossed into no-man's land
    
    #insert first cones into lists (Bind/Yind)
    if Cones[NewCrossEdge[0],2]==ord('y'): 
        Yind[0]=NewCrossEdge[0]
    else: 
        Bind[0]=NewCrossEdge[0]
    if Cones[NewCrossEdge[1],2]==ord('y'): 
        Yind[0]=NewCrossEdge[1]
    else: 
        Bind[0]=NewCrossEdge[1]
    NewV=np.setdiff1d(DT.simplices[NewID,:],NewCrossEdge);
    if Cones[NewV,2]==ord('y'): 
        Yind[1]=NewV 
    else: 
        Bind[1]=NewV
    
    for Itr in range(2,MaxItrAmnt):
        #find next triangle
        NewID,Newu,NewCrossEdge,Cost,RRatio=FindNextTriangle(DT,Cones[:,2],NewID,Newu,NewCrossEdge)
        #check conditions, if not good enough - break
        if NewID==-1 or CostThreshold<Cost or RRatioThreshold<RRatio:
            break
        #add next cone to Bind/Yind
        NewV=np.setdiff1d(DT.simplices[NewID,:],NewCrossEdge)
        if Cones[NewV,2]==ord('y'):
            Yind[Itr]=NewV
        else: 
            Bind[Itr]=NewV
    
    #create output
    Bind=Bind[~np.isnan(Bind)].astype(int); Yind=Yind[~np.isnan(Yind)].astype(int) #delete rows with NaNs
    BCones=Cones[Bind,:2]; YCones=Cones[Yind,:2];
    
    #Find middle
    MidPoints=FindMiddle(BCones,YCones)
    
    return MidPoints

