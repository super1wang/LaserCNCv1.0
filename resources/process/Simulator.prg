#/ Controller version = 3.12
#/ Date = 5/11/2026 11:57 PM
#/ User remarks =
#0

!------------------------system init-----------------------------------
SYSTEM_INIT:
REAL TimeOut; TimeOut = 10000

! Safety Parameters must be set here
ENABLE Z; ! ENABLE GANTRY AXIS
ENABLE B;
ENABLE 0; ! ENABLE THE VERTICAL Z AXIS

!GLOBAL INT ZHome;ZHome=1
!INT GLOBAL  XHome;XHome=1
!GLOBAL INT GantryHome;GantryHome=1

STOP

#1

!--------------------Gantry  Homing Program-----------------------------
Y_HOME:


GLOBAL INT GantryHome;GantryHome=0!
GantryHome=1

STOP

#2

!--------------------------- X axis homming program-------------------------------
X_HOME:


INT GLOBAL  XHome;XHome=0
XHome=1

STOP

#3

! ----------------------------Z axis homming program--------------------------------
Z_HOME:




GLOBAL INT ZHome;ZHome=0

ZHome=1


STOP

#4



#5

! ----------------------------Z axis homming program--------------------------------
Z_HOME:




GLOBAL INT ZHome;ZHome=0
ZHome=1


STOP


#6

!------------------------write buffer--------------------------------

A_START:
! FIFO Buffer Simulation
GLOBAL INT giFifoHead, giFifoEnd ! Flag of FIFO's head and end
GLOBAL INT gbFifoFull, gbFifoEmpty ! FIFO's state
GLOBAL INT gbFifoInProcess ! FIFO's state is changing
GLOBAL INT FIFO_SIZE ! FIFO's size
FIFO_SIZE = 8000

! FIFO's element data structure
GLOBAL INT iSPMotionType(10001) ! Motion type
GLOBAL REAL rSPXEndP(10000), rSPYEndP(10000) ! Line parameter
GLOBAL REAL rSPArcCenterX(10000), rSPArcCenterY(10000) ! Arc parameter
GLOBAL REAL rSPPreX(10000), rSPPreY(10000) ! Arc parameter
GLOBAL REAL rSPMotionVel(10000) ! Arc parameter
GLOBAL REAL rSPMotionAccel(10000)
GLOBAL REAL rSPMotionJerk(10000)

! Interface between host program and ACSPL+ program
GLOBAL REAL rControlData(10)
rControlData(0)=0

CALL FIFO_INIT
WHILE 1
  CALL FIFO_INSERT
END

! FIFO Initialization
FIFO_INIT:
  giFifoHead = 0; giFifoEnd = 0
  gbFifoEmpty = 1; gbFifoFull = 0
  gbFifoInProcess = 0
RET

! FIFO IN
FIFO_INSERT:
  INT ii
  TILL ^gbFifoFull
  TILL rControlData(0)
  ii = giFifoHead + 1
  IF ii >= FIFO_SIZE; ii = 0; END
  iSPMotionType(giFifoHead) =  rControlData(0)
  rSPXEndP(giFifoHead) = rControlData(1)
  rSPYEndP(giFifoHead) = rControlData(2)
  rSPArcCenterX(giFifoHead) = rControlData(3)
  rSPArcCenterY(giFifoHead) = rControlData(4)
  rSPPreX(giFifoHead) = rControlData(5)
  rSPPreY(giFifoHead) = rControlData(6)
  rSPMotionVel(giFifoHead) = rControlData(7)
  rSPMotionAccel(giFifoHead) = rControlData(8)
  rSPMotionJerk(giFifoHead) = rControlData(9)

  TILL ^gbFifoInProcess; gbFifoInProcess = 1
    giFifoHead = ii
    IF giFifoHead = giFifoEnd; gbFifoFull = 1; END
    IF gbFifoEmpty; gbFifoEmpty = 0; END
  gbFifoInProcess = 0
  rControlData(0) = 0
RET
STOP

#7

!-------------------------------read buffer-------------------------

B_START:
! FIFO Buffer Simulation
GLOBAL INT giFifoHead, giFifoEnd
GLOBAL INT gbFifoFull, gbFifoEmpty
GLOBAL INT gbFifoInProcess
GLOBAL INT FIFO_SIZE
GLOBAL INT EventType

GLOBAL INT iSPMotionType(10001)
GLOBAL REAL rSPXEndP(10000), rSPYEndP(10000)
GLOBAL REAL rSPArcCenterX(10000), rSPArcCenterY(10000)
GLOBAL REAL rSPPreX(10000), rSPPreY(10000)
GLOBAL REAL rSPMotionVel(10000)
GLOBAL REAL rSPMotionAccel(10000)
GLOBAL REAL rSPMotionJerk(10000)


global int laserStatus ; laserStatus = 0
GLOBAL REAL rControlData(10)
GLOBAL REAL LASER_ON_B_WAIT,LASER_ON_A_WAIT,LASER_OFF_B_WAIT,LASER_OFF_A_WAIT
!LASER_ON_B_WAIT=10;LASER_ON_A_WAIT=10;LASER_OFF_B_WAIT=10;LASER_OFF_A_WAIT=10
! FIFO OUT
INT MT_LINE; MT_LINE  = POW (2, 0)
INT MT_ARC1P; MT_ARC1P = POW (2, 1)
INT MT_ARC1N; MT_ARC1N = POW (2, 2)
INT MT_LASER_ON; MT_LASER_ON = POW (2, 3)
INT MT_LASER_OFF; MT_LASER_OFF = POW (2, 4)
INT MT_CUT_POS; MT_CUT_POS=POW(2,5)

! Cutting Trajectory Logging Variables 2011-03-10
GLOBAL INT giArrSize; giArrSize = 1000;
GLOBAL REAL gArrTrajectoryLog(1000)(10)
  ! the start index of storing, the next trajectory data will be stored here
GLOBAL INT giStoreStartIdx; giStoreStartIdx = 0
  ! the start index of checking
GLOBAL INT giCheckStartIdx; giCheckStartIdx= 0
! End of Variables

! Contnously interpret the cmmands in the FIFO queue
WHILE 1
  TILL ^gbFifoEmpty

  ! Store the trajectory log (Simple way, if full, it will replace the old rotary logs)
  gArrTrajectoryLog(giStoreStartIdx)(0) = rSPPreX(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(1) = rSPPreY(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(2) = rSPXEndP(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(3) = rSPYEndP(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(4) = iSPMotionType(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(5) = rSPArcCenterX(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(6) = rSPArcCenterY(giFifoEnd)
  gArrTrajectoryLog(giStoreStartIdx)(7) = B_APOS
  gArrTrajectoryLog(giStoreStartIdx)(8) = Z_APOS
  gArrTrajectoryLog(giStoreStartIdx)(9) = rSPMotionJerk(giFifoEnd)
  giStoreStartIdx = giStoreStartIdx + 1
  IF giStoreStartIdx >= giArrSize; giStoreStartIdx = 0; END
  ! End of Store Trajectory

  IF iSPMotionType(giFifoEnd) & MT_LINE
   ! ACC5 = rSPMotionAccel(giFifoEnd);DEC5 = rSPMotionAccel(giFifoEnd);
   ! IF MST0.#MOVE
   !    HALT 0; TILL ^MST0.#MOVE
   ! END
   JERK5 = rSPMotionJerk(giFifoEnd)
   if laserStatus = 0
     PTP/EV BZ, rSPXEndP(giFifoEnd), rSPYEndP(giFifoEnd),rSPMotionVel(giFifoEnd)
     TILL ^MST0.#MOVE
   elseif laserStatus = 1
      LINE BZ, rSPXEndP(giFifoEnd), rSPYEndP(giFifoEnd),rSPMotionVel(giFifoEnd)
      STOPPER BZ
      if GSFREE5<2
        GO BZ
      end
   end
    iSPMotionType(giFifoEnd) = iSPMotionType(giFifoEnd) - MT_LINE
  END

  IF iSPMotionType(giFifoEnd) & MT_ARC1P
    ACC5 = rSPMotionAccel(giFifoEnd);DEC5 = rSPMotionAccel(giFifoEnd);
    ARC1 BZ, rSPArcCenterX(giFifoEnd), rSPArcCenterY(giFifoEnd), rSPXEndP(giFifoEnd), rSPYEndP(giFifoEnd), +,rSPMotionVel(giFifoEnd)
    STOPPER BZ
    if GSFREE5<2
        GO BZ
    end
    iSPMotionType(giFifoEnd) = iSPMotionType(giFifoEnd) - MT_ARC1P
  END

  IF iSPMotionType(giFifoEnd) & MT_CUT_POS
    PTP/EV 0,  rSPXEndP(giFifoEnd), rSPMotionVel(giFifoEnd)
  END

  IF iSPMotionType(giFifoEnd) & MT_ARC1N
     ACC5 = rSPMotionAccel(giFifoEnd);DEC5 = rSPMotionAccel(giFifoEnd);
     ARC1 BZ, rSPArcCenterX(giFifoEnd), rSPArcCenterY(giFifoEnd), rSPXEndP(giFifoEnd), rSPYEndP(giFifoEnd), -,rSPMotionVel(giFifoEnd)
    STOPPER BZ
    if GSFREE5<2
       GO BZ
    end
    iSPMotionType(giFifoEnd) = iSPMotionType(giFifoEnd) - MT_ARC1N
  END

  IF iSPMotionType(giFifoEnd) & MT_LASER_ON

   laserStatus = 1
   MSEG/WV BZ,APOS5,APOS2

   IF rSPMotionVel(giFifoEnd)<=0.0
   ELSE
    HALT 0; TILL ^MST(0).#MOVE
    PTP/EV 0, rSPMotionVel(giFifoEnd),40000
    TILL ^MST0.#MOVE
   END
    WAIT LASER_ON_B_WAIT
    OUT0.4=1
    TILL OUT0.4
    WAIT LASER_ON_A_WAIT
    iSPMotionType(giFifoEnd) = iSPMotionType(giFifoEnd) - MT_LASER_ON
  END

  IF iSPMotionType(giFifoEnd) & MT_LASER_OFF

    ENDS BZ
    GO BZ
    SPLITALL
    laserStatus = 0


    WAIT LASER_OFF_B_WAIT
    OUT0.4=0
    TILL ^OUT0.4
    WAIT LASER_OFF_A_WAIT
    iSPMotionType(giFifoEnd) = iSPMotionType(giFifoEnd) - MT_LASER_OFF
    EventType=5
    INTERRUPT
  END


  TIll ^gbFifoInProcess; gbFifoInProcess = 1
    giFifoEnd = giFifoEnd + 1
    IF giFifoEnd >= FIFO_SIZE; giFifoEnd = 0; END
    IF giFifoEnd = giFifoHead; gbFifoEmpty = 1; END
    IF gbFifoFull; gbFifoFull = 0; END
  gbFifoInProcess = 0
  END
STOP

#8

GLOBAL INT EventType
GLOBAL INT ErrorCode
GLOBAL INT bInterLock
!--------------------------------on...return----------------------------------
!*******************Motor Limit error Detect**********************

GLOBAL INT XHome
ON MST5.#ENABLED & FAULT5.#LL & XHome
JERK(5) = JERK(5)/10; ACC(5) = ACC(5)/10; DEC(5) = DEC(5)/10
HALT 5
JOG/V 5,30000,+; TILL ^FAULT5.#LL
HALT 5
FCLEAR 5
JERK(5) = JERK(5)*10; ACC(5) = ACC(5)*10; DEC(5) = DEC(5)*10
EventType=2
INTERRUPT
RET

ON MST5.#ENABLED & FAULT5.#RL & XHome
JERK(5) = JERK(5)/10; ACC(5) = ACC(5)/10; DEC(5) = DEC(5)/10
HALT 5
JOG/V 5,30000,-; TILL ^FAULT5.#RL
HALT 5
FCLEAR 5
JERK(5) = JERK(5)*10; ACC(5) = ACC(5)*10; DEC(5) = DEC(5)*10
EventType=2
INTERRUPT
RET

ON MST5.#ENABLED & FAULT5.#SRL & XHome
JERK(5) = JERK(5)/10; ACC(5) = ACC(5)/10; DEC(5) = DEC(5)/10
HALT 5
JOG/V 5,500000,-;TILL ^FAULT5.#SRL
HALT 5
FCLEAR 5
JERK(5) = JERK(5)*10; ACC(5) = ACC(5)*10; DEC(5) = DEC(5)*10
EventType=2
INTERRUPT
RET

ON MST5.#ENABLED & FAULT5.#SLL & XHome
JERK(5) = JERK(5)/10; ACC(5) = ACC(5)/10; DEC(5) = DEC(5)/10
HALT 5
JOG/V 5,500000,+;TILL ^FAULT5.#SLL
HALT 5
FCLEAR 5
JERK(5) = JERK(5)*10; ACC(5) = ACC(5)*10; DEC(5) = DEC(5)*10
EventType=2
INTERRUPT
RET

GLOBAL INT GantryHome
ON MST2.#ENABLED & FAULT2.#LL & GantryHome
JERK(2) = JERK(2)/10; ACC(2) = ACC(2)/10; DEC(2) = DEC(2)/10
HALT 2
JOG/V 2,30000,+; TILL ^FAULT2.#LL
HALT 2
FCLEAR 2
JERK(2) = JERK(2)*10; ACC(2) = ACC(2)*10; DEC(2) = DEC(2)*10
EventType=2
INTERRUPT
RET

ON MST2.#ENABLED & FAULT2.#RL & GantryHome
JERK(2) = JERK(2)/10; ACC(2) = ACC(2)/10; DEC(2) = DEC(2)/10
HALT 2
JOG/V 2,30000,-; TILL ^FAULT2.#RL
HALT 2
FCLEAR 2
JERK(2) = JERK(2)*10; ACC(2) = ACC(2)*10; DEC(2) = DEC(2)*10
EventType=2
INTERRUPT
RET

ON MST2.#ENABLED & FAULT2.#SRL & GantryHome
JERK(2) = JERK(2)/10; ACC(2) = ACC(2)/10; DEC(2) = DEC(2)/10
HALT 2
JOG/V 2,500000,-;TILL ^FAULT2.#SRL
HALT 2
FCLEAR 2
JERK(2) = JERK(2)*10; ACC(2) = ACC(2)*10; DEC(2) = DEC(2)*10
EventType=2
INTERRUPT
RET

ON MST2.#ENABLED & FAULT2.#SLL & GantryHome
JERK(2) = JERK(2)/10; ACC(2) = ACC(2)/10; DEC(2) = DEC(2)/10
HALT 2
JOG/V 2,500000,+;TILL ^FAULT2.#SLL
HALT 2
FCLEAR 2
JERK(2) = JERK(2)*10; ACC(2) = ACC(2)*10; DEC(2) = DEC(2)*10
EventType=2
INTERRUPT
RET

GLOBAL INT ZHome
ON MST0.#ENABLED & FAULT0.#LL & ZHome
JERK(0) = JERK(0)/10; ACC(0) = ACC(0)/10; DEC(0) = DEC(0)/10
HALT 0
JOG/V 0,30000,+; TILL ^FAULT0.#LL
HALT 0
FCLEAR 0
JERK(0) = JERK(0)*10; ACC(0) = ACC(0)*10; DEC(0) = DEC(0)*10
EventType=2
INTERRUPT
RET

ON MST0.#ENABLED & FAULT0.#RL & ZHome
JERK(0) = JERK(0)/10; ACC(0) = ACC(0)/10; DEC(0) = DEC(0)/10
HALT 0
JOG/V 0,30000,-; TILL ^FAULT0.#RL
HALT 0
FCLEAR 0
JERK(0) = JERK(0)*10; ACC(0) = ACC(0)*10; DEC(0) = DEC(0)*10
EventType=2
INTERRUPT
RET

ON MST0.#ENABLED & FAULT0.#SRL & ZHome
JERK(0) = JERK(0)/10; ACC(0) = ACC(0)/10; DEC(0) = DEC(0)/10
HALT 0
JOG/V 0,500000,-;TILL ^FAULT0.#SRL
HALT 0
FCLEAR 0
JERK(0) = JERK(0)*10; ACC(0) = ACC(0)*10; DEC(0) = DEC(0)*10
EventType=2
INTERRUPT
RET

ON MST0.#ENABLED & FAULT0.#SLL & ZHome
JERK(0) = JERK(0)/10; ACC(0) = ACC(0)/10; DEC(0) = DEC(0)/10
HALT 0
JOG/V 0,500000,+;TILL ^FAULT0.#SLL
HALT 0
FCLEAR 0
JERK(0) = JERK(0)*10; ACC(0) = ACC(0)*10; DEC(0) = DEC(0)*10
EventType=2
INTERRUPT
RET


!******************buffer error detect***********************

ON (PERR0>=3020 & PERR0<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR0
INTERRUPT
RET

ON (PERR1>=3020 & PERR1<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR1
INTERRUPT
RET

ON (PERR2>=3020 & PERR2<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR2
INTERRUPT
RET

ON (PERR3>=3020 & PERR3<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR3
INTERRUPT
RET

ON (PERR4>=3020 & PERR4<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR4
INTERRUPT
RET

ON (PERR5>=3020 & PERR5<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR5
INTERRUPT
RET

ON (PERR6>=3020 & PERR6<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR6
INTERRUPT
RET

ON (PERR7>=3020 & PERR7<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR7
INTERRUPT
RET

ON (PERR8>=3020 & PERR8<=3999)
OUT0.4=0
OUT0.0=0
DISABLE B
DISABLE Z
DISABLE 0
EventType=1
ErrorCode=PERR8
INTERRUPT
RET


!*******************motor error detect**********************

ON (MERR5>=5009 & MERR5<=5999)
OUT0.4=0
OUT0.0=0
DISABLE 5
DISABLE 2
DISABLE 0
EventType=1
ErrorCode=MERR5
INTERRUPT
RET

ON (MERR2>=5009 & MERR2<=5999)
OUT0.4=0
OUT0.0=0
DISABLE 5
DISABLE 2
DISABLE 0
EventType=1
ErrorCode=MERR2
INTERRUPT
RET

ON (MERR0>=5009 & MERR0<=5999)
OUT0.4=0
OUT0.0=0
DISABLE 5
DISABLE 2
DISABLE 0
EventType=1
ErrorCode=MERR0
INTERRUPT
RET


ON IN0.1
bInterLock=0
RET

ON ^IN0.1
bInterLock=1
RET

ON bInterLock
EventType=3
INTERRUPT
RET

ON ^bInterLock
EventType=4
INTERRUPT
RET

#9
ACC0 = 100;DEC0 = 100;JERK0 = 1000;
ACC1 = 100;DEC1 = 100;JERK1 = 1000;
ACC2 = 100;DEC2 = 100;JERK2 = 1000;
PTP/EV 2,0,10
TILL ^MST(2).#MOVE
PTP/EV 0, 4.6304523640455155, 20
PTP/EV 1, 15.82075593188514, 20
PTP/EV 2,0,10
TILL ^MST(2).#MOVE
OUT0.2=1;
WAIT 0
WAIT 0
OUT0.4=1;TILL OUT0.4;
WAIT 0
ACC0 = 100;DEC0 = 100;JERK0 = 1000;
ACC1 = 100;DEC1 = 100;JERK1 = 1000;
ACC2 = 100;DEC2 = 100;JERK2 = 1000;
XSEG/VFJA (0, 1), APOS0, APOS1, 11, 1,1,0.017453292222222222
LINE/V (0, 1), 11.701520175910991, 22.891823743750614, 11
IF GSFREE0<2; GO (0, 1); END
ENDS (0, 1)
GO (0, 1)
SPLIT (0, 1)
WAIT 0
OUT0.4=0;TILL ^OUT0.4;
WAIT 0
kill 2
STOP
#13
GLOBAL REAL XPOSTABLE(4),MATTABLE(4)
XPOSTABLE(0)=0;XPOSTABLE(1)=1;XPOSTABLE(2)=2;XPOSTABLE(3)=3
MATTABLE(0)=0;
MATTABLE(1)=0.1;
MATTABLE(2)=0.2;
MATTABLE(3)=0.3;
!ptp/ev (0,4),0,0,1
MFLAGS(4).17=0
CONNECT RPOS(4)=MAPN(APOS(0), XPOSTABLE, MATTABLE)
DEPENDS 4,(4,0)
ptp/ev (0), 1,1;
wait 1000
ptp/ev (0), 2,1;
wait 1000
ptp/ev (0), 3,1;
wait 1000
MFLAGS(4).17=1
STOP

#A
!axisdef X=0,Y=1,Z=2,T=3,A=4,B=5,C=6,D=7
!axisdef x=0,y=1,z=2,t=3,a=4,b=5,c=6,d=7
global int I(100),I0,I1,I2,I3,I4,I5,I6,I7,I8,I9,I90,I91,I92,I93,I94,I95,I96,I97,I98,I99
global real V(100),V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V90,V91,V92,V93,V94,V95,V96,V97,V98,V99
