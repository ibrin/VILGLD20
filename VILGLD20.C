#use LCD2L.LIB
#use fk.lib
#use vdriver.lib

#use eziocmmn.lib
#use eziopbdv.lib
#use ezioplc.lib

#use srtk.lib
#nointerleave //force program functions to be compiled first
#define RUNKERNEL 1
/*
#define TroLoadPosSensor1		!DIGIN1 // trolley load position Station-A
#define TroLoadPosSensor2		!DIGIN2 // trolley load position Station-B
#define ReceiveSensor			!DIGIN3 // receive sensor from top bin
#define ReturnSensor				!DIGIN4 // /Return sensor End A 
#define TroBinAFullSensor	   !DIGIN5 // trolley bin A full sensor
#define TroBinAEmpSensor  	   !DIGIN6 // trolley bin A empty sensor
#define TroBinBFullSensor 	   !DIGIN7 // trolley bin B full sensor (mix side)
#define TroBinBEmpSensor  		!DIGIN8 // trolley bin B empty sensor (mix side)
#define GpPosSensorA  			!DIGIN9 // gp position sensor A (L3 to L5-3)
#define GpPosSensorB  			!DIGIN10 // gp position sensor B (L2 to L5-2)(mix side)
#define GPHopSensorA			   !DIGIN11 // hopper full/Emp sensor A 
#define GPHopSensorB		 	   !DIGIN12 // hopper full/Emp sensor B 
#define HopAGateOpenSensor		!DIGIN13 // hopper A gate open sensor
#define HopAGateCloseSensor	!DIGIN14 // hopper A gate close sensor
#define HopBGateOpenSensor		!DIGIN15 // hopper B gate open sensor
#define HopBGateCloseSensor	!DIGIN16 // hopper B gate close sensor

#define SafetySwitchLeftMix	Board 7 Bank B pin0 //(7*32+16) Mixer side SefetySW
#define SafetySwitchRight		Board 7 Bank B pin1 //(7*32+17) Platform side SafetySW
#define Auto/Manual				Board 7 Bank B pin2 //(7*32+18)
#define MoveBackward				Board 7 Bank B pin3 //(7*32+19) move to left mixer side
#define MoveForward				Board 7 Bank B pin4 //(7*32+20)
#define HopAGateOpenSW			Board 7 Bank B pin5 //(7*32+21)
#define HopAGateCloseSW			Board 7 Bank B pin6 //(7*32+22)
#define HopBGateOpenSW			Board 7 Bank B pin7 //(7*32+23)
#define HopBGateCloseSW			Board 7 Bank B pin8 //(7*32+24)
#define Reset						Board 7 Bank B pin9 //(7*32+25)
#define E/Stop						Board 7 Bank B pin10 //(7*32+26)
#define ReloadSensor				Board 7 Bank B pin11 //(7*32+27) Reload sensor End B 


OUTPUTS
#define TroGateAClose				OUT1 //troller gate A close 
#define TrogateAOpen					OUT2 //trolley gate A open
#define TroGateBClose				OUT3 //troller gate B close (mixer side)
#define TrogateBOpen					OUT4 //trolley gate B open
#define TroAPos						OUT5 //Send signal
#define SendAlarmSignal				OUT6 //send signal to top bin to stop alarm
#define MotorEnable					OUT7 
#define MotorRunBrake				OUT8
#define MotorFororBack				OUT9
#define SendSignalPos		OUT10 // Send signal 3 
#define xxx		OUT11 //not use
#define EmergencyFlashLight					OUT12
//#define TrolleyMotorIsolator					OUT13
#define Alarm							OUT14

ON 0 for all sensors using relays.  
//ON 0 for sensor without reflector  
//ON 1 for sensor with reflector 
*/

shared unsigned long LoadTimer;
shared unsigned long DropTimer1, DropTimer2, DropTime, DropTime3;
shared unsigned long TroFullTimerA, TroFullTimerB;
shared unsigned long TBGateTimer; 
shared unsigned long AlarmTimer, AlarmTime; 
shared unsigned long GpPosNoTimerA, GpPosNoTimerB; //02

shared int   RtnPosFlag;
shared int  auto, TroForStop, TroLoadPos,TroLoadPos1,TroLoadPos2, EndPos;
shared int	GpPos, GpPos1, GpPosFF, RtnPos;
shared int  TroFullSenA, TroFullSenB, TroEmpSen, MoveForF;
shared int 	HopSensor1, HopSensor2, Hop1GateSen, Hop2GateSen;
shared int 	Step1, Step2, Step21, Step3, Step4, Step41, Step5, Step51;
shared int 	SafetySW, TroLoadPosFlag, StopAuto, Begin;
shared int	Hop1drop, Hop2drop; 
shared int	GpPosF, GpPosF1, GpPosF2, GpPos1F, GpPos1F1, GpPos1F2; 
shared int	TroLoadPosL3BL2, TroLoadPosL3BL3;
shared int  AutoFlag, AutoFlag1, TroFullSenAF, TroFullSenAFOff, TroFullSenAF1, TroFullSenAF2;
shared int  TroFullSenBF, TroFullSenBFOff, TroFullSenBF1, TroFullSenBF2;
shared int  TroLoadMoveTime;
shared int  GateAClose, GateBClose, AutoStart, AutoStart1; 
shared int	DropTimeFlag1, DropTimeFlag2, DropTime1, DropTime2; 
shared int	AlarmTimerF; //08
shared int	GpPosNoA, GpPosNoB, GpPosNoFlagA, GpPosNoFlagB; //02
shared int	HopFullNoA, HopFullNoB, RestTime; //02
shared int	TroLoadPosF1, HopFullTime; //10 11
extern CoData move;
extern CoData gate;
extern CoData gateauto;
extern CoData manual;

struct tm Time;

main()
{
	RestTime=300; //02 If trolley is not empty when moving back, trolley moves forwards after RestTime (Sec)
	TroLoadMoveTime=180; //sec from 120
	/* AlarmTime=1200; //20 mins from 900 */
	AlarmTime=600; // 10 minutes or 600 sec - modification 02/10/2025
	DropTime=400;//Drop time from trolley to hopper from 50
	DropTime3=3000; //gate close time if sensor not working from 2000
	TroFullSenAF=1;
	TroFullSenBF=1;
	TroFullSenAFOff=1;
	TroFullSenBFOff=1;
	TroFullSenAF1=0;
	TroFullSenBF1=0; 
	TroFullSenAF2=0;
	TroFullSenBF2=0; 
	TroLoadPos=0;
	TroLoadPos1=0;
	TroLoadPosFlag=0;
	GpPos=0;
	GpPos1=0;
	GpPosFF=1;
	Step1=0;
	Step2=0;
	Step21=0;
	Step3=0;
	Step4=0;
	Step41=0;
	Step5=0;
	Step51=0;
	TroEmpSen=0;
	RtnPos=0;
	Hop1drop=0;
	Hop2drop=0;
	GpPosF=1; 
	GpPos1F=1; 
	AutoFlag=1;
	AutoFlag1=1;
	SafetySW = 1;
	AutoStart=0; 
	AutoStart1=0; 
	GateAClose=0;
	GateBClose=0;
	DropTime1=0; 
	DropTime2=0; 
	DropTimeFlag1=0; 
	DropTimeFlag2=0;
	up_setout(1,0);
	up_setout(2,0);
	up_setout(3,0);
	up_setout(4,0);
	up_setout(5,0);
	up_setout(7,0); //motor enable off
	AlarmTimerF=1; //08
	GpPosNoA=0; //02
	GpPosNoB=0; //02
	GpPosNoFlagA=0; //02
	GpPosNoFlagB=0; //02
	HopFullNoA=0; //02
	HopFullNoB=0; //02
	Begin=0;//04
	TroLoadPosF1=0;//10
	HopFullTime=0; //11
		
	VdInit();				// virtual driver needed by SRTK
	init_srtkernel(); 	// initialize the SRTK
	up_beepvol(1);			// set beeper low(2=high) volume 

	
	while(1)
	{
		runwatch();
	}
}

srtk_hightask()
{
	fk_monitorkeypad();  
//send alarm stop signal to top bin
	costate //v10
	{
		if ((plcXP81In(7*32+26)==1)&&((up_digin(1)==0)||(up_digin(2)==0))) //V9 
		{
			up_setout(6,1);
			AutoStart1=0;
		}
		else
		{
			if (((up_digin(1)==0)||(up_digin(2)==0))&&(TroLoadPosF1==0))
			{
				up_setout(6,1);
				waitfor(DelayMs(100));
				up_setout(6,0);
				TroLoadPosF1=1;
			}
			else
			{
				up_setout(6,0);
			}
		}
	}
	if (up_digin(1)==1)
	{
		TroLoadPosF1=0;
	}

//GP head counting 02 start
	if (RtnPos==1)   
	{
		if ((up_digin(9)==0)&&(GpPosNoFlagA==0))
		{
			GpPosNoA += 1;
			if (up_digin(11)==0) //Hopper sensorA full
			{
				HopFullNoA +=1;
			}
			GpPosNoFlagA=1;
			GpPosNoTimerA=MS_TIMER;
		}
		if (up_digin(9)==1)
		{
			if ((MS_TIMER-GpPosNoTimerA)>500)
			{
				GpPosNoFlagA=0;
			}
		}
		if ((up_digin(10)==0)&&(GpPosNoFlagB==0))
		{
			GpPosNoB += 1;
			if (up_digin(12)==0) //Hopper sensorB full
			{
				HopFullNoB +=1;
			}
			GpPosNoFlagB=1;
			GpPosNoTimerB=MS_TIMER;
		}
		if (up_digin(10)==1)
		{
			if ((MS_TIMER-GpPosNoTimerB)>500)
			{
				GpPosNoFlagB=0;
			}
		}
	}	//02 end						
	if ((up_digin(1)==0)||(up_digin(2)==0)) //V1 start
	{
		up_setout(10,1);
	}
	else
	{
		up_setout(10,0);
	}                     //v1 end
	if (DropTimeFlag1==0) 
	{
		DropTimer1=MS_TIMER;
	}  
	if (DropTimeFlag2==0) 
	{
		DropTimer2=MS_TIMER;
	}  
	if((MS_TIMER-DropTimer1)>DropTime3)
	{
		DropTime1=1;
	}
	if((MS_TIMER-DropTimer2)>DropTime3)
	{
		DropTime2=1;
	}								
	if (plcXP81In(7*32+18)==0) 
	{
		Step1=0;
		Step2=0;
		Step21=0;
		Step3=0;
		Step4=0;
		Step41=0;
		Step5=0;
		Step51=0;
		auto=0; //manual
		GpPos=0;
		GpPos1=0;
		GpPosFF=1;
		TroFullSenAF=1;
		TroFullSenBF=1;
		TroFullSenAFOff=1;
		TroFullSenBFOff=1;
		TroFullSenAF1=0; 
		TroFullSenBF1=0;
		TroFullSenAF2=0; 
		TroFullSenBF2=0; 
		TroLoadPosFlag=0;
		TroLoadPos=0;
		RtnPos=0;
		TroEmpSen=0;
		GpPosF=1; 
		GpPos1F=1;
		DropTime1=0; 
		DropTime2=0; 
		DropTimeFlag1=0; 
		DropTimeFlag2=0; 
		GpPosNoA=0; //02
		GpPosNoB=0; //02
		GpPosNoFlagA=0; //02
		GpPosNoFlagB=0; //02
		HopFullNoA=0; //02
		HopFullNoB=0; //02
		costate gate always_on
		{
			if (AutoFlag==1)//gates close
			{
				CoReset(&move);
				CoBegin(&move);
				CoReset(&gateauto);
				CoBegin(&gateauto);
				up_setout(1,0);  //2 to 1 v1
				waitfor(DelayMs(200)); 
				up_setout(2,1);
				GateAClose=1;  
				waitfor(up_digin(14)==1);//gate A close stop
				up_setout(2,0);
				up_setout(3,0);
				waitfor(DelayMs(200));
				up_setout(4,1);
				waitfor(up_digin(16)==1);//gate B close stop
				up_setout(4,0);
				up_setout(5,0);
				up_setout(8,0);
				GateAClose=0; 
				AutoFlag=0;
				Begin=0;//04
			}
		}
		AutoFlag1=1;
	}
	else
	{
		costate gateauto always_on
		{
			if (AutoFlag1==1)//gates close
			{
				CoReset(&gate);
				CoBegin(&gate);
				CoReset(&manual);
				CoBegin(&manual);
				up_setout(1,0);  //2 to 1 v1
				waitfor(DelayMs(200));
				up_setout(2,1);
				GateAClose=1; 
				waitfor(up_digin(14)==1);//gate A close stop
				up_setout(2,0);
				up_setout(3,0);
				waitfor(DelayMs(200));
				up_setout(4,1);
				waitfor(up_digin(16)==1);//gate B close stop
				up_setout(4,0);
				up_setout(5,0);
				up_setout(8,0);
				GateAClose=0; 
				AutoFlag1=0;
			}
		}
		auto=1; // auto
		AutoFlag=1;
	}
	if (plcXP81In(7*32+26)==1)
	{
		SafetySW = 1;
		AutoStart=0; 
		AutoStart1=0; 
		CoReset(&move);
		CoBegin(&move);
		CoReset(&gate);
		CoBegin(&gate);
		CoReset(&gateauto);
		CoBegin(&gateauto);
		up_setout(8,0);
		up_setout(12,1);
		Step1=0;
		Step2=0;
		Step21=0;
		Step3=0;
		Step4=0;
		Step41=0;
		Step5=0;
		Step51=0;
		GpPos=0;
		GpPos1=0;
		GpPosFF=1;
		TroFullSenAF=1;
		TroFullSenBF=1;
		TroFullSenAFOff=1;
		TroFullSenBFOff=1;
		TroFullSenAF1=0; 
		TroFullSenBF1=0; 
		TroFullSenAF2=0; 
		TroFullSenBF2=0; 
		TroLoadPosFlag=0;
		TroLoadPos=0;
		TroLoadPos1=0;
		RtnPos=0;
		TroEmpSen=0;
		GpPosF=1; 
		GpPos1F=1; 
		Hop1drop=0;
		Hop2drop=0;
		DropTime1=0; 
		DropTime2=0; 
		DropTimeFlag1=0;  
		DropTimeFlag2=0; 
		AutoFlag=1;
		AutoFlag1=1;
		up_setout(1,0);
		up_setout(2,0);
		up_setout(3,0);
		up_setout(4,0);
		up_setout(5,0);
		up_setout(7,0); //motor enable off
		GpPosNoA=0; //02
		GpPosNoB=0; //02
		GpPosNoFlagA=0; //02
		GpPosNoFlagB=0; //02
		HopFullNoA=0; //02
		HopFullNoB=0; //02
		Begin=0;//04
		HopFullTime=0; //11
	}
	if ((plcXP81In(7*32+16)==0)||(plcXP81In(7*32+17)==0))//02 
	{
		SafetySW = 1;
		AutoStart=0; 
		CoReset(&move);
		CoBegin(&move);
		CoReset(&gate);
		CoBegin(&gate);
		CoReset(&gateauto);
		CoBegin(&gateauto);
		up_setout(8,0);
		up_setout(12,1);
		Step1=0;
		Step2=0;
		Step21=0;
		Step3=0;
		Step4=0;
		Step41=0;
		Step5=0;
		Step51=0;
		GpPos=0;
		GpPos1=0;
		GpPosFF=1;
		TroFullSenAF=1;
		TroFullSenBF=1;
		TroFullSenAFOff=1;
		TroFullSenBFOff=1;
		TroFullSenAF1=0; 
		TroFullSenBF1=0; 
		TroFullSenAF2=0; 
		TroFullSenBF2=0; 
		TroLoadPosFlag=0;
		TroLoadPos=0;
		RtnPos=0;
		TroEmpSen=0;
		GpPosF=1; 
		GpPos1F=1; 
		DropTime1=0; 
		DropTime2=0; 
		DropTimeFlag1=0;  
		DropTimeFlag2=0;
		GpPosNoA=0; //02
		GpPosNoB=0; //02
		GpPosNoFlagA=0; //02
		GpPosNoFlagB=0; //02
		HopFullNoA=0; //02
		HopFullNoB=0; //02
		Begin=0;//04
		HopFullTime=0; //11
 	}
	if ((plcXP81In(7*32+25)==0)&&(SafetySW==1))
	{
		SafetySW = 0;
		up_setout(8,0);
		up_setout(12,0);
	}
	if ((plcXP81In(7*32+25)==0)&&(plcXP81In(7*32+18)==1)&&(plcXP81In(7*32+16)==1)&&(plcXP81In(7*32+17)==1))//02
	{
		SafetySW = 0;
		AutoStart=1; 
		AutoStart1=1; 
		AlarmTimer=SEC_TIMER; 
		up_setout(5,0);
		up_setout(8,0);
		up_setout(12,0);
		Step1=1;
	}


	if ((up_digin(9)==0)&&(GpPosF==1)) 
	{
		GpPos=1;
		GpPosF1=1;
		GpPosF2=1;
		GpPosF=0; 
	}
	costate
	{
		if (GpPosF2==1) 
		{
			waitfor(DelayMs(800));
			GpPosF2=0;
		}
		if ((up_digin(9)==1)&&(GpPosF1==1)&&(Step3==1)) 
		{
			waitfor(DelayMs(2000)); // from 500
			GpPosF=1; 
			GpPosF1=0; 
		}
	}
	if ((up_digin(10)==0)&&(GpPos1F==1))
	{
		GpPos1=1;
		GpPos1F1=1; 
		GpPos1F2=1; 
		GpPos1F=0; 
	}
	costate
	{
		if (GpPos1F2==1) 
		{
			waitfor(DelayMs(800));
			GpPos1F2=0; 
		}
		if ((up_digin(10)==1)&&(GpPos1F1==1)&&(Step3==1)) 
		{
			waitfor(DelayMs(2000));//from 500
			GpPos1F=1;
			GpPos1F1=0;
		}
	}
	if (up_digin(5)==1)
	{
		TroFullTimerA=MS_TIMER;
	}
	if((MS_TIMER-TroFullTimerA)>400)// from 400
	{
		TroFullSenA=1;
	}
	else
	{
		TroFullSenA=0;
	}
	if (up_digin(7)==1)
	{
		TroFullTimerB=MS_TIMER;
	}
	if((MS_TIMER-TroFullTimerB)>400)
	{
		TroFullSenB=1;
	}
	else
	{
		TroFullSenB=0;
	}
	if (GpPos==1)
	{
		if (up_digin(11)==1) //15
		{
			HopSensor1=1;
		}
	}
	else
	{
		HopSensor1=0;
	}
	if (GpPos1==1)
	{
		if (up_digin(12)==1) //15
		{
			HopSensor2=1;
		}
	}
	else
	{
		HopSensor2=0;
	}
	if ((auto==1)&&(SafetySW==0)&&(AutoStart==1))
	{
		if (((up_digin(6)==1)||(up_digin(8)==1))&&(TroEmpSen==0)&&(GateAClose==0)&&(GateBClose==0))
		{
			TroEmpSen=1;	//trolley empty
			ReturnToLoad();
		}
		if ((up_digin(4)==0)&&(RtnPos==0))//02 &&(GateAClose==0)&&(GateBClose==0))
		{
			RtnPos=1;
			ReturnToLoad();
		}
		TroLoadPosL3BL3=1;
		if ((up_digin(2)==0)&&(MoveForF==0))
		{
			TroLoadPos=1;
//			AlarmTimer=SEC_TIMER;// Reset Alarm Timer.
		}
		if (up_digin(2)==1)  //06
		{
			TroLoadPos=0;
		}
		if ((up_digin(1)==0)&&(MoveForF==0))
		{
			TroLoadPos1=1;
		}
		if (up_digin(1)==1)  //06
		{
			TroLoadPos1=0;
		}
	} 
//main steps
	if ((auto==1)&&(SafetySW==0)&&(AutoStart==1))  
	{
		costate move always_on
		{
			if (Step1==1) 
			{
				up_setout(7,1);
				if (Begin==0) //04 start
				{
					waitfor(DelayMs(1000));
					up_setout(9,0); //trolley moves Forward
					waitfor(DelayMs(200));
					up_setout(8,1);
					waitfor((DelayMs(5000))||(up_digin(4)==0)||(up_digin(9)==0)||(up_digin(10)==0));//05 from 3000
					waitfor(DelayMs(200));
					up_setout(8,0);
					Begin=1;
				} //04 end
				if ((TroEmpSen==1)||(RtnPos==1))	//trolley B empty sensor on
				{
					if ((TroLoadPos==0) && (TroLoadPosFlag==0))
					{
						waitfor(DelayMs(1000));
						up_setout(9,1); //trolley moves back
						waitfor(DelayMs(200));
						up_setout(8,1);
						MoveForF=0;
						TroLoadPosFlag=1;
					}
					if (TroLoadPos==1)
					{
						up_setout(8,0);
						waitfor(DelayMs(1000));
						Step1 = 0;
						Step2 = 1;
						TroFullSenAF=1;
						TroFullSenBF=1;
						TroFullSenAFOff=1;
						TroFullSenBFOff=1;
						LoadTimer=SEC_TIMER;
					}	
				}
				else
				{
						waitfor(DelayMs(1000));
						up_setout(9,0); //trolley moves Forward
						MoveForF=1;
						waitfor(DelayMs(200));
						up_setout(8,1);
						TroLoadPos=0;
						Step1 = 0;
						Step3 = 1;
						GpPos=0;
						GpPos1=0;
						GpPosFF=1;
				}
			}
			if (Step2==1)
			{
					if ((TroLoadPos==1)&&(TroFullSenB==0)&&(TroFullSenBF==1))
					{
						GpPos=0;
						GpPos1=0;
						GpPosFF=1;
						TroLoadPos=0; //06
						TroLoadPos1=0; 
						up_setout(5,1); 
						TroFullSenBF=0; 
//						AlarmTimer=SEC_TIMER;// Reset Alarm Timer.
					}
					if ((TroFullSenB==1)&&(TroFullSenBFOff==1))
					{
						up_setout(5,0);	
						waitfor(up_digin(3)==0); //03
						waitfor(DelayMs(5000)); //05
						TroLoadPos1=0;
						up_setout(9,1); //trolley moves Back
						MoveForF=0;
						waitfor(DelayMs(200));
						up_setout(8,1);
						waitfor(TroLoadPos1==1);
						waitfor(DelayMs(10));
						up_setout(8,0);
						waitfor(DelayMs(1000));
						TroFullSenAF1=1;
						TroFullSenAF2=1;
						TroFullSenBFOff=0;
					}
					if ((TroFullSenA==0)&&(TroFullSenAF==1)&&(TroFullSenAF1==1)&&(TroLoadPos1==1))
					{
						TroLoadPos1=0; 
						up_setout(5,1);  
						TroFullSenAF1=0;
					}
					if ((TroFullSenA==1)&&(TroFullSenAF==1)&&(TroFullSenAF2==1))
					{
						up_setout(5,0);
						waitfor(up_digin(3)==0); //03
						if (RtnPos==1) //06 start
						{
							waitfor(DelaySec(TroLoadMoveTime));
						}
						else
						{
							waitfor(DelayMs(5000)); 
						} //06 end
						TroFullSenAF=0;
				 		TroFullSenAF2=0;
						if ((GpPosNoA+GpPosNoB==HopFullNoA+HopFullNoB)&&(RtnPos==1))
						{
							waitfor(DelaySec(RestTime));
							HopFullTime += 1; //11
						}
						else //11 start
						{
							HopFullTime = 0;
						}
						if (HopFullTime > 1) 
						{
							waitfor(DelaySec(RestTime));
						} 				//11 end
						GpPosNoA=0;
						GpPosNoB=0;
						HopFullNoA=0;
						HopFullNoB=0;
						GoToStep3();
					}
			}				
			if (Step3==1)
			{
				if ((GpPos==0)&&(GpPos1==0)&&(GpPosFF==1)) 
				{
		//			up_setout(7,0); //motor enable off
		//			waitfor(DelayMs(100)); 
		//			up_setout(7,1); //motor enable on
					waitfor(DelayMs(100)); 
					up_setout(9,0); //trolley moves forward
					MoveForF=1;
					waitfor(DelayMs(100)); 
					up_setout(8,1);
					GpPosFF=0;
				}
				if ((GpPos==1)||(GpPos1==1)) 
				{
					waitfor(DelayMs(100)); 
					up_setout(8,0);
					waitfor(DelayMs(200)); //from 1000 
					Step3=0;
					Step4=1;
					Step5=1;
					TroLoadPosFlag=0;
					TroLoadPos=0;
				}
			}
			if ((Step4==1)&&(GpPos==1))
			{
				if (HopSensor1==1) //trolley position on and hop1 sensor off
				{
					GateAClose=1; 
					up_setout(1,1);
					Hop1drop=1;
					DropTimeFlag1=1; 
				}
				else
				{
					Step4=0;
					Step41=1;
				}
				if (Hop1drop==1)
				{
					waitfor((up_digin(13)==0)||(DropTime1==1));
					up_setout(1,0);   //2 to 1 V1
					waitfor(DelayMs(DropTime));
					up_setout(2,1);
					waitfor(up_digin(14)==1);
					up_setout(2,0);
					waitfor(DelayMs(400));
					GateAClose=0; 
					Step4=0;
					Step41=1;
				}
			}
			if ((Step5==1)&&(GpPos1==1))
			{
				if (HopSensor2==1) //trolley position on and hop2 sensor off
				{
					GateBClose=1; 
					up_setout(3,1);
					Hop2drop=1;
					DropTimeFlag2=1;
				} 
				else
				{
					Step5=0;
					Step51=1;
				}
				if (Hop2drop==1)
				{
					waitfor((up_digin(15)==0)||(DropTime2==1));
					up_setout(3,0);
					waitfor(DelayMs(DropTime));
					up_setout(4,1);
					waitfor(up_digin(16)==1);
					up_setout(4,0);
					waitfor(DelayMs(400));
					GateBClose=0; 
					Step5=0;
					Step51=1;
				}
			}
			if ((Step41==1)||(Step51==1))
			{
				Step3=0;
				if ((Hop1drop==1)||(Hop2drop==1))
				{
					waitfor(DelayMs(300));
					Hop1drop=0;
					Hop2drop=0;
				}
				GpPos=0;
				GpPos1=0;
				GpPosFF=1;
				Step3=1;
				Step4=0;
				Step41=0;
				Step5=0;
				Step51=0;
				DropTime1=0; 
				DropTime2=0; 
				DropTimeFlag1=0;  
				DropTimeFlag2=0;
			}
		} 
	}	
	else if (auto==0)
	{
		costate manual always_on
		{
		if ((plcXP81In(7*32+26)==0)&&(SafetySW==0)) 
		{
//Manual move trolley
			up_setout(7,1);
			if (plcXP81In(7*32+20)==0)
			{
				up_setout(9,0); //trolley moves forward
				MoveForF=1;
				waitfor(DelayMs(200));
				up_setout(8,1);
			}
			if (plcXP81In(7*32+19)==0)
			{
				up_setout(9,1); //trolley moves backward
				MoveForF=0;
				waitfor(DelayMs(200));
				up_setout(8,1);
			}
			if ((plcXP81In(7*32+19)==1)&&(plcXP81In(7*32+20)==1))
			{
				up_setout(8,0);
			}
//Open gate A
			if (plcXP81In(7*32+21)==0)//&&(up_digin(13)==0))//gate A open
			{
				up_setout(2,0);  
				waitfor(DelayMs(500)); 
				up_setout(1,1);
				DropTimeFlag1=1; 
			}
			if ((up_digin(13)==0)||(DropTime1==1))//gate A open stop
			{
				up_setout(1,0);
				DropTimeFlag1=0; 
			}
//Close gate A
			if (plcXP81In(7*32+22)==0)//&&(up_digin(14)==1))//gate A close
			{
				up_setout(1,0); 
				waitfor(DelayMs(500)); 
				up_setout(2,1);
			}
			if (up_digin(14)==1)//gate A close stop
			{
				up_setout(2,0);
			}
//Open gate B
			if (plcXP81In(7*32+23)==0)//&&(up_digin(15)==0))//gate B open
			{
				up_setout(4,0); 
				waitfor(DelayMs(500)); 
				up_setout(3,1);
				DropTimeFlag2=1; //14
			}
			if ((up_digin(15)==0)||(DropTime2==1))//gate B open stop 14
			{
				up_setout(3,0);
				DropTimeFlag2=0; //14 
			}
//Close gate B
			if (plcXP81In(7*32+24)==0)//&&(up_digin(16)==1))//gate B close
			{
				up_setout(3,0); 
				waitfor(DelayMs(500)); 
				up_setout(4,1);
			}
			if (up_digin(16)==1)//gate B close stop
			{
				up_setout(4,0);
			}
//top bin V1
			if ((plcXP81In(6*32+17)==0)&&((up_digin(1)==0)||(up_digin(2)==0))&&(TroFullSenA==0)) //16
			{
				up_setout(5,1);
			}
			else
			{
				up_setout(5,0);
			}
		}
		else
		{
			up_setout(7,0); //motor enable off
		}
		}
	}		
}
#funcchain _srtk_lowtask fivekey_lowtask
int fivekey_lowtask()
{	
	segchain _srtk_hightask { lc_kget(1); }
	
	fk_monitorkeypad();

//Alarm control 
	if (AutoStart1==0)
	{
		AlarmTimer=SEC_TIMER;
	}								
	if ((((SEC_TIMER - AlarmTimer)>AlarmTime)||(plcXP81In(7*32+16)==0)||(plcXP81In(7*32+17)==0))&&(auto==1))
	{
		up_setout(14,1);
	}
	else
	{
		up_setout(14,0);
	}
}
int ReturnToLoad()
{
	CoReset(&move);
	CoBegin(&move);
	up_setout(5,0);
	up_setout(8,0);
	TroLoadPosFlag=0;
	TroLoadPos=0; 
	Step1=1;
	Step2=0;
	Step21=0;
	Step3=0;
	Step4=0;
	Step41=0;
	Step5=0;
	Step51=0;
} 
int GoToStep3()
{
	up_setout(5,0);
	Step1=0;
	Step2=0;
	Step3=1;
	GpPos=0;
	GpPos1=0;
	GpPosFF=1;
	TroLoadPosFlag=0;
	TroLoadPos=0; 
	TroLoadPos1=0; 
	RtnPos=0;
	TroEmpSen=0;
	GpPosF=1; 
	GpPos1F=1;
	AlarmTimer=SEC_TIMER;// Reset Alarm Timer. 08
	AlarmTimerF=1; //08 
}
//end of program 1014 lines using


