

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"						// gsf standard #defines
#include "Debug.h"

#include "HardwareProfile.h"

#include "LEDs.h"				// LED display handler function definition, used for stall recovery states
#include "SmartTrak.h"
#include "ADC10Read.h"
#include "BMS.h"



int Avg_Adc_Value(UINT8 bADCChannel)
{

    struct ADBUF_TYPE *adcreg = (struct ADBUF_TYPE *)&ADC1BUF0;
    volatile int *bufp = &adcreg[bADCChannel].buf;

        int  rval = -1;
	if (bADCChannel >= 0) {
		do {
			rval = *bufp;
		} while (rval != *bufp);
	}

   nadc[bADCChannel] =  (padc[bADCChannel]+ rval)/2;
   padc[bADCChannel] = nadc[bADCChannel] ;

    return nadc[bADCChannel];
}

int AVG_SAMPLES_ADC(UINT8 bADCChannel)
{
    int Itm=0; int Vret=0;
    for(Itm=0;Itm<50;Itm++)
    {
        Vret = Vret+Avg_Adc_Value(bADCChannel);
    }
    Vret = Vret/50;
    return Vret;
}
void Calculate_Avg_ADC()
{
    float temp1,temp2;
    M1C =AVG_SAMPLES_ADC(MOTOR1_CUR_CHANNEL) *CNV_ADC2V * 4.35;

    M2C = AVG_SAMPLES_ADC(MOTOR2_CUR_CHANNEL)*CNV_ADC2V * 4.35;

   if((IsCommandComplete(AXIS_AZIMUTH) IS_FALSE)||(M1C <0.1))
    {
        BMS_V = AVG_SAMPLES_ADC(BAT_VOL_CHANNEL)*CNV_ADC2V *(10000+700)/700;
        BMS_V = BMS_V - 0.2;
    }

    temp1=AVG_SAMPLES_ADC(TEMP_CHANNEL1 )*CNV_ADC2V;
    temp2=AVG_SAMPLES_ADC(TEMP_CHANNEL2 )*CNV_ADC2V;
    BTemp = (temp1 - temp2) *100;
}





void Intial_calc_adc()
{
    int i;
    for(i=0;i<=NO_SAMPLES;i++)
         Calculate_Avg_ADC();
}
/*
BOOL Cal_Avg_Battery_Voltage(UINT8 bADCChannel)
{
    int vadc;
    vadc = Avg_Adc_Value(bADCChannel) ;
    AZ_V = (float)nadc[bADCChannel]*CNV_ADC2V;
    AZ = VOL_TO_DEG(AZ_V);

    return TRUE;
}

void AVG_AZ_Angle()
{
	static BYTE newSampleIndex = 0;
	BYTE i;
	float fSumOfXSamples = 0.0;

	BOOL bRetVal = TRUE;

	// copy sample into averaging array az voltages
	AZ_Sample[newSampleIndex] = AZ_V;

	// sum values in averaging array
	for (i = 0; i < NO_SAMPLES; i++)
	{
		fSumOfXSamples += AZ_Sample[i];
	}

	// divide by sample count and save in global structure
	AZ_AVG_V = fSumOfXSamples / NO_SAMPLES;
        AVG_AZ = VOL_TO_DEG(AZ_AVG_V);
	// bump array index
	++newSampleIndex;
	if(newSampleIndex IS NO_SAMPLES)
	{
		newSampleIndex = 0;
	}

	return bRetVal;


}

float Cal_Avg_String_Voltage()
{
    float  VString;
    float VAvg = 0;

    VAvg = Avg_Adc_Value(VStringChannel);

    VString =  1.104567*VAvg - 6.461862; //3.282547508*Vsample - 1.760981505;// (Vadc * (300K + 1K)) = 1K * VString
    if(VString < 0)
        VString = 0;
    return VString;
}
float Cal_Avg_temp(UINT8 bADCChannel)
{
    float  temp;
    int vadc;
    vadc = Avg_Adc_Value(bADCChannel) ;

    temp = ((8.194-sqrtf((-8.194 * -8.194)+(4*0.00262*(1324-((vadc/1023.0)*3.3)*1000))))/(2 * -0.00262))+30;
    return temp;
}
BOOL FormatAverageAZ_POT(char *ptrOutputStr)
{

	int nStatus;

	strcpy(ptrOutputStr, "\tPOT Vol: ");
	strcat(ptrOutputStr, ftoa2(AZ_V, &nStatus));

	strcat(ptrOutputStr, "\tAZ: ");
	strcat(ptrOutputStr, ftoa2(AZ, &nStatus));

        strcat(ptrOutputStr, "\tAVG_AZ: ");
	strcat(ptrOutputStr, ftoa2(AVG_AZ, &nStatus));

	return TRUE;

}
*/