#include "simExtK3.h"
#include "scriptFunctionData.h"
#include <iostream>
#include "simLib.h"
#include <math.h>
#include <sstream>
#include <iomanip>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

LIBRARY simLib;

struct sK3
{
    int k3BaseHandle;
    int handle;
    int scriptHandle;
    int wheelMotorHandles[2];
    int colorSensorHandles[2];
    int irSensorHandles[9];
    int usSensorHandles[5];
    int armMotorHandles[6];
    int fingerMotorHandles[3];
    int gripperDistanceSensorHandles[2];
    int gripperColorSensorHandles[2];

    float maxVelocity;
    float maxAcceleration;
    float maxArmAcceleration;
    float targetVelocities[2];
    float currentVelocities[2];
    float cumulativeMotorAngles[2];
    float previousMotorAngles[2];
    float targetArmPosition;
    float currentArmPosition;
    float currentArmVelocity;
    float targetGripperGap;
    float currentGripperGap;
};

std::vector<sK3> allK3s;
int nextK3Handle=0;

int getK3IndexFromHandle(int k3Handle)
{
    for (unsigned int i=0;i<allK3s.size();i++)
    {
        if (allK3s[i].handle==k3Handle)
            return(i);
    }
    return(-1);
}

int getK3IndexFromScriptHandle(int h)
{
    for (unsigned int i=0;i<allK3s.size();i++)
    {
        if (allK3s[i].scriptHandle==h)
            return(i);
    }
    return(-1);
}

// --------------------------------------------------------------------------------------
// simExtK3_create
// --------------------------------------------------------------------------------------
#define LUA_CREATE_COMMAND "simK3.create"

const int inArgs_CREATE[]={
    8,
    sim_script_arg_int32|sim_script_arg_table,2, // wheel motor handles
    sim_script_arg_int32|sim_script_arg_table,2, // color sensor handles
    sim_script_arg_int32|sim_script_arg_table,9, // IR sensor handles
    sim_script_arg_int32|sim_script_arg_table,5, // US sensor handles
    sim_script_arg_int32|sim_script_arg_table,6, // arm motor handles
    sim_script_arg_int32|sim_script_arg_table,3, // finger motor handles
    sim_script_arg_int32|sim_script_arg_table,2, // gripper distance sensor handles
    sim_script_arg_int32|sim_script_arg_table,2, // gripper color sensor handles
};

void LUA_CREATE_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    int handle=-1;
    if (D.readDataFromStack(p->stackID,inArgs_CREATE,inArgs_CREATE[0],LUA_CREATE_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        sK3 k3;
        k3.k3BaseHandle=p->objectID;
        handle=nextK3Handle++;
        k3.handle=handle;
        k3.scriptHandle=p->scriptID;
        for (unsigned int i=0;i<2;i++)
            k3.wheelMotorHandles[i]=inData->at(0).int32Data[i];
        for (unsigned int i=0;i<2;i++)
            k3.colorSensorHandles[i]=inData->at(1).int32Data[i];
        for (unsigned int i=0;i<9;i++)
            k3.irSensorHandles[i]=inData->at(2).int32Data[i];
        for (unsigned int i=0;i<5;i++)
            k3.usSensorHandles[i]=inData->at(3).int32Data[i];
        for (unsigned int i=0;i<6;i++)
            k3.armMotorHandles[i]=inData->at(4).int32Data[i];
        for (unsigned int i=0;i<3;i++)
            k3.fingerMotorHandles[i]=inData->at(5).int32Data[i];
        for (unsigned int i=0;i<2;i++)
            k3.gripperDistanceSensorHandles[i]=inData->at(6).int32Data[i];
        for (unsigned int i=0;i<2;i++)
            k3.gripperColorSensorHandles[i]=inData->at(7).int32Data[i];

        k3.maxVelocity=6.283f;
        k3.maxAcceleration=25.0f;
        k3.maxArmAcceleration=0.5f;
        k3.targetVelocities[0]=0.0f;
        k3.targetVelocities[1]=0.0f;
        k3.currentVelocities[0]=0.0f;
        k3.currentVelocities[1]=0.0f;
        k3.cumulativeMotorAngles[0]=0.0f;
        k3.cumulativeMotorAngles[1]=0.0f;
        k3.previousMotorAngles[0]=0.0f;
        k3.previousMotorAngles[1]=0.0f;
        k3.targetArmPosition=0.0f;
        k3.currentArmPosition=0.0f;
        k3.currentArmVelocity=0.0f;
        k3.targetGripperGap=0.055f;
        k3.currentGripperGap=0.055f;

        allK3s.push_back(k3);
    }
    D.pushOutData(CScriptFunctionDataItem(handle));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// simExtK3_destroy
// --------------------------------------------------------------------------------------
#define LUA_DESTROY_COMMAND "simK3.destroy"

const int inArgs_DESTROY[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_DESTROY_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_DESTROY,inArgs_DESTROY[0],LUA_DESTROY_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=inData->at(0).int32Data[0];
        int k3Index=getK3IndexFromHandle(handle);
        if (k3Index>=0)
        {
            allK3s.erase(allK3s.begin()+k3Index);
            success=true;
        }
        else
            simSetLastError(LUA_DESTROY_COMMAND,"Invalid Khepera3 handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(success));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// simExtK3_getInfrared
// --------------------------------------------------------------------------------------
#define LUA_GETINFRARED_COMMAND "simK3.getInfrared"

const int inArgs_GETINFRARED[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETINFRARED_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    float distance=100.0f; // no detection
    if (D.readDataFromStack(p->stackID,inArgs_GETINFRARED,inArgs_GETINFRARED[0],LUA_GETINFRARED_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int sensorIndex=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if ( (sensorIndex>=0)&&(sensorIndex<9) )
            {
                float ptAndDist[4];
                if (((simGetExplicitHandling(allK3s[k3Index].irSensorHandles[sensorIndex])&1)==0)&&(simReadProximitySensor(allK3s[k3Index].irSensorHandles[sensorIndex],ptAndDist,nullptr,nullptr)>0))
                    distance=ptAndDist[3];
                success=true;
            }
            else
                simSetLastError(LUA_GETINFRARED_COMMAND,"Invalid index.");
        }
        else
            simSetLastError(LUA_GETINFRARED_COMMAND,"Invalid Khepera3 handle.");
    }
    if (success)
        D.pushOutData(CScriptFunctionDataItem(distance));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_getUltrasonic
// --------------------------------------------------------------------------------------
#define LUA_GETULTRASONIC_COMMAND "simK3.getUltrasonic"

const int inArgs_GETULTRASONIC[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETULTRASONIC_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    float distance=100.0f; // no detection
    if (D.readDataFromStack(p->stackID,inArgs_GETULTRASONIC,inArgs_GETULTRASONIC[0],LUA_GETULTRASONIC_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int sensorIndex=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if ( (sensorIndex>=0)&&(sensorIndex<5) )
            {
                float ptAndDist[4];
                if (((simGetExplicitHandling(allK3s[k3Index].usSensorHandles[sensorIndex])&1)==0)&&(simReadProximitySensor(allK3s[k3Index].usSensorHandles[sensorIndex],ptAndDist,nullptr,nullptr)>0))
                    distance=ptAndDist[3];
                success=true;
            }
            else
                simSetLastError(LUA_GETULTRASONIC_COMMAND,"Invalid index.");
        }
        else
            simSetLastError(LUA_GETULTRASONIC_COMMAND,"Invalid Khepera3 handle.");
    }
    if (success)
        D.pushOutData(CScriptFunctionDataItem(distance));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_getLineSensor
// --------------------------------------------------------------------------------------
#define LUA_GETLINESENSOR_COMMAND "simK3.getLineSensor"

const int inArgs_GETLINESENSOR[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETLINESENSOR_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    float intensity=0.0f; // no detection
    if (D.readDataFromStack(p->stackID,inArgs_GETLINESENSOR,inArgs_GETLINESENSOR[0],LUA_GETLINESENSOR_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int sensorIndex=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if ( (sensorIndex>=0)&&(sensorIndex<2) )
            {
                float* auxValues=nullptr;
                int* auxValuesCount=nullptr;
                if (simReadVisionSensor(allK3s[k3Index].colorSensorHandles[sensorIndex],&auxValues,&auxValuesCount)>=0)
                {
                    if ((auxValuesCount[0]>0)||(auxValuesCount[1]>=15))
                        intensity=auxValues[10];
                    simReleaseBuffer((char*)auxValues);
                    simReleaseBuffer((char*)auxValuesCount);
                }
                success=true;
            }
            else
                simSetLastError(LUA_GETLINESENSOR_COMMAND,"Invalid index.");
        }
        else
            simSetLastError(LUA_GETLINESENSOR_COMMAND,"Invalid Khepera3 handle.");
    }
    if (success)
        D.pushOutData(CScriptFunctionDataItem(intensity));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_getEncoder
// --------------------------------------------------------------------------------------
#define LUA_GETENCODER_COMMAND "simK3.getEncoder"

const int inArgs_GETENCODER[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETENCODER_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    int encoder=0;
    if (D.readDataFromStack(p->stackID,inArgs_GETENCODER,inArgs_GETENCODER[0],LUA_GETENCODER_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int encoderIndex=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if ( (encoderIndex>=0)&&(encoderIndex<2) )
            {
                encoder=int(float(2764)*allK3s[k3Index].cumulativeMotorAngles[encoderIndex]/(2.0f*3.1415f)); // 2764 are the impulses per turn
                success=true;
            }
            else
                simSetLastError(LUA_GETENCODER_COMMAND,"Invalid index.");
        }
        else
            simSetLastError(LUA_GETENCODER_COMMAND,"Invalid Khepera3 handle.");
    }
    if (success)
        D.pushOutData(CScriptFunctionDataItem(encoder));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_getGripperProxSensor
// --------------------------------------------------------------------------------------
#define LUA_GETGRIPPERPROXSENSOR_COMMAND "simK3.getGripperProxSensor"

const int inArgs_GETGRIPPERPROXSENSOR[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_GETGRIPPERPROXSENSOR_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    float distance=100.0f;
    if (D.readDataFromStack(p->stackID,inArgs_GETGRIPPERPROXSENSOR,inArgs_GETGRIPPERPROXSENSOR[0],LUA_GETGRIPPERPROXSENSOR_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int sensorIndex=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if ( (sensorIndex>=0)&&(sensorIndex<2) )
            {
                float ptAndDist[4];
                if (((simGetExplicitHandling(allK3s[k3Index].gripperDistanceSensorHandles[sensorIndex])&1)==0)&&(simReadProximitySensor(allK3s[k3Index].gripperDistanceSensorHandles[sensorIndex],ptAndDist,nullptr,nullptr)>0))
                    distance=ptAndDist[3];
                success=true;
            }
            else
                simSetLastError(LUA_GETGRIPPERPROXSENSOR_COMMAND,"Invalid index.");
        }
        else
            simSetLastError(LUA_GETGRIPPERPROXSENSOR_COMMAND,"Invalid Khepera3 handle.");
    }
    if (success)
        D.pushOutData(CScriptFunctionDataItem(distance));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_setArmPosition
// --------------------------------------------------------------------------------------
#define LUA_SETARMPOSITION_COMMAND "simK3.setArmPosition"

const int inArgs_SETARMPOSITION[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETARMPOSITION_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_SETARMPOSITION,inArgs_SETARMPOSITION[0],LUA_SETARMPOSITION_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int position=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if (position>900)
                position=900;
            if (position<300)
                position=300;
            allK3s[k3Index].targetArmPosition=(1.0f-float(position-300)/600.0f)*195.0f*3.1415f/180.0f;
            success=true;
        }
        else
            simSetLastError(LUA_SETARMPOSITION_COMMAND,"Invalid Khepera3 handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(success));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_setGripperGap
// --------------------------------------------------------------------------------------
#define LUA_SETGRIPPERGAP_COMMAND "simK3.setGripperGap"

const int inArgs_SETGRIPPERGAP[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETGRIPPERGAP_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_SETGRIPPERGAP,inArgs_SETGRIPPERGAP[0],LUA_SETGRIPPERGAP_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int gap=inData->at(1).int32Data[0];
        if (k3Index!=-1)
        {
            if (gap>170)
                gap=170;
            if (gap<0)
                gap=0;
            allK3s[k3Index].targetGripperGap=0.055f*float(gap)/170.0f;
            success=true;
        }
        else
            simSetLastError(LUA_SETGRIPPERGAP_COMMAND,"Invalid Khepera3 handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(success));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_setVelocity
// --------------------------------------------------------------------------------------
#define LUA_SETVELOCITY_COMMAND "simK3.setVelocity"

const int inArgs_SETVELOCITY[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_float,0,
    sim_script_arg_float,0,
};

void LUA_SETVELOCITY_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_SETVELOCITY,inArgs_SETVELOCITY[0],LUA_SETVELOCITY_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        float leftVel=inData->at(1).floatData[0];
        float rightVel=inData->at(2).floatData[0];
        if (k3Index!=-1)
        {
            if (leftVel>allK3s[k3Index].maxVelocity)
                leftVel=allK3s[k3Index].maxVelocity;
            if (leftVel<-allK3s[k3Index].maxVelocity)
                leftVel=-allK3s[k3Index].maxVelocity;
            if (rightVel>allK3s[k3Index].maxVelocity)
                rightVel=allK3s[k3Index].maxVelocity;
            if (rightVel<-allK3s[k3Index].maxVelocity)
                rightVel=-allK3s[k3Index].maxVelocity;
            allK3s[k3Index].targetVelocities[0]=leftVel;
            allK3s[k3Index].targetVelocities[1]=rightVel;
            success=true;
        }
        else
            simSetLastError(LUA_SETVELOCITY_COMMAND,"Invalid Khepera3 handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(success));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtK3_setEncoders
// --------------------------------------------------------------------------------------
#define LUA_SETENCODERS_COMMAND "simK3.setEncoders"

const int inArgs_SETENCODERS[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_SETENCODERS_CALLBACK(SScriptCallBack* p)
{

    CScriptFunctionData D;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_SETENCODERS,inArgs_SETENCODERS[0],LUA_SETENCODERS_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int k3Index=getK3IndexFromHandle(inData->at(0).int32Data[0]);
        int leftEncoder=inData->at(1).int32Data[0];
        int rightEncoder=inData->at(2).int32Data[0];
        if (k3Index!=-1)
        {
            allK3s[k3Index].cumulativeMotorAngles[0]=float(leftEncoder)*(2.0f*3.1415f)/float(2764); // 2764 are the impulses per turn
            allK3s[k3Index].cumulativeMotorAngles[1]=float(rightEncoder)*(2.0f*3.1415f)/float(2764);
            success=true;
        }
        else
            simSetLastError(LUA_SETENCODERS_COMMAND,"Invalid Khepera3 handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(success));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of CoppeliaSim.
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(nullptr,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif

    simLib=loadSimLibrary(temp.c_str());
    if (simLib==nullptr)
    {
        printf("simExtK3: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        printf("simExtK3: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
    }

    simRegisterScriptVariable("simK3","require('simExtK3')",0);

    // Register the new functions:
    simRegisterScriptCallbackFunction(strConCat(LUA_CREATE_COMMAND,"@","K3"),strConCat("number k3Handle=",LUA_CREATE_COMMAND,"(table_2 wheelMotorHandles,table_2 colorSensorHandles,table_9 IrSensorHandles,table_5 usSensorHandles,table_6 armMotorHandles,table_3 fingerMotorHandles,table_2 gripperDistSensHandles,table_2 gripperColSensHandles)"),LUA_CREATE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_DESTROY_COMMAND,"@","K3"),strConCat("boolean result=",LUA_DESTROY_COMMAND,"(number k3Handle)"),LUA_DESTROY_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GETINFRARED_COMMAND,"@","K3"),strConCat("number distance=",LUA_GETINFRARED_COMMAND,"(number k3Handle,number index)"),LUA_GETINFRARED_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GETULTRASONIC_COMMAND,"@","K3"),strConCat("number distance=",LUA_GETULTRASONIC_COMMAND,"(number k3Handle,number index)"),LUA_GETULTRASONIC_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GETLINESENSOR_COMMAND,"@","K3"),strConCat("number intensity=",LUA_GETLINESENSOR_COMMAND,"(number k3Handle,number index)"),LUA_GETLINESENSOR_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GETENCODER_COMMAND,"@","K3"),strConCat("number encoderValue=",LUA_GETENCODER_COMMAND,"(number k3Handle,number index)"),LUA_GETENCODER_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GETGRIPPERPROXSENSOR_COMMAND,"@","K3"),strConCat("number distance=",LUA_GETGRIPPERPROXSENSOR_COMMAND,"(number k3Handle,number index)"),LUA_GETGRIPPERPROXSENSOR_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_SETARMPOSITION_COMMAND,"@","K3"),strConCat("boolean result=",LUA_SETARMPOSITION_COMMAND,"(number k3Handle,number position)"),LUA_SETARMPOSITION_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_SETGRIPPERGAP_COMMAND,"@","K3"),strConCat("boolean result=",LUA_SETGRIPPERGAP_COMMAND,"(number k3Handle,number gap)"),LUA_SETGRIPPERGAP_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_SETVELOCITY_COMMAND,"@","K3"),strConCat("boolean result=",LUA_SETVELOCITY_COMMAND,"(number k3Handle,number leftVelocity,number rightVelocity)"),LUA_SETVELOCITY_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_SETENCODERS_COMMAND,"@","K3"),strConCat("boolean result=",LUA_SETENCODERS_COMMAND,"(number k3Handle,number leftEncoderValue,number rightEncoderValue)"),LUA_SETENCODERS_CALLBACK);

    // Following for backward compatibility:
    simRegisterScriptVariable("simExtK3_create",LUA_CREATE_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_create","@","K3"),strConCat("Please use the ",LUA_CREATE_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_destroy",LUA_DESTROY_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_destroy","@","K3"),strConCat("Please use the ",LUA_DESTROY_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_getInfrared",LUA_GETINFRARED_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_getInfrared","@","K3"),strConCat("Please use the ",LUA_GETINFRARED_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_getUltrasonic",LUA_GETULTRASONIC_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_getUltrasonic","@","K3"),strConCat("Please use the ",LUA_GETULTRASONIC_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_getLineSensor",LUA_GETLINESENSOR_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_getLineSensor","@","K3"),strConCat("Please use the ",LUA_GETLINESENSOR_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_getEncoder",LUA_GETENCODER_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_getEncoder","@","K3"),strConCat("Please use the ",LUA_GETENCODER_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_getGripperProxSensor",LUA_GETGRIPPERPROXSENSOR_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_getGripperProxSensor","@","K3"),strConCat("Please use the ",LUA_GETGRIPPERPROXSENSOR_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_setArmPosition",LUA_SETARMPOSITION_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_setArmPosition","@","K3"),strConCat("Please use the ",LUA_SETARMPOSITION_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_setGripperGap",LUA_SETGRIPPERGAP_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_setGripperGap","@","K3"),strConCat("Please use the ",LUA_SETGRIPPERGAP_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_setVelocity",LUA_SETVELOCITY_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_setVelocity","@","K3"),strConCat("Please use the ",LUA_SETVELOCITY_COMMAND," notation instead"),0);
    simRegisterScriptVariable("simExtK3_setEncoders",LUA_SETENCODERS_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtK3_setEncoders","@","K3"),strConCat("Please use the ",LUA_SETENCODERS_COMMAND," notation instead"),0);

    return(9); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
    // version 6 is for CoppeliaSim versions after CoppeliaSim 3.2.0 (completely rewritten)
    // version 7 is for CoppeliaSim versions after CoppeliaSim 3.3.0 (using stacks to exchange data with scripts)
    // version 8 is for CoppeliaSim versions after CoppeliaSim 3.4.0 (new API notation)
    // version 9 is for CoppeliaSim versions after CoppeliaSim 4.2.0
}

SIM_DLLEXPORT void simEnd()
{ // This is called just once, at the end of CoppeliaSim
    unloadSimLibrary(simLib); // release the library
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    void* retVal=nullptr;

    if ( (message==sim_message_eventcallback_simulationactuation)&&(auxiliaryData[0]==0) )
    { // the main script's actuation section is about to be executed
        float dt=simGetSimulationTimeStep();
        for (unsigned int k3Index=0;k3Index<allK3s.size();k3Index++)
        {
            // 1. Run the robot control:
            for (int i=0;i<2;i++)
            {
                if (allK3s[k3Index].targetVelocities[i]>allK3s[k3Index].currentVelocities[i])
                {
                    allK3s[k3Index].currentVelocities[i]=allK3s[k3Index].currentVelocities[i]+allK3s[k3Index].maxAcceleration*dt;
                    if (allK3s[k3Index].currentVelocities[i]>allK3s[k3Index].targetVelocities[i])
                        allK3s[k3Index].currentVelocities[i]=allK3s[k3Index].targetVelocities[i];
                }
                else
                {
                    allK3s[k3Index].currentVelocities[i]=allK3s[k3Index].currentVelocities[i]-allK3s[k3Index].maxAcceleration*dt;
                    if (allK3s[k3Index].currentVelocities[i]<allK3s[k3Index].targetVelocities[i])
                        allK3s[k3Index].currentVelocities[i]=allK3s[k3Index].targetVelocities[i];
                }
                simSetJointTargetVelocity(allK3s[k3Index].wheelMotorHandles[i],allK3s[k3Index].currentVelocities[i]);
                float jp;
                simGetJointPosition(allK3s[k3Index].wheelMotorHandles[i],&jp);
                float dp=jp-allK3s[k3Index].previousMotorAngles[i];
                if (fabs(dp)>3.1415f)
                    dp-=(2.0f*3.1415f*fabs(dp)/dp);
                allK3s[k3Index].cumulativeMotorAngles[i]+=dp; // corrected on 5/3/2012 thanks to Felix Fischer
                allK3s[k3Index].previousMotorAngles[i]=jp;
            }


            float adp=allK3s[k3Index].targetArmPosition-allK3s[k3Index].currentArmPosition;
            if (adp!=0.0f)
            {
                if (adp*allK3s[k3Index].currentArmVelocity>=0.0f)
                {
                    float decelToZeroTime=(fabs(allK3s[k3Index].currentArmVelocity)+allK3s[k3Index].maxArmAcceleration*dt*1.0f)/allK3s[k3Index].maxArmAcceleration;
                    float distToZero=0.5f*allK3s[k3Index].maxArmAcceleration*decelToZeroTime*decelToZeroTime;
                    float dir=1.0f;
                    if (allK3s[k3Index].currentArmVelocity!=0.0f)
                        dir=allK3s[k3Index].currentArmVelocity/fabs(allK3s[k3Index].currentArmVelocity);
                    else
                        dir=adp/fabs(adp);
                    if (fabs(adp)>distToZero)
                        allK3s[k3Index].currentArmVelocity+=dir*allK3s[k3Index].maxArmAcceleration*dt;
                    else
                        allK3s[k3Index].currentArmVelocity-=dir*allK3s[k3Index].maxArmAcceleration*dt;
                }
                else
                    allK3s[k3Index].currentArmVelocity*=(1-allK3s[k3Index].maxArmAcceleration*dt/fabs(allK3s[k3Index].currentArmVelocity));
            }
            else
            {
                if (allK3s[k3Index].currentArmVelocity!=0.0f)
                {
                    float dv=-allK3s[k3Index].currentArmVelocity*allK3s[k3Index].maxArmAcceleration*dt/fabs(allK3s[k3Index].currentArmVelocity);
                    if ((allK3s[k3Index].currentArmVelocity+dv)*allK3s[k3Index].currentArmVelocity<0.0f)
                        allK3s[k3Index].currentArmVelocity=0.0f;
                    else
                        allK3s[k3Index].currentArmVelocity+=dv;
                }
            }

            allK3s[k3Index].currentArmPosition+=allK3s[k3Index].currentArmVelocity*dt;

            simSetJointTargetPosition(allK3s[k3Index].armMotorHandles[0],allK3s[k3Index].currentArmPosition);
            simSetJointTargetPosition(allK3s[k3Index].armMotorHandles[1],-allK3s[k3Index].currentArmPosition);
            simSetJointTargetPosition(allK3s[k3Index].armMotorHandles[2],allK3s[k3Index].currentArmPosition);
            simSetJointTargetPosition(allK3s[k3Index].armMotorHandles[3],allK3s[k3Index].currentArmPosition);
            simSetJointTargetPosition(allK3s[k3Index].armMotorHandles[4],allK3s[k3Index].currentArmPosition);
            simSetJointTargetPosition(allK3s[k3Index].armMotorHandles[5],allK3s[k3Index].currentArmPosition);

            float jp;
            simGetJointPosition(allK3s[k3Index].fingerMotorHandles[0],&jp);
            allK3s[k3Index].currentGripperGap=(jp)+0.04f;
            float dp=allK3s[k3Index].targetGripperGap-allK3s[k3Index].currentGripperGap;
            float velToRegulate=0.0f;
            if (fabs(dp)<0.005f)
            {
                if (dp!=0.0f)
                    velToRegulate=(fabs(dp)/0.005f)*0.045f*dp/fabs(dp);
            }
            else
                velToRegulate=0.045f*dp/fabs(dp);
            simSetJointTargetVelocity(allK3s[k3Index].fingerMotorHandles[0],velToRegulate);
            simSetJointTargetPosition(allK3s[k3Index].fingerMotorHandles[1],-jp*0.5f);
            simSetJointTargetPosition(allK3s[k3Index].fingerMotorHandles[2],-jp*0.5f);
        }
    }

    if (message==sim_message_eventcallback_scriptstatedestroyed)
    { // script state was destroyed. Destroy all associated BubbleRob instances:
        int index=getK3IndexFromScriptHandle(auxiliaryData[0]);
        while (index>=0)
        {
            allK3s.erase(allK3s.begin()+index);
            index=getK3IndexFromScriptHandle(auxiliaryData[0]);
        }
    }

    return(retVal);
}
