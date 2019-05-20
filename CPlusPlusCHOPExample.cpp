/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "CPlusPlusCHOPExample.h"

#include <string>

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillCHOPPluginInfo(CHOP_PluginInfo *info)
{
	// Always set this to CHOPCPlusPlusAPIVersion.
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	// The opType is the unique name for this CHOP. It must start with a 
	// capital A-Z character, and all the following characters must lower case
	// or numbers (a-z, 0-9)
	info->customOPInfo.opType->setString("Slamteca1lidar");

	// The opLabel is the text that will show up in the OP Create Dialog
	info->customOPInfo.opLabel->setString("Slamtec A1");

	// Information about the author of this OP
	info->customOPInfo.authorName->setString("Kai Curtis");
	info->customOPInfo.authorEmail->setString("morecode@kcurtis.com");

	// This CHOP can work with 0 inputs
	info->customOPInfo.minInputs = 0;

	// It can accept up to 1 input though, which changes it's behavior
	info->customOPInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
    printf("lidar chop::CreateCHOPInstance");
	return new CPlusPlusCHOPExample(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
    printf("lidar chop::DestroyCHOPInstance");
	delete (CPlusPlusCHOPExample*)instance;
}

};


CPlusPlusCHOPExample::CPlusPlusCHOPExample(const OP_NodeInfo* info) : myNodeInfo(info)
{
    printf("lidar chop::constructor");
    
	myExecuteCount = 0;
	myOffset = 0.0;
    calibrationFramesRemaining = 0;
    
    //    TODO: Make this not hardcoded; pull from param
    const char * opt_com_path = "/dev/tty.SLAB_USBtoUART";
    _u32         baudrateArray[2] = {115200, 256000};
    u_result     op_result;

    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    // Error handling for driver creation — not sure that this is the right thing to do in a CPPChop
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for(size_t i = 0; i < baudRateArraySize; ++i)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
                break;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }

    if (!connectSuccess) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
        return;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           , devinfo.firmware_version>>8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);



//    // check health...
//    // TODO: Add health check in?
////    if (!checkRPLIDARHealth(drv)) {
////        throw std::runtime_error("failed to construct");
////    }

    drv->startMotor();
    drv->startScan(0, 1);
}

CPlusPlusCHOPExample::~CPlusPlusCHOPExample()
{
    printf("lidar chop::destructor");

    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
}

void
CPlusPlusCHOPExample::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved1)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = false;
	ginfo->inputMatchIndex = 0;
}

bool
CPlusPlusCHOPExample::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved1)
{
	// If there is an input connected, we are going to match it's channel names etc
	// otherwise we'll specify our own.
	if (inputs->getNumInputs() > 0)
	{
		return false;
	}
	else
	{
		info->numChannels = 2;
        
        int numSamples = 0;
        for (int i=0; i<720; ++i) {
            if (distances[i] > 0.0f) {
                ++numSamples;
            }
        }
        info->numSamples = 720;
//        info->numSamples = inputs->getParDouble("Samples");

		// Since we are outputting a timeslice, the system will dictate
		// the numSamples and startIndex of the CHOP data
		//info->numSamples = 1;
		//info->startIndex = 0

		// For illustration we are going to output 120hz data
		info->sampleRate = inputs->getParDouble("Samples");
		return true;
	}
}

void
CPlusPlusCHOPExample::getChannelName(int32_t index, OP_String *name, const OP_Inputs* inputs, void* reserved1)
{
    std::string fulllabel;
    
    int coords = inputs->getParInt("Coordsystem");

    if      (coords == 0 && index == 0) fulllabel = "angle";
    else if (coords == 0 && index == 1) fulllabel = "distance";
    else if (coords == 1 && index == 0) fulllabel = "x";
    else if (coords == 1 && index == 1) fulllabel = "y";

	name->setString(fulllabel.c_str());
}

const int angleChannel = 0, xChannel = 0;
const int distanceChannel = 1, yChannel = 1;
const float degreesToRadians = (2.0f * 3.1415926535f)/360.0f;

void
CPlusPlusCHOPExample::execute(CHOP_Output* output,
							  const OP_Inputs* inputs,
							  void* reserved)
{
    u_result op_result;
    size_t   count = _countof(nodes);
    
//    op_result = drv->grabScanData(nodes, count);
    op_result = drv->getScanDataWithInterval(nodes, retrieved);
//    printf("Retrieved: %lu", retrieved);
    
    int samples = inputs->getParInt("Samples");
    float offsetDegrees = inputs->getParDouble("Offsetdegrees");
    
    float angle = 0.0f;
    float distance = 0.0f;
    
    if (IS_OK(op_result)) {
        drv->ascendScanData(nodes, count);
        int coordSystem = inputs->getParInt("Coordsystem");
        
        double tempAngle;
        int halfAngle;
        
        for (int pos = 0; pos < (int)samples ; ++pos) {
            float unadjustedAngle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
            if (unadjustedAngle > 360.0f) {
//                printf("Skipping bad angle: %f\n", unadjustedAngle);
                continue;
            }
            if (unadjustedAngle < 0.0f) {
//                printf("Skipping bad angle: %f\n", unadjustedAngle);
                continue;
            }
            tempAngle = offsetDegrees - unadjustedAngle;
            halfAngle = floor(tempAngle * 2);
            
            distance = nodes[pos].distance_q2/4.0f;
            if (distance > 5000.0f) {
                distances[halfAngle] = 0;
//                printf("Skipping bad distance: %f\n", distance);
                continue;
            }
            
            distances[halfAngle] = distance;
        }
        
        // if calibrating, calibrate!
        if (calibrationFramesRemaining > 0) {
            for (int pos = 0; pos < (int)720 ; ++pos) {
                if (distances[pos] < 5.0f) continue;
                if (calibration[pos] < 5.0f) calibration[pos] = distances[pos];
                calibration[pos] = ((calibrationFramesRemaining) * distances[pos] + (totalCalbrationFrames - calibrationFramesRemaining) * calibration[pos]) / (float)totalCalbrationFrames;
            }
            
            // DEBUG: print calibration data on last calibration frame
            if (calibrationFramesRemaining == 1) {
                for (int pos = 0; pos < (int)720 ; ++pos) {
                    printf("CAL: Angle %d    Distance %f\n", pos, calibration[pos]);
                }
            }
            
            --calibrationFramesRemaining;
        }

        switch (coordSystem) {
            case 0:
                for (int pos = 0, channelOffset = 0; pos < (int)720 ; ++pos) {
                    angle = pos/2.0f * degreesToRadians;
                    output->channels[angleChannel][channelOffset] = pos / 2.0f;
                    output->channels[distanceChannel][channelOffset] = distances[pos];
                    ++channelOffset;

//                for (int pos = 0; pos < (int)samples ; ++pos) {
//                    output->channels[angleChannel][pos] = offsetDegrees - (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
//                    output->channels[distanceChannel][pos] = nodes[pos].distance_q2/4.0f;
//                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                           (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
//                           (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
//                           nodes[pos].distance_q2/4.0f,
//                           nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                }
                break;
                
            case 1:
            default:
//                for (int pos = 0; pos < (int)samples ; ++pos) {
//                    float tempAngle = ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
//                    float offsetAngle = offsetDegrees - tempAngle;
//
//                    angle = floor(angle * 10.0f) / 10.0f;
//                    angle = offsetAngle * degreesToRadians;
//                    distance = nodes[pos].distance_q2/4.0f;
//
//                    output->channels[xChannel][pos] = distance * cos(angle);
//                    output->channels[yChannel][pos] = distance * sin(angle);

                for (int pos = 0, channelOffset = 0; pos < (int)720 ; ++pos) {
                    if (distances[pos] < 1.0f) continue;
                    
                    angle = pos/2.0f * degreesToRadians;
                    if (abs(calibration[pos] - distances[pos]) < 150.0f) {
                        distance = 0;
                    } else {
                        distance = distances[pos];
//                        if (myExecuteCount > 1020) {
//                            printf("Angle %d    Distance %f    CalDist: %f\n", pos, distances[pos], calibration[pos]);
//                        }
                    }
                    output->channels[xChannel][channelOffset] = distance * cos(angle);
                    output->channels[yChannel][channelOffset] = distance * sin(angle);
                    ++channelOffset;
                    
                    

                    
//                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                           (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
//                           (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
//                           nodes[pos].distance_q2/4.0f,
//                           nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                }
                break;
        }
    }

    
//    u_result op_result;
//    rplidar_response_measurement_node_hq_t nodes[8192];
//    size_t count = _countof(nodes);
//
//    op_result = drv->grabScanDataHq(nodes, count, 0);
//
//    int pos = 0;
//    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//           (nodes[pos].quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
//           (nodes[pos].angle_z_q14 >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
//           nodes[pos].dist_mm_q2 / 4.0f,
//           nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
//
//    return;
//
//    if (IS_OK(op_result))
//    {
//        drv->ascendScanData(nodes, count);
//        for (int pos = 0; pos < (int)count; ++pos)
//        {
//            printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                   (nodes[pos].quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
//                   (nodes[pos].angle_z_q14 >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
//                   nodes[pos].dist_mm_q2 / 4.0f,
//                   nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
//        }
//    }
    
    /////////////////
    
    myExecuteCount++;

    // In this case we'll just take the first input and re-output it scaled.

//    if (inputs->getNumInputs() > 0)
//    {
//        // We know the first CHOP has the same number of channels
//        // because we returned false from getOutputInfo.
//
//        inputs->enablePar("Speed", 0);    // not used
//        inputs->enablePar("Reset", 0);    // not used
//        inputs->enablePar("Shape", 0);    // not used
//
//        int ind = 0;
//        for (int i = 0 ; i < output->numChannels; i++)
//        {
//            for (int j = 0; j < output->numSamples; j++)
//            {
//                const OP_CHOPInput    *cinput = inputs->getInputCHOP(0);
//                output->channels[i][j] = float(cinput->getChannelData(i)[ind] * scale);
//                ind++;
//
//                // Make sure we don't read past the end of the CHOP input
//                ind = ind % cinput->numSamples;
//            }
//        }
//
//    }
//    else // If not input is connected, lets output a sine wave instead
//    {
//        inputs->enablePar("Speed", 1);
//        inputs->enablePar("Reset", 1);
//
//        double speed = inputs->getParDouble("Speed");
//        double step = speed * 0.01f;
//
//
//        // menu items can be evaluated as either an integer menu position, or a string
//        int shape = inputs->getParInt("Shape");
////        const char *shape_str = inputs->getParString("Shape");
//
//        // keep each channel at a different phase
//        double phase = 2.0f * 3.14159f / (float)(output->numChannels);
//
//        // Notice that startIndex and the output->numSamples is used to output a smooth
//        // wave by ensuring that we are outputting a value for each sample
//        // Since we are outputting at 120, for each frame that has passed we'll be
//        // outputing 2 samples (assuming the timeline is running at 60hz).
//
//
//        for (int i = 0; i < output->numChannels; i++)
//        {
//            double offset = myOffset + phase*i;
//
//
//            double v = 0.0f;
//
//            switch(shape)
//            {
//                case 0:        // sine
//                    v = sin(offset);
//                    break;
//
//                case 1:        // square
//                    v = fabs(fmod(offset, 1.0)) > 0.5;
//                    break;
//
//                case 2:        // ramp
//                    v = fabs(fmod(offset, 1.0));
//                    break;
//            }
//
//
//            v *= scale;
//
//            for (int j = 0; j < output->numSamples; j++)
//            {
//                output->channels[i][j] = float(v);
//                offset += step;
//            }
//        }
//
//        myOffset += step * output->numSamples;
//    }
}

int32_t
CPlusPlusCHOPExample::getNumInfoCHOPChans(void * reserved1)
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send one channel.
	return 2;
}

void
CPlusPlusCHOPExample::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan,
										void* reserved1)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name->setString("executeCount");
		chan->value = (float)myExecuteCount;
	}

	if (index == 1)
	{
		chan->name->setString("offset");
		chan->value = (float)myOffset;
	}
}

bool		
CPlusPlusCHOPExample::getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved1)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
CPlusPlusCHOPExample::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries, 
										void* reserved1)
{
	char tempBuffer[4096];

	if (index == 0)
	{
		// Set the value for the first column
		entries->values[0]->setString("executeCount");

		// Set the value for the second column
#ifdef WIN32
		sprintf_s(tempBuffer, "%d", myExecuteCount);
#else // macOS
        snprintf(tempBuffer, sizeof(tempBuffer), "%d", myExecuteCount);
#endif
		entries->values[1]->setString(tempBuffer);
	}

	if (index == 1)
	{
		// Set the value for the first column
		entries->values[0]->setString("offset");

		// Set the value for the second column
#ifdef WIN32
        sprintf_s(tempBuffer, "%g", myOffset);
#else // macOS
        snprintf(tempBuffer, sizeof(tempBuffer), "%g", myOffset);
#endif
		entries->values[1]->setString( tempBuffer);
	}
}

void
CPlusPlusCHOPExample::setupParameters(OP_ParameterManager* manager, void *reserved1)
{
    {
        OP_NumericParameter    np;

        np.name = "Samples";
        np.label = "Samples";

        np.minSliders[0] = 1;
        np.maxSliders[0] = 720;
        np.defaultValues[0] = 720;
        
        OP_ParAppendResult res = manager->appendFloat(np);
        assert(res == OP_ParAppendResult::Success);
    }
    
    {
        OP_StringParameter    sp;
        
        sp.name = "Coordsystem";
        sp.label = "Coordsystem";
        
        sp.defaultValue = "Polar";
        
        const char *names[] = { "Polar", "Cartesian" };
        const char *labels[] = { "Polar", "Cartesian" };
        
        OP_ParAppendResult res = manager->appendMenu(sp, 2, names, labels);
        assert(res == OP_ParAppendResult::Success);
    }
    
    {
        OP_NumericParameter    np;
        
        np.name = "Offsetdegrees";
        np.label = "Offset (degrees)";
        
        np.minSliders[0] = 0;
        np.maxSliders[0] = 360;
        np.defaultValues[0] = 0;
        
        OP_ParAppendResult res = manager->appendFloat(np);
        assert(res == OP_ParAppendResult::Success);
    }
    
    // shape
    {
        OP_StringParameter    sp;
        
        sp.name = "Shape";
        sp.label = "Shape";
        
        sp.defaultValue = "Sine";
        
        const char *names[] = { "Sine", "Square", "Ramp" };
        const char *labels[] = { "Sine", "Square", "Ramp" };
        
        OP_ParAppendResult res = manager->appendMenu(sp, 3, names, labels);
        assert(res == OP_ParAppendResult::Success);
    }

    
//    // shape
//    {
//        OP_StringParameter    com_port;
//
//        com_port.name = "COM Port";
//        com_port.label = "COM Port";
//
//        com_port.defaultValue = "/dev/tty.SLAB_USBtoUART";
//
//        const char *names[] = { "/dev/tty.SLAB_USBtoUART" };
//        const char *labels[] = { "/dev/tty.SLAB_USBtoUART" };
//
//        OP_ParAppendResult res = manager->appendMenu(com_port, 1, names, labels);
//        assert(res == OP_ParAppendResult::Success);
//    }

	// pulse
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset";
		
		OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}

}

void 
CPlusPlusCHOPExample::pulsePressed(const char* name, void* reserved1)
{
	if (!strcmp(name, "Reset"))
	{
        for (int i=0; i<720; ++i) {
            calibration[i] = 0.0f;
        }
		calibrationFramesRemaining = totalCalbrationFrames;
	}
}

