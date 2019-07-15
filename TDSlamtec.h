/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include <string>

#include "CHOP_CPlusPlusBase.h"
#include <rplidar.h>

using namespace rp::standalone::rplidar;

/*

This example file implements a class that does 2 different things depending on
if a CHOP is connected to the CPlusPlus CHOPs input or not.
The example is timesliced, which is the more complex way of working.

If an input is connected the node will output the same number of channels as the
input and divide the first 'N' samples in the input channel by 2. 'N' being the current
timeslice size. This is noteworthy because if the input isn't changing then the output
will look wierd since depending on the timeslice size some number of the first samples
of the input will get used.

If no input is connected then the node will output a smooth sine wave at 120hz.
*/

// To get more help about these functions, look at CHOP_CPlusPlusBase.h
class CPlusPlusCHOPExample : public CHOP_CPlusPlusBase
{
public:
	CPlusPlusCHOPExample(const OP_NodeInfo* info);
	virtual ~CPlusPlusCHOPExample();

	virtual void		getGeneralInfo(CHOP_GeneralInfo*, const OP_Inputs*, void* ) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*, const OP_Inputs*, void*) override;
	virtual void		getChannelName(int32_t index, OP_String *name, const OP_Inputs*, void* reserved) override;

	virtual void		execute(CHOP_Output*,
								const OP_Inputs*,
								void* reserved) override;


	virtual int32_t		getNumInfoCHOPChans(void* reserved1) override;
	virtual void		getInfoCHOPChan(int index,
										OP_InfoCHOPChan* chan,
										void* reserved1) override;

	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize, void* resereved1) override;
	virtual void		getInfoDATEntries(int32_t index,
											int32_t nEntries,
											OP_InfoDATEntries* entries,
											void* reserved1) override;

	virtual void		setupParameters(OP_ParameterManager* manager, void *reserved1) override;
	virtual void		pulsePressed(const char* name, void* reserved1) override;

private:
    void connect(std::string com_path);
    void disconnect();

	// We don't need to store this pointer, but we do for the example.
	// The OP_NodeInfo class store information about the node that's using
	// this instance of the class (like its name).
	const OP_NodeInfo*	myNodeInfo;
    
    // Driver instance for the lidar driver
    RPlidarDriver *drv;
    rplidar_response_measurement_node_t     nodes[8192];

    // Note on sample count:
    //  When letting the lidar device fill in the nodes set aside above, as
    //  in the included example, it looks like maybe 500-600 good samples
    //  followed by a bunch of bad data. (That is for the blocking call.)
    //
    //  To get around this, I'm just assuming that I can get samples for
    //  every half degree, and whatever data comes in from the async scan
    //  call, I'll bucket it into the half degree that it rounds to. Need
    //  to do more analysis of the data coming back from the scans to see
    //  if there's a better method.

    static const int sampleCount = 720;
    double distances[sampleCount] = {0};
    double calibration[sampleCount] = {0};

    const int totalCalbrationFrames = 60;
    int calibrationFramesRemaining = 0;
    
    size_t retrieved;

	// In this example this value will be incremented each time the execute()
	// function is called, then passes back to the CHOP 
	int32_t				myExecuteCount;

	double				myOffset;
};
