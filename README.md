# SlamtecLidarTDCPPCHOP
A CPP CHOP for TouchDesigner for gathering data from a Slamtec RPLidar scanner

## Building

### Starting point for both platforms

1. Clone the project *and submodules* (`git clone --recurse-submodules` works well for this)
1. Go to steps below for your platform

### MAC

1. Start with platform-independent steps above
1. Open the .xcodeproj file
1. Build

You should now have a .plugin file to use with the cpp chop.

### WINDOWS
_Note: solution and projects are set up for VS2019 (made in VS2019 Community Edition)._

1. Start with platform-independent steps above
1. Open the top level .sln file
1. Build

You should now have .dll to use with the cpp chop. (By default, this will be located in a Debug directory that Visual Studio will create within the main directory when you build.) 


## Interface / How to use

1. Plug in your sensor (being sure to use a USB cable that isn't power only; some of us learned this the hard way)
1. Determine what port your sensor is on

    If you've used the command line tools from the [rplidar SDK](https://github.com/Slamtec/rplidar_sdk), then you should've had to find this already.
    
    Example ports from my machines:
    
    OSX - `/dev/tty.SLAB_USBtoUART`
    
    Win - `\\.\COM3`
  
1. In a TouchDesigner project, make a new CPP CHOP
1. For the path, use the location of the .dll (on Windows) or .plugin (on OSX)
1. Go to the custom parameters page of the CPP CHOP
1. Enter the correct port path in the `COM Port` field
1. Click the active toggle

At this point, your lidar sensor should pause and then spin back up, and some data should appear in your CHOP. There's a good chance that you'll need to use the offset slider to get the correct distance data to appear. (This is due to a bug that needs fixing.)

## Miscellaneous notes and caveats

Data from the device is currently being bucketed into half-degree slots. You can get this in polar (angle/distance) or cartesian (x/y). Because you may want to reorient the data, there's an angle offset value that can be used to rotate the .


## TODO
- [ ] Try using the hq data methods from the sdk (good for A2, A3)
- [ ] Experiment with data representations other than the forced half-degree bucketing
- [ ] Make an example TD project
