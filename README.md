# SlamtecLidarTDCPPCHOP
A CPP CHOP for TouchDesigner for gathering data from a Slamtec RPLidar scanner

## External documentation
- Lake Heckaman posted a nice video on using this plugin; check it out! https://www.youtube.com/watch?v=fAvF2niosNA

## Downloading

There are downloadable versions of the plugin available in the Releases. However, the mac version was built many moons ago on Intel silicon. ~~I've got an M2 in the office these days, though, so I should be able to go in and make a new build.~~ I don't really have an office anymore, so I could use some help building for ARM macs. Caveat: I've got no experience aside from this in building native libraries, so it may not work for you, which is why I've got the building instructions below.

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


## How to use

_NB: [@AlphaMoonbaseBerlin](https://github.com/AlphaMoonbaseBerlin) left this note in the issues:_
> On windows, if there is not internet or just a metered connection, Windows will not autoDownload the driver for the bridge.
Either force it to download via windowsUpdate or download the drivers directly.
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads


1. Plug in your sensor (being sure to use a USB cable that isn't power only; some of us learned this the hard way)
1. Determine what port your sensor is on

    If you've used the command line tools from the [rplidar SDK](https://github.com/Slamtec/rplidar_sdk), then you should've had to find this already.
    
    Example ports from my machines:
    
    OSX - `/dev/tty.SLAB_USBtoUART` (the hard-coded default)
    
    Win - `\\.\COM3`
  
1. In a TouchDesigner project, make a new CPP CHOP
1. For the path, use the location of the .dll (on Windows) or .plugin (on OSX)
1. Go to the custom parameters page of the CPP CHOP
1. Enter the correct port path in the `COM Port` field
1. Click the active toggle

At this point, your lidar sensor should pause and then spin back up, and some data should appear in your CHOP.

## Interface
_Diagram coming soon!_

**Active** - toggle this on to make the plugin attempt to connect to the lidar sensor. If given a bad port, this will currently fail silently and the toggle will stay on. To re-attempt connection, toggle this off and back on again.

**COM Port** - the port on which the driver will attempt to connect. This should be a COM port on Windows and a `/dev/tty` port on OSX.

**Coordsystem** - the coordinate system in which data will be returned.

    Polar: data comes in as an angle channel and corresponding distance channel
    
    Cartesian: data comes in as an x channel and corresponding y channel
    
**Offset** - the degrees by which to rotate all incoming data. Example use case: when I have my sensor mounted the way I want, the data as mapped by default looks like it's off by 90 degrees i.e. boxes instanced to what I'd consider data from the ceiling form a vertical line on the left. Using the offset value, I can rotate the ceiling to be a horizontal line across the top of my instances.

**Reset** - (experimental) a button that will set a "baseline" for the data. Clicking this will store averages of the next several rotations at each half-degree and then only send data through that's significantly different from that baseline data. The idea here was that I might want to situate the sensor and effectively zero out the default distances so that I only really see _changing_ data. Currently disconnected from output, so tapping this should do nothing.

## Miscellaneous notes and caveats

 - Data from the device is currently being bucketed into half-degree slots
 - **BUG**: Cartesian data isn't properly filling out the channel


## TODO
- [ ] Try using the hq data methods from the sdk (good for A2, A3)
- [ ] Experiment with data representations other than the forced half-degree bucketing
- [ ] Make an example TD project
