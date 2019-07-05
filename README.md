# SlamtecLidarTDCPPCHOP
A CPP CHOP for TouchDesigner for gathering data from a Slamtec RPLidar scanner

## Building

1. **REMEMBER TO CLONE THE SUBMODULE**
1. **REMEMBER TO CLONE THE SUBMODULE**
1. **REMEMBER TO CLONE THE SUBMODULE**

### MAC

Building should be pretty straightforward. Clone the repo, **make sure you have the git submodule for the rplidar sdk pulled**, then open the .xcodeproj file and build to produce a .plugin artifact that you can pull in to your TD project.

### WINDOWS
Building should be pretty straightforward? I haven't checked to make sure that this works on other machines, in other directories, etc. Solution and projects are set up for VS2019 (made in VS2019 Community Edition). Clone the repo, **make sure you have the git submodule for the rplidar sdk pulled**, then open the top level .sln file and build to produce a .dll artifact that you can pull in to your TD project. (By default, this will be located in a Debug directory that Visual Studio will create within the main directory when you build.) 


## Interface / How to use

1. Plug in your sensor, being sure to use a USB cable that isn't power only
1. Determine what port your sensor is on

    More directions for this will be forthcoming, but the default path is the one from my OSX machine. On Windows, this will usually be a COM# path e.g. `\\.\COM11`. If you've used the command line tools from the [rplidar SDK](https://github.com/Slamtec/rplidar_sdk), then you should've had to find this already.
  
1. In a TouchDesigner project, make a new CPP CHOP
1. For the path, use the location of the .dll (on Windows) or .plugin (on OSX)
1. Go to the custom parameters page of the CPP CHOP
1. Enter the correct port path in the `COM Port` field
1. Click the active toggle

At this point, your lidar sensor should pause and then spin back up, and some data should appear in your CHOP. There's a good chance that you'll need to use the offset slider to get the correct distance data to appear. (This is due to a bug that needs fixing.)

## Miscellaneous notes and caveats

I don't totally understand what clumps of data are being returned from the sensor when at the moment, so I'm taking the data that I'm getting from the scan call and bucketing it into half-degree increments. That means that the plugin should always be shooting to return 720 samples. You can get this in polar (angle/distance) or cartesian (x/y). Because I have my sensor oriented a particular way and I want the data to represent the real world when I feed it directly into instancing, I added an angle offset that can be fed into the plugin. If you let the data populate and adjust that value back and forth, it should be pretty easy to see what it does.


## TODO
- [ ] Try using the hq data methods from the sdk (good for A2, A3)
- [ ] Experiment with data representations other than the forced half-degree bucketing
- [ ] Fix the offset bug so that data always appears, just with an offset
- [ ] Make an example TD project
