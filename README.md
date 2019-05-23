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

Documentation and screenshots will be coming shortly, but the short version is that you need to figure out what port your sensor is on (e.g. COM3 on Windows; it's some wacky port on my osx machine), put that into the port field, and toggle the on switch. That *should* work, but I've certainly had issues during development where I'm not initializing the sensor correctly from whatever state it's in and something hangs.

I don't totally understand what clumps of data are being returned from the sensor when at the moment, so I'm taking the data that I'm getting from the scan call and bucketing it into half-degree increments. That means that the plugin should always be shooting to return 720 samples. You can get this in polar (angle/distance) or cartesian (x/y). Because I have my sensor oriented a particular way and I want the data to represent the real world when I feed it directly into instancing, I added an angle offset that can be fed into the plugin. If you let the data populate and adjust that value back and forth, it should be pretty easy to see what it does.


## TODO
- [ ] Try using the hq data methods from the sdk (good for A2, A3)
- [ ] Experiment with data representations other than the forced half-degree bucketing
