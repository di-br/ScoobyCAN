### ! This is a test repository !
Absolutely everything is work-in-progress and will work for me and me alone.
You're on your own and have been warned.

**originally based on a fork from [fabiobaltieri/toyothack](https://github.com/fabiobaltieri/toyothack)**  
Since his repository does not seem to be active and I do not assume a merge would be desirable, I started a new repository. It is, however, worthwhile visiting his [blog entry](https://fabiobaltieri.com/2013/07/23/hacking-into-a-vehicle-can-bus-toyothack-and-socketcan/).

# ScoobyCAN
A 'tool' to convert CAN frames of a Subaru into human readable form with an ncurses interface.

### General idea
Starting from the original repository as inspiration and code skeleton, this is my attempt to look at CAN data while sniffing the CAN bus.  
The interfaced used is [USBtin](http://www.fischl.de/usbtin/), along with SocketCAN interfaces and the lot. Recording/replaying data is easy and still fun. Ultimately I want to play with the available data and see what I can make of it. The CAN bus will only be read, no inkjecting of data.

## Current status
Well, it kind a works. Still needs extending. It looks like this:
![screenshot](https://github.com/di-br/ScoobyCAN/blob/master/examples/screenshot.png "screenshot")
