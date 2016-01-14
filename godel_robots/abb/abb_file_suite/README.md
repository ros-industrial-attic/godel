# abb_file_suite
A suite of tools used for interfacing ROS-I and an ABB robot through the use of module files that are generated and downloaded on the fly.

## ABB Controller Side
The `abb_motion_ftp_downloader` interface uses the standard ROS-I State Server and a custom motion server. Files for it can be found under the `rapid/` subdirectory. The system works by listening for the existence of a specific file in the `HOME/PARTMODULES` directory on the controller. Files are uploaded via FTP using [libcurl](http://curl.haxx.se/libcurl/). This library can be downloaded through the apt-get interface:

```
sudo apt-get install libcurl4-openssl-dev
```

## Rapid Generator
When using the Rapid generation routines, be sure to flush/close your output file before sending it to the 'execute program' service. 

