#ifndef GL_SCREENSHOT_PLUGIN_H
#define GL_SCREENSHOT_PLUGIN_H

#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/Timer.h>
#include <string>
#include <KrisLibrary/GLdraw/GLScreenshot.h>
#include <KrisLibrary/math/math.h>
#include <stdio.h>

/** @ingroup GLDraw
 * @brief A plugin class that "automatically" saves a movie to disk in the form of
 * PPM screenshots.
 *
 * To save the screenshots, set #saveMovie to true and call MovieUpdate() during your
 * idle loop.
 *
 * By default, the frame rate is 30fps and the screenshot file format is image[xxxx].ppm
 * where [xxxx] is the frame number.  To change the fps, set #frameTime to 1.0/fps.
 * To use another file format, set #screenshotFile to the desired name of the initial
 * frame.  The program will automatically increment the digits in #screenshotFile,
 * keeping the same number of leading zeroes if possible.
 */

class GLScreenshotPlugin
{
public:
  std::string screenshotFile;
  bool saveMovie;
  double lastScreenshotTime;
  double frameTime;
  Timer timer;
  int verbose;
  bool stop_encode; //encodes automatically when recording ends
  std::string moviefile;
  std::string video_encoding_command;

  GLScreenshotPlugin()
  {
    saveMovie = false;
    lastScreenshotTime = 0;
    frameTime = 1.000/30.0;
    screenshotFile = "image0000.ppm";
    verbose = 1;
    moviefile="klampt_record.mp4";
    video_encoding_command = "ffmpeg -y -qscale 2 -f image2 -i image%04d.ppm";
    stop_encode=true;
  }

  virtual ~GLScreenshotPlugin() {}

  void SaveScreenshot()
  {
    const char* screenshotFilec=screenshotFile.c_str();
    bool res=GLSaveScreenshotPPM(screenshotFilec);
    if(!res) LOG4CXX_ERROR(KrisLibrary::logger(),"Error saving screenshot to "<<screenshotFile.c_str());
    if(verbose) LOG4CXX_INFO(KrisLibrary::logger(),"Screenshot saved to "<<screenshotFile.c_str());
  }

  void StartMovie()
  {
    saveMovie = true;
    lastScreenshotTime = 0;
    timer.Reset();
  }

  void EncodeMovie(){
      //TODO add native support
      //http://ffmpeg.org/doxygen/trunk/api-example_8c-source.html
      //temporarily using system calls

  #ifdef WIN32
      system((video_encoding_command + " " + moviefile + " & del image*.ppm").c_str());
      return;
  #else
      system((video_encoding_command + " " + moviefile + ";rm image*.ppm").c_str());
      //system(("avconv -y -f image2 -i image%04d.ppm " + moviefile + ";rm image*.ppm").c_str());
  #endif
      screenshotFile = "image0000.ppm";
  }

  void StopMovie()
  {
    saveMovie = false;
    if(stop_encode)
      EncodeMovie();
  }

  void ToggleMovie()
  {
    if(saveMovie)
      StopMovie();
    else StartMovie();
  }

  //call this if you want a real-time movie
  void MovieUpdate()
  {
    this->MovieUpdate(timer.ElapsedTime());
  }

  //call this if you want to control movie time
  void MovieUpdate(double t)
  {
    if(saveMovie) {
      if(t >= lastScreenshotTime + frameTime) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Time "<<t<<" last "<<lastScreenshotTime<<", Saving "<<(int)Math::Floor((t-lastScreenshotTime)/frameTime));
	while(lastScreenshotTime+frameTime < t) {
	  SaveScreenshot();
	  IncrementStringDigits(screenshotFile);
	  lastScreenshotTime += frameTime;
	}
      }
    }
  }
};


#endif
