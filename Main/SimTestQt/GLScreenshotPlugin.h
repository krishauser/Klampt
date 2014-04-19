#ifndef GL_SCREENSHOT_PLUGIN_H
#define GL_SCREENSHOT_PLUGIN_H

#include <utils/stringutils.h>
#include <Timer.h>
#include <string>
#include <GLdraw/GLScreenshot.h>
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
  string moviefile;

  GLScreenshotPlugin()
  {
    saveMovie = false;
    lastScreenshotTime = 0;
    frameTime = 1.0/30.0;
    screenshotFile = "image0000.ppm";
    verbose = 1;
    moviefile="klampt_record.mpg";
    stop_encode=true;
  }

  virtual ~GLScreenshotPlugin() {}

  void SaveScreenshot()
  {
    const char* screenshotFilec=screenshotFile.c_str();
    GLSaveScreenshotPPM(screenshotFilec);
    if(verbose) printf("Screenshot saved to %s\n",screenshotFile.c_str());
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

  #ifdef WIN32
      printf("No encoding support on Windows\n");
      return;
  #else
      system(("ffmpeg -y -f image2 -i image%04d.ppm " + moviefile + ";rm image*.ppm").c_str());
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
    if(saveMovie) StopMovie();
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
	printf("Time %g last %g, Saving %d screenshots\n",t,lastScreenshotTime,(int)Floor((t-lastScreenshotTime)/frameTime));
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
