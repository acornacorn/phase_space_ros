/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Edited by: Sean Seungkook Yun <seungkook.yun@sri.com> 
*/

#include <string>
#include <sstream>
#include <cstdio>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <ctime>
#include <time.h>
#include <unistd.h>
#include "tf/tf.h"
#include "phasespace/phaseSpaceDriver.h"
#include "src_folder.h"

using namespace phasespace;

struct termios oldterm, newterm;
void set_unbuffered ( void ) {
  tcgetattr( STDIN_FILENO, &oldterm );
  newterm = oldterm;
  newterm.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newterm );
}
void set_buffered ( void ) {
  tcsetattr( STDIN_FILENO, TCSANOW, &oldterm );
}

// kbhit() function
int kbhit ( void ) {
    int result;
    fd_set  set;
    struct timeval tv;

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO,&set);  /* watch stdin */
    tv.tv_sec = 0;
    tv.tv_usec = 0;             /* don't wait */

    /* quick peek at the input, to see if anything is there */
    set_unbuffered();
    result = select( STDIN_FILENO+1,&set,NULL,NULL,&tv);
    set_buffered();

    return result == 1;
}


int main( int argc, char** argv )
{
  ROS_INFO("starting phasespace print");
  std::vector<std::string> file_names;

  ROS_INFO("Absolute Path: %s", SRC_FOLDER);

  std::stringstream ss;
  ss<<SRC_FOLDER<<"/penA-14.rb";
  file_names.push_back(ss.str());

  std::stringstream ss2;
  ss2<<SRC_FOLDER<<"/penB-12.rb";
  file_names.push_back(ss2.str());


  phaseSpaceDriver phasespace("phasespace-pc", file_names);
  if (phasespace.init()<0) {
	  ROS_ERROR("PhaseSpace init failed. check the device?");
	  return -1;
  }
  ROS_INFO("phasespace inited");

  std::vector<tf::Pose> poses;

  while (!kbhit()) {
	phasespace.read_phasespace(poses);
	ROS_INFO("pose0 x: %4.1f  y: %4.1f  z: %4.1f ",
			poses[0].getOrigin().getX(),
			poses[0].getOrigin().getY(),
			poses[0].getOrigin().getZ());
	ROS_INFO("pose0 w: %3.2f  x: %3.2f  y: %3.2f  z: %3.2f ",
			poses[0].getRotation().w(),
			poses[0].getRotation().x(),
			poses[0].getRotation().y(),
			poses[0].getRotation().z());
    usleep(100000);
  }

	return 0;
}


