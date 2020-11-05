#ifndef DEBUG_X_H
#define DEBUG_X_H

#include <ros/console.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */     // CALL BACK QUEUE COUNT //Debug open
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */  // line detector
#define BLUE    "\033[34m"      /* Blue */    // laser_calibration // convert scan data type
#define MAGENTA "\033[35m"      /* Magenta */ // Receive scan // Skip scan
#define CYAN    "\033[36m"      /* Cyan */    
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */      // params
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */    // icp //close process
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */   // Params setting // start thread  
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */     // occ map
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */  // params
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */     // pose estimator
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */    // loop closure

#endif