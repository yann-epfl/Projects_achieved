/***************************************************************************
 *   Copyright (C) 2011 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *   $Rev: 1870 $
 *   $Id: ufunclocalize.h 1870 2012-03-16 14:41:22Z eba $
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UFUNC_LOCALIZE_H
#define UFUNC_LOCALIZE_H

#include <cstdlib>
#include <list>

using namespace std;

#include <ulms4/ufunclaserbase.h>
#include <urob4/uresposehist.h>
#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <umap4/uposev.h>
#include <LEL_ransac.h>
#include <iau_mat.h>
#include <eigen3/Eigen/Dense>
#include <mhf/MultiHypDist.h>
// #include <mhf/SplitTable.h>
using namespace Eigen;

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/*const float odoB = 0.26;
const float kR = 0.0003;
const float kL = 0.0003;*/
//float lsrRelx;  // Read from the server after each scan
//float lsrRely;  // Read from the server after each scan
//float lsrRelTh;  // Read from the server after each scan
//const float varAlpha = 0.001;
//const float varR = 0.0004;
//const int MAXMATCHABLELINES = 100;
//const int MAXLASERLINES = 20;

/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class UFuncLocalize : public UFuncLaserBase
{
public:
  /**
	Constructor */
	UFuncLocalize();
	/**
	Destructor */
	virtual ~UFuncLocalize();

  /**
    Called by the server core. Should return the
    name of function. There should be a first short part separated
    by a space to some additional info (e.g. version and author).
    The returned name is intended as informative to clients
    and should include a version number */
    //virtual const char * name();
    /**
    Called by the server core when loaded, to get a list of
    keywords (commands) handled by this plugin.
    Return a list of handled functions in
    one string separated by a space.
    e.g. return "ball".
    The functions should be unique on the server. */
    //virtual const char * commandList();
    /**
    List (space separated) of shared resources
    provided by this function.
    Must be an empty string if no resources are to be shared.
    Each resource ID must be no longer than 20 characters long. */
    //virtual const char * resourceList()
    //{ return "odoPose"; }
    /**
    Called by the server core, when a new resource is
    available (or is removed), local pointers to the resource should be
    updated as appropriate. */
    virtual bool setResource(UResBase * resource, bool remove);
    /**
    This function is called by the server core, when it needs a
    resource provided by this plugin.
    Return a pointer to a resource with an ID taht matches this 'resID' ID string.
    The string match should be case sensitive.
    Returns false if the resource faled to be created (e.g. no memory space). */
    //UResBase * getResource(const char * resID);

  /**
  return true if all ressources is available */
  //virtual bool gotAllResources(char * missingThese, int missingTheseCnt);

  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Update module status (global variables)
  \param mapX is the localized position in X
  \param mapY is the localized position in Y
  \param mapTh is the localized position in radians
  \param time is the valid time for the position
  \param match was the last update a match. */
  void updateStatus(double mapX, double mapY, double mapTh, UTime time);
  /**
  Update covariance status in global variables. */
  void updateCovStatus();


protected:
  list<LEL_ARLine> lineList;
  Matrix<double,3,1> pose;
  Matrix<double,3,3> poseCov;
  MultiHypDist<3> poseDist;
  SplitTable<1> table;

  UResPoseHist * poseHist;
  int poseIndex;
  UTime lastScanTime;
  int matchMiss;
  UPosRot lasPose;
  double transX, transY, transTh;

  /// number of points that must be inside line segment to correlate
  UVariable * varPointsInThreshold;
  /// value of current covariance matrix
  UVariable * varCovar;
  /// eigenvectors of x,y part of current covariance matrix
  UVariable * varCovarV;
  /// eigenvalues of x,y part of current covariance matrix
  UVariable * varCovarA;
  /// number of successful match updates
  UVariable * varUpdates;
  /// number of failed matches since last successful match
  UVariable * varFailed;
  /// number of defined lines in localizer
  UVariable * varDefinedLines;
  /// robot base (differential drive?)
  UVariable * varOdoB; // = 0.26;
  /// distance varaince for right wheel each moved meter
  UVariable * varKR; // = 0.0003;
  /// distance varaince for left wheel each moved meter
  UVariable * varKL; // = 0.0003;
  /// The maximum allowed variances for each hypothesis
  UVariable * varMaxXYVariance; // = 0.05;
  UVariable * varMaxThVariance; // = 0.05;
  /// The noise covariances for laser readings
  UVariable * varLaserAlpha; // = 0.001;
  UVariable * varLaserR; // = 0.0004;


private:
  bool handleOdoposeUpdate();
  bool handleLocalize(UServerInMsg * msg, void * extra);
  bool handleLocalizeUKF(UServerInMsg * msg, void * extra);
  bool handleLocalizeMHF(UServerInMsg * msg, void * extra);
  bool handleAddLine(UServerInMsg * msg);
  /**
   * Add a detectable line from 3D position p1 to 3D position p2 */
  void addLine(UPosition p1, UPosition p2);
  bool handleSetInitPose(UServerInMsg * msg);
  bool handleSetInitCov(UServerInMsg * msg);
  bool handleResetLocalizer(UServerInMsg * msg);
  bool handleOutputDist(UServerInMsg * msg);
  bool handleSetTable(UServerInMsg * msg);
  bool handleResample(UServerInMsg * msg);
  void projectToLaser(LEL_ARLine worldLine, Matrix<double,3,1> & pose, Matrix<double,3,3> & poseCov, LEL_ARLine &projLine, Matrix<double,2,2> &lineCov, Matrix<double,2,3> & delH_delP);

  //Matrix<double,2,1> projectToLaserUKF(Matrix<double,3,1> pose, Matrix<double,2,1> noise, Matrix<double,5,1> auxin);
  /**
  Create local variables for manipulating parameters */
  void createBaseVar();

public:
  Matrix<double,2,1> projectToLaserUKF(Matrix<double,3,1> pose, Matrix<double,2,1> noise, Matrix<double,2,1> measline, Matrix<double,5,1> worldLine_lasPose);

};


#endif

