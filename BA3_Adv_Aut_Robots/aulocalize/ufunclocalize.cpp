/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
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
#include <urob4/usmltag.h>
#include "ufunclocalize.h"
#include "../aupoly/urespoly.h"
#include <utils/localizationutils.h>

#include <iostream>
#include <string.h>
#include <mhf/iau_ukf_eigen.hpp>
#include <boost/foreach.hpp>

#include <fstream>

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
	/** replace 'UFuncNear' with your classname */
	return new UFuncLocalize();
}
#endif

Matrix<double,2, 1> UFuncLocalize::projectToLaserUKF(Matrix<double,3, 1> pose, Matrix<double,
		2, 1> noise, Matrix<double,2, 1> measline, Matrix<double,5, 1> worldLine_lasPose) {
	Matrix<double,2, 1> worldLine = worldLine_lasPose.block<2, 1> (0, 0);
	Matrix<double,3, 1> lasPose = worldLine_lasPose.block<3, 1> (2, 0);
	double posex = pose(0, 0);
	double posey = pose(1, 0);
	double poseTh = pose(2, 0);
	double alpha = worldLine(0, 0) - poseTh;
	Matrix<double,2, 1> projLine;
	projLine(0, 0) = alpha - lasPose(2, 0) + noise(0, 0);
	projLine(1, 0) = worldLine(1, 0) - (posex * cos(worldLine(0, 0)) + posey
			* sin(worldLine(0, 0))) - (lasPose(0, 0) * cos(alpha) + lasPose(1,
					0) * sin(alpha)) + noise(1, 0);

	double angleDif =
			fmod(measline(0, 0) - projLine(0, 0) + 3 * M_PI, 2 * M_PI) - M_PI;
	double rDif;
	if (fabs(angleDif) > M_PI / 2) {
		angleDif = fmod(angleDif + 2 * M_PI, 2 * M_PI) - M_PI;
		rDif = -measline(1, 0) - projLine(1, 0);
	} else
		rDif = measline(1, 0) - projLine(1, 0);
	Matrix<double,2, 1> lineDif;
	lineDif << angleDif, rDif;
	return lineDif;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncLocalize::UFuncLocalize(): table() { // initialization of variables in class - as needed
	setCommand(
			"settable addline setinitpose setinitcov localizeMHF localize resetlocalizer outputdist odoposeupdate resample localizeUKF",
			"localize",
			"auLocalize (" __DATE__ " " __TIME__ " by Enis BAYRAMOGLU)");
	//setResID("localize", 915);
	createBaseVar();
	//
	poseIndex = 0;
	lastScanTime.setTime(-1);
	matchMiss = 100;

	//Initialize the trans x,y,th variables for returning to MRC
	transX = 0.0;
	transY = 0.0;
	transTh = 0.0;

}

UFuncLocalize::~UFuncLocalize() { // possibly remove allocated variables here - if needed

}

void UFuncLocalize::createBaseVar() {
	varOdoB = addVar("wheelBase", 0.26, "d",
			"(rw) (odoB) wheel base for assumed differential drive robot");
	/// distance varaince for right wheel each moved meter
	varKR
	= addVar("sdmRight", sqrt(0.003), "d",
			"(rw) (kR) variance each driven meter - as sd of err each m (right)");
	/// distance varaince for left wheel each moved meter
	varKL = addVar("sdmLeft", sqrt(0.003), "d",
			"(rw) (kL) variance each driven meter - as sd (left)");
	varMaxXYVariance = addVar("maxXYVariance", 0.05, "d",
			"(rw) The maximum variance of each hypothesis along the x and y dimensions");
	varMaxThVariance = addVar("maxThVariance", 0.05, "d",
			"(rw) The maximum variance of each hypothesis along the theta dimension");
	varLaserAlpha = addVar("laserAlphaVariance", 0.001, "d",
			"(rw) The assumed noise variance over the laser alpha reading (of lines)");
	varLaserR = addVar("laserRVariance", 0.0004, "d",
			"(rw) The assumed noise variance over the laser R reading (of lines)");
	varDefinedLines = addVar("lines", 0.0, "d",
			"(r) Number lines defined in localizer");
	varPointsInThreshold
	= addVar("pointsIn", 0.1, "d",
			"(r/w) number of points supporting line that must be inside line segment");
	varCovar = addVar("covar", "0 0 0; 0 0 0; 0 0 0", "m2",
			"(r) Current covariance martix (x, y, th)");
	varCovarV = addVar("covarV", "0 0; 0 0", "d",
			"(r) covariance ellipse vectors - x,y in rows");
	varCovarA = addVar("covarA", "0 0", "d",
			"(r) covariance ellipse length - major; minor; degrees (1-sigma)");
	varUpdates = addVar("hit", 0.0, "d",
			"(r) Number of successful updates (total)");
	varFailed = addVar("mis", 0.0, "d",
			"(r) Number of scans with no update since last update");
}

// const char * UFuncLocalize::name()
// {
//   return "auLocalize (" __DATE__ " " __TIME__ " by Enis BAYRAMOGLU)";
// }
//
// const char * UFuncLocalize::commandList()
// { // space separated list og command keywords handled by this plugin
//   return "addline setinitpose setinitcov localize localizeUKF odoposeupdate resetlocalizer";
// }

///////////////////////////////////////////////////

bool UFuncLocalize::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
	bool result = true;

	if (resource->isA(UResPoseHist::getOdoPoseID())) { // pointer to server the resource that this plugin can provide too
		// but as there might be more plugins that can provide the same resource
		// use the provided
		if (remove)
			// the resource is unloaded, so reference must be removed
			poseHist = NULL;
		else if (poseHist != (UResPoseHist *) resource)
			// resource is new or is moved, save the new reference
			poseHist = (UResPoseHist *) resource;
		else
			// reference is not used
			result = false;
	}

	// other resource types may be needed by base function.
	result = UFunctionBase::setResource(resource, remove);
	return result;
}

bool UFuncLocalize::handleCommand(UServerInMsg * msg, void * extra) { // message is unhandled
	bool result = false;
	// Test for the handled commands, and call a function to do the job
	// the tagname is not case sensitive - see the library documentation for
	// 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
	if (msg->tag.isTagA("odoposeupdate"))
		result = handleOdoposeUpdate();
	else if (msg->tag.isTagA("localize"))
		result = handleLocalize(msg, extra);
	else if (msg->tag.isTagA("localizeUKF"))
		result = handleLocalizeUKF(msg, extra);
	else if (msg->tag.isTagA("localizeMHF"))
		result = handleLocalizeMHF(msg, extra);
	else if (msg->tag.isTagA("addline"))
		result = handleAddLine(msg);
	else if (msg->tag.isTagA("setinitpose"))
		result = handleSetInitPose(msg);
	else if (msg->tag.isTagA("setinitcov"))
		result = handleSetInitCov(msg);
	else if (msg->tag.isTagA("resetlocalizer"))
		result = handleResetLocalizer(msg);
	else if (msg->tag.isTagA("outputdist"))
		result = handleOutputDist(msg);
	else if (msg->tag.isTagA("settable"))
		result = handleSetTable(msg);
	else if (msg->tag.isTagA("resample"))
		result = handleResample(msg);
	else
		sendDebug(msg, "Command not handled (by me)");
	return result;
}

bool UFuncLocalize::handleResample(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL];
	bool ask4help;
	//
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "RESAMPLE");
		sendText(msg, "--- Description of resample command ---\n");
		sendText(msg," Resample the gaussian hypotheses.\n");
		sendText(msg," Example:\n");
		sendText(msg," resample samplecount=100\n");
		sendHelpDone(msg);
		return true;
	}
	if (msg->tag.getAttValue("samplecount", val, MVL)) {
		int samplecount = static_cast<int>(strtol(val,NULL,10));
		poseDist.resample(samplecount);
	}
	return true;
}

bool UFuncLocalize::handleSetTable(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL]="dist";
	bool ask4help;
	//
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "SETTABLE");
		sendText(msg, "--- Description of settable command ---\n");
		sendText(msg," Set the splitting table to be used.\n");
		sendText(msg," Example:\n");
		sendText(msg," settable filename=dist\n");
		sendHelpDone(msg);
		return true;
	}
	if (msg->tag.getAttValue("filename", val, MVL)) {
		table = SplitTable<1>(val);
	}
	return true;
}

bool UFuncLocalize::handleOutputDist(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL]="dist";
	bool ask4help;
	//
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "OUTPUTDIST");
		sendText(msg, "--- Description of outputdist command ---\n");
		sendText(msg," Write the current distribution to the given file name.\n");
		sendText(msg," Example:\n");
		sendText(msg," outputdist filename=dist\n");
		sendHelpDone(msg);
		return true;
	}
 	if (msg->tag.getAttValue("filename", val, MVL)) {
	  printf("UFuncLocalize::handleOutputDist: failed to write to disk - disabled due to compile error in new c++ compiler\n");
// 		ofstream the_file;
// 		the_file.open(val);
// 		the_file<<poseDist;
// 		the_file.close();
 	}
	return true;
}


bool UFuncLocalize::handleResetLocalizer(UServerInMsg * msg) {
	bool ask4help;
	ask4help = msg->tag.getAttValue("help", NULL, 0);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "RESETLOCALIZER");
		sendText(msg, "--- resetlocalizer options:\n");
		sendText(msg, " resetlocalizer accepts no options\n\n");
		sendText(msg, "--- Description of resetlocalizer command ---\n");
		sendText(
				msg,
				" Deletes the current line map and specifies the next scan to be the first scan. That is, localize doesn't try to estimate the pose during scan.\n");
		sendHelpDone(msg);
		return true;
	} else {
		lastScanTime.setTime(-1);
		poseIndex = 0;
		lineList.clear();
		varDefinedLines->setDouble(0.0);
		sendInfo("reset");
	}
	return true;
}

bool UFuncLocalize::handleOdoposeUpdate() {
	//printf("received a new pose with time:%f\n",poseHist->getNewest(NULL).t.getDecSec());
	return true;
}

bool UFuncLocalize::handleAddLine(UServerInMsg * msg) {
  const int MVL = 50;
  char val[MVL];
  double alpha=NAN;
  bool alphaavailable = false;
  double r=NAN;
  bool ravailable = false;
  double startx=NAN;
  double starty=NAN;
  double endx=NAN;
  double endy=NAN;
  bool startxavailable = false;
  bool startyavailable = false;
  bool endxavailable = false;
  bool endyavailable = false;
  bool ask4help;
  char name[LEL_ARLine::MNL] = "noname";
  bool result = false;
  int n = 0;
  //
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (ask4help) { // create the reply in XML-like (html - like) format
    sendHelpStart("ADDLINE");
    sendText("--- addline options:\n");
    sendText(" polyLine=name         Add line segments from polygon plug-in with 'name', name may be e.g. 'mapLine*' for all lines starting with mapLine\n");
    sendText(" help                  this help\n");

    sendText(" addline accepts no options\n\n");
    sendText("--- Description of addline command ---\n");
    sendText(
        " This command is used to construct the line map to be used by the localize command. One line is added per call. ");
    sendText(
        "Lines could be infinite or finite length. localize will not match finite lines to the extracted lines that lie outside their finite range..\n\n");
    sendText("--- Usage ---\n");
    sendText(
        " addline is called with two alternative ways, which have different meanings.\n");
    sendText(
        " The first way adds an infinitely long line described by its distance from the origin (r) and the angle of its normal(alpha).\n");
    sendText(
        " addline alpha='the angle of the line normal' r='the line distance to the origin'\n");
    sendText(
        " The second way adds a finite length line described by its starting and end points.\n");
    sendText(
        " addline startx='startx' starty='starty' endx='endx' endy='endy'\n");
    sendHelpDone();
    return true;
  }
  if (msg->tag.getAttValue("alpha", val, MVL)) {
    alpha = strtod(val, NULL);
    alphaavailable = true;
  }
  if (msg->tag.getAttValue("r", val, MVL)) {
    r = strtod(val, NULL);
    ravailable = true;
  }
  if (msg->tag.getAttValue("startx", val, MVL)) {
    startx = strtod(val, NULL);
    startxavailable = true;
  }
  if (msg->tag.getAttValue("starty", val, MVL)) {
    starty = strtod(val, NULL);
    startyavailable = true;
  }
  if (msg->tag.getAttValue("endx", val, MVL)) {
    endx = strtod(val, NULL);
    endxavailable = true;
  }
  if (msg->tag.getAttValue("endy", val, MVL)) {
    endy = strtod(val, NULL);
    endyavailable = true;
  }
  msg->tag.getAttValue("name", name, LEL_ARLine::MNL);
  //
  if (alphaavailable && ravailable) {
    LEL_ARLine newLine(alpha, r);
    lineList.push_front(newLine);
    result = true;
    n++;
  } else if (startxavailable && startyavailable && endxavailable
      && endyavailable) {
    LEL_ARLine newLine(startx, starty, endx, endy, name);
    lineList.push_front(newLine);
    result = true;
    n++;
  }
  else if (msg->tag.getAttValue("polyLine", val, MVL))
  { // 'val' is now the name of the lines to added
    UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
    UPolyItem * poly;
    int p1 = 0, p2;
    const int MNL = 32;
    char s[MNL];
    //
    if ((resPoly != NULL) and (strlen(val) > 0))
    poly = resPoly->getNext(p1, &p2, val);
    while (poly != NULL)
    {
      UPosition * pos1 = poly->getPoints();
      UPosition pos2;
      for (int i = 0; i < poly->getPointsCnt() - 1; i++)
      { // line name in localizer uses polygon name and add a index number
        snprintf(s, MNL, "%s_%03d", poly->name, i);
        pos2 = pos1[1];
        if (pos1->dist(pos2) > 0.2)
        { // must have some length to be usefull
          LEL_ARLine newLine(pos1->x, pos1->y, pos2.x, pos2.y, s);
          lineList.push_front(newLine);
          n++;
        }
        pos1++;
      }
      p1 = p2;
      poly = resPoly->getNext(p1, &p2, val);
    }
  }
  else {
    cout << "inappropriate arguments for addline\n";
  }
  varDefinedLines->setDouble((double) lineList.size());
  snprintf(val, MVL, "added %d line(s), now defined %lu lines\n", n, lineList.size());
  sendInfo(val);
  return result;
}

bool UFuncLocalize::handleSetInitPose(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL];
	double x=NAN;
	double xavailable = false;
	double y=NAN;
	double yavailable = false;
	double th=NAN;
	double thavailable = false;
	bool ask4help;
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "SETINITPOSE");
		sendText(msg, "--- setinitpose options:\n");
		sendText(msg, " setinitpose accepts no options\n\n");
		sendText(msg, "--- Description of setinitcov command ---\n");
		sendText(msg, " set the initial robot pose to be used by localize.\n\n");
		sendText(msg, "--- Usage ---\n");
		sendText(msg,
				" setinitpose has to be called with all tree pose parameters as follows:\n");
		sendText(
				msg,
				" setinitcov x='robot x position(in meters)' y='robot y position' th='robot heading'\n");
		sendHelpDone(msg);
		return true;
	}
	if (msg->tag.getAttValue("x", val, MVL)) {
		x = strtod(val, NULL);
		xavailable = true;
	}
	if (msg->tag.getAttValue("y", val, MVL)) {
		y = strtod(val, NULL);
		yavailable = true;
	}
	if (msg->tag.getAttValue("th", val, MVL)) {
		th = strtod(val, NULL);
		thavailable = true;
	}

	if (xavailable && yavailable && thavailable) {
		pose << x, y, th;
		updateStatus(x, y, th, lastScanTime);
    sendInfo("done");
		return true;
	} else {
		//cout << "inappropriate arguments for setinitpose\n";
    sendWarning("inappropriate arguments for setinitpose");
		return false;
	}

}

bool UFuncLocalize::handleSetInitCov(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL];
	double Cx=NAN;
	double Cxavailable = false;
	double Cy=NAN;
	double Cyavailable = false;
	double Cth=NAN;
	double Cthavailable = false;
	bool ask4help;
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "SETINITCOV");
		sendText(msg, "--- setinitcov options:\n");
		sendText(msg, " setinitcov accepts no options\n\n");
		sendText(msg, "--- Description of setinitcov command ---\n");
		sendText(msg,
				" set the covariance of the initial robot pose to be used by localize.\n\n");
		sendText(msg, "--- Usage ---\n");
		sendText(
				msg,
				" setinitcov has to be called with all tree covariance parameters as follows:\n");
		sendText(
				msg,
				"setinitcov Cx='variance of x(in meter^2)' Cy='variance of y' Cth='variance of heading'\n");
		sendHelpDone(msg);
		return true;
	}

	if (msg->tag.getAttValue("Cx", val, MVL)) {
		Cx = strtod(val, NULL);
		Cxavailable = true;
	}
	if (msg->tag.getAttValue("Cy", val, MVL)) {
		Cy = strtod(val, NULL);
		Cyavailable = true;
	}
	if (msg->tag.getAttValue("Cth", val, MVL)) {
		Cth = strtod(val, NULL);
		Cthavailable = true;
	}

	if (Cxavailable && Cyavailable && Cthavailable) {
		poseCov << Cx, 0, 0, 0, Cy, 0, 0, 0, Cth;
		GaussianHypothesis<3> initDist(pose,poseCov,1);
		Matrix<double, 3, 1> maxVariances;
		maxVariances << varMaxXYVariance->getValued(), varMaxXYVariance->getValued(), varMaxThVariance->getValued();
		GaussianHypothesis<3>::list splittedList;
		poseDist.split(initDist,splittedList,maxVariances,table);
		poseDist.GHlist.splice_after(poseDist.GHlist.begin(),splittedList);
		updateCovStatus();
    sendInfo("done");
		return true;
	} else {
		//cout << "inappropriate arguments for setinitcov\n";
    sendWarning("inappropriate arguments for setinitcov");
		return false;
	}
}

bool UFuncLocalize::handleLocalizeMHF(UServerInMsg * msg, void * extra) { // handle a plugin command
	const int MRL = 500;
	char reply[MRL];
	bool ask4help;
	const int MVL = 30;
	char value[MVL];
	ULaserData * data;
	int match = 0;
	bool aList = false, getonly = false;
  int sl;
	//
	// check for parameters - one parameter is tested for - 'help'
	ask4help = msg->tag.getAttValue("help", value, MVL);
	aList = msg->tag.getAttValue("list", value, MVL);
	getonly = msg->tag.getAttValue("getonly", value, MVL);
	if (msg->tag.getAttInteger("silent", &sl, true))
    silent = sl;
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "LOCALIZE");
		sendText(msg, "--- localize options:\n");
		sendText(msg,
				"list             list loaded lines (no localize performed)\n");
		sendText(
				msg,
				"silent[=false]   Be more or less silent in status printout\n");
		sendText(msg,
				"getonly          Return MRC laservariables only (no localize performed)\n");
		sendText(msg, "--- Description of localize command ---\n");
		sendText(msg,
				" localize first extracts lines from the most recent laser scan ");
		sendText(msg,
				"and uses that combined with a map of lines to estimate the current ");
		sendText(msg,
				"robot pose in the map frame. It has two output channels. First, it ");
		sendText(msg,
				"sends the map (world) origin in the odometry frame through the mrc ");
		sendText(msg,
				"variables $l0 $l1 $l2 (x y th). Second, it sends pose-time information ");
		sendText(msg, "to \"mappose\" whenever it's called.\n\n");
		sendText(msg, "--- Usage ---\n");
		sendText(msg, "localize requires the following to have been called:\n");
		sendText(msg, "setinitpose - to set the initial pose\n");
		sendText(msg, "setinitcov - to set the initial pose covariance\n");
		sendText(msg, "addline - to add lines to the map\n");
    sendText(msg, "Other update parameters see: 'VAR LOCALIZE'\n");
		sendHelpDone(msg);
	} else if (aList) {
		list<LEL_ARLine>::iterator itWrld;
		int n;
		//
		n = 0;
		snprintf(reply, MRL, "List of loaded %d localizer line(s)",
				(int) lineList.size());
		sendHelpStart(reply);
		for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++) {
			if (itWrld->limited) {
				snprintf(
						reply,
						MRL,
						"   %d a=%8.5frad r=%7.4fm (%7.3fx,%7.3fy to %7.3fx,%7.3fy) %s\n",
						n, itWrld->alpha, itWrld->r, itWrld->p1[0],
						itWrld->p1[1], itWrld->p2[0], itWrld->p2[1],
						itWrld->name);
				//printf("%s", reply);
			} else
				snprintf(reply, MRL, "   %d a=%8.5frad r=%7.4fm (unlimited)\n",
						n, itWrld->alpha, itWrld->r);
			sendText(reply);
			n++;
		}
		sendHelpDone();
	}
	//Only return laservariables without running localizer
	else if (getonly) {
		snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n",
				transX, transY, transTh);
		// send this string as the reply to the client
		sendMsg(msg, reply);

	} else if (msg->tag.getAttValue("resample", value, MVL)) {
		int samplecount = static_cast<int>(strtol(value,NULL,10));
		poseDist.resample(samplecount);
		return true;
	} else {

		// do some action and send a reply
		data = getScan(msg, (ULaserData*) extra);
		//
		if (data->isValid()) { // make analysis for closest measurement
			ULaserDevice * device = getDevice(msg, data);
			lasPose = device->getDevicePose();

			//		lsrRelx=lasPose->x;
			//		lsrRely=lasPose->y;
			//		lsrRelTh=0;

			UPose uNewPose = poseHist->getPoseAtTime(data->getScanTime());
			UTime scanTime = data->getScanTime();

			if (lastScanTime.getDecSec() == -1) {
				if (silent == 0)
					cout << "processingFirstScan\n";
			} else {
				updateDisplacement(lastScanTime, poseDist, scanTime, poseHist,
						silent, varOdoB->getValued(), varKR->getValued(), varKL->getValued());
			}

			if (msg->tag.getAttValue("split", value, MVL)) {
				Matrix<double, 3, 1> maxVariances;
				maxVariances << varMaxXYVariance->getValued(), varMaxXYVariance->getValued(), varMaxThVariance->getValued();
				poseDist.split(maxVariances, table);
			}

			double devThreshold = 3 * 3;
			double pointsInThreshold = varPointsInThreshold->getDouble();
			double minRange = 0.05;

			const int MAXNOOFSCANPOINTS = 2000;
			double X[MAXNOOFSCANPOINTS];
			double Y[MAXNOOFSCANPOINTS];

			if (data->getRangeCnt() > MAXNOOFSCANPOINTS)
				printf(
						"UFUNCLOCALIZE::HANDLELOCALIZEMHF: Warning, num of scan points (%d) larger that the specified maximum",data->getRangeCnt());

			int dataI,i;
			for (i = 0, dataI = 0; i < data->getRangeCnt(); i++) { // range are stored as an integer in current units
				bool rangeValid;
				double r = data->getRangeMeter(i, &rangeValid);
				if (!rangeValid || r < minRange)
					continue;
				double alpha = data->getAngleRad(i);
				X[dataI] = r * cos(alpha);
				Y[dataI] = r * sin(alpha);
				dataI++;
			}

			list<LEL_GFLine> GFLL;
			ransac(X, Y, dataI, GFLL);

			list<LEL_GFLine>::iterator itLas;


			for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++) {
				LEL_ARLine line = (*itLas).toARLine();
				if (silent == 0)
					cout << "line params:" << line.alpha << ", " << line.r << "\n";
			}

			list<LEL_ARLine>::iterator itWrld;
			if (silent == 0) {
				cout
				<< "\nprinting pose and poseCov before measurement update\n";
				cout << poseDist.getMean();
				cout << poseDist.getCovariance() << "\n";
			}
			LEL_ARLine bestLine;
			Matrix<double,2, 3> delH_delPMin;
			Matrix<double,2, 1> lineDifMin;
			Matrix<double,2, 2> lineDifCovMin;
			double varAlpha = varLaserAlpha->getValued(), varR = varLaserR->getValued();
			vector<GaussianHypothesis<3> *> hypotheses(poseDist.GHlist.size());
			int index = 0;
			BOOST_FOREACH(GaussianHypothesis<3> & gh, poseDist.GHlist){ hypotheses[index++] = &gh;}
			for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++) {
				double matchProbability =0;
				BOOST_FOREACH(GaussianHypothesis<3> &gh, poseDist.GHlist) {
//				int numOfHyps = hypotheses.size();
//				int i;
//				#pragma omp parallel for default(shared) private(i) schedule(static,4) reduction(+:matchProbability)
//				for(i=0; i<numOfHyps; i++) {
//					GaussianHypothesis<3> &gh = *hypotheses[i];
					Matrix<double,3,1> &poseHyp = gh.mean;
					Matrix<double,3,3> &poseCovHyp = gh.cov;
					LEL_ARLine projLine;
					Matrix<double,2, 3> delH_delP;
					Matrix<double,2, 2> lineDifCov;
					projectToLaser(*itWrld, poseHyp, poseCovHyp, projLine, lineDifCov,
							delH_delP);
					lineDifCov(0, 0) += varAlpha;
					lineDifCov(1, 1) += varR;
					double minMahDist = devThreshold;
					LEL_ARLine closestLine;
					for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++) {
						LEL_ARLine line = (*itLas).toARLine();
						double angleDif = fmod(line.alpha - projLine.alpha + 3
								* M_PI, 2 * M_PI) - M_PI;
						double rDif;
						if (fabs(angleDif) > M_PI / 2) {
							angleDif = fmod(angleDif + 2 * M_PI, 2 * M_PI) - M_PI;
							rDif = -line.r - projLine.r;
						} else {
							rDif = line.r - projLine.r;
						}
						Matrix<double,2, 1> lineDif;
						lineDif << angleDif, rDif;
						double mahDist = lineDif.transpose()
											* lineDifCov.inverse() * lineDif;
						if (mahDist < minMahDist) {
							if (projLine.limited) {
								int pointsIn = 0;
								for (int p = 0; p < itLas->edgeCount; p++) {
									double x = itLas->edgesX[p];
									double y = itLas->edgesY[p];
									double lp = projLine.positionAlongLine(x, y);
									if (projLine.lb < lp and lp < projLine.le)
										pointsIn++;
								}
								if (pointsIn < pointsInThreshold * itLas->edgeCount) {
									// cout<<"Match ruled out due to line limits mismatch\n";
									continue;
								}
							}
							minMahDist = mahDist;
							delH_delPMin = delH_delP;
							lineDifMin = lineDif;
							lineDifCovMin = lineDifCov;
							closestLine = line;
							bestLine = *itWrld;
						}
					}
					double likelihood;
					if (minMahDist < devThreshold) {
						matchProbability+=gh.weight;
						Matrix<double,2, 1> meas;
						meas << closestLine.alpha, closestLine.r;
						Matrix<double,2, 2> noisecov;
						noisecov << varAlpha, 0, 0, varR;
						Matrix<double,5, 1> auxin;
						auxin << bestLine.alpha, bestLine.r, lasPose.x, lasPose.y, lasPose.Kappa;
						likelihood = iau_ukf::update(poseHyp, poseCovHyp, meas, noisecov, auxin,
								*this, &UFuncLocalize::projectToLaserUKF);
						//cout << "updated\n";
					} else {
						likelihood = exp(-devThreshold)/sqrt(0.1*0.1); // This is the likelihood of total randomness
					}
					gh.weight *= likelihood;
				}
				poseDist.normalize();
				if ((silent == 0) || ((silent == 1) && matchProbability>0))
					cout << "Localizer: Line match to " << bestLine.name
					<< " with probability:" << matchProbability<<"\n";

			}
			//pose=newPose;
			//cout<<poseCov;
			if (silent == 0) {
				cout
				<< "\nprinting pose and poseCov after measurement update\n";
				cout << poseDist.getMean();
				cout.setf(std::ios_base::scientific);
				cout << "\n Pose Covariance: \n";
				cout << poseDist.getCovariance();
			}
			cout.setf(std::ios_base::fixed);

			UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());

			UPose trans;
			Matrix<double,3,1> poseMean = poseDist.getMean();

			double poseX = poseMean(0, 0);
			double poseY = poseMean(1, 0);
			double poseTh = poseMean(2, 0);

			trans.h = poseAtScan.h - poseTh;
			trans.x = poseAtScan.x - (cos(trans.h) * poseX - sin(trans.h)
					* poseY);
			trans.y = poseAtScan.y - (sin(trans.h) * poseX + cos(trans.h)
					* poseY);

			if (match > 0)
				matchMiss = 0;
			else
				matchMiss += 1;
			if (silent == 1)
				printf("Localizer: got %d line matches\n", match);
			if (match)
				varUpdates->add(1.0);
			//
			updateStatus(poseX, poseY, poseTh, data->getScanTime());
			//
			lastScanTime = scanTime;

			//Save the trans variables to class-variables
			transX = trans.x;
			transY = trans.y;
			transTh = trans.h;

			if (msg->client >= 0) { // send to real client only, not a push in the .ini file
				snprintf(reply, MRL,
						"<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n", trans.x,
						trans.y, trans.h);

				// send this string as the reply to the client
				sendMsg(msg, reply);
			}
		} else
			sendWarning(msg, "No scandata available");
	}
	// return true if the function is handled with a positive result
	return true;
}

bool UFuncLocalize::handleLocalizeUKF(UServerInMsg * msg, void * extra) { // handle a plugin command
	const int MRL = 500;
	char reply[MRL];
	bool ask4help;
	const int MVL = 30;
	char value[MVL];
	ULaserData * data;
	int match = 0;
	bool aList = false, getonly = false;
	//
	int i;
	// check for parameters - one parameter is tested for - 'help'
	ask4help = msg->tag.getAttValue("help", value, MVL);
	aList = msg->tag.getAttValue("list", value, MVL);
	getonly = msg->tag.getAttValue("getonly", value, MVL);
	msg->tag.getAttBool("silent", &silent, true);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "LOCALIZE");
		sendText(msg, "--- localize options:\n");
		sendText(msg,
				"list             list loaded lines (no localize performed)\n");
		sendText(
				msg,
				"silent[=false]   Be more or less silent in printout\n");
		sendText(msg,
				"getonly          Return MRC laservariables only (no localize performed)\n");
		sendText(msg, "--- Description of localize command ---\n");
		sendText(msg,
				" localize first extracts lines from the most recent laser scan ");
		sendText(msg,
				"and uses that combined with a map of lines to estimate the current ");
		sendText(msg,
				"robot pose in the map frame. It has two output channels. First, it ");
		sendText(msg,
				"sends the map (world) origin in the odometry frame through the mrc ");
		sendText(msg,
				"variables $l0 $l1 $l2 (x y th). Second, it sends pose-time information ");
		sendText(msg, "to \"mappose\" whenever it's called.\n\n");
		sendText(msg, "--- Usage ---\n");
		sendText(msg, "localize requires the following to have been called:\n");
		sendText(msg, "setinitpose - to set the initial pose\n");
		sendText(msg, "setinitcov - to set the initial pose covariance\n");
		sendText(msg, "addline - to add lines to the map\n");
    sendText(msg, "Other update parameters see: 'VAR LOCALIZE'\n");
		sendHelpDone(msg);
	} else if (aList) {
		list<LEL_ARLine>::iterator itWrld;
		int n;
		//
		n = 0;
		snprintf(reply, MRL, "List of loaded %d localizer line(s)",
				(int) lineList.size());
		sendHelpStart(reply);
		for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++) {
			if (itWrld->limited) {
				snprintf(
						reply,
						MRL,
						"   %d a=%8.5frad r=%7.4fm (%7.3fx,%7.3fy to %7.3fx,%7.3fy) %s\n",
						n, itWrld->alpha, itWrld->r, itWrld->p1[0],
						itWrld->p1[1], itWrld->p2[0], itWrld->p2[1],
						itWrld->name);
				//printf("%s", reply);
			} else
				snprintf(reply, MRL, "   %d a=%8.5frad r=%7.4fm (unlimited)\n",
						n, itWrld->alpha, itWrld->r);
			sendText(reply);
			n++;
		}
		sendHelpDone();
	}
	//Only return laservariables without running localizer
	else if (getonly) {
		snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n",
				transX, transY, transTh);
		// send this string as the reply to the client
		sendMsg(msg, reply);

	} else { // do some action and send a reply
		data = getScan(msg, (ULaserData*) extra);
		//
		if (data->isValid()) { // make analysis for closest measurement
			ULaserDevice * device = getDevice(msg, data);
			lasPose = device->getDevicePose();

			//		lsrRelx=lasPose->x;
			//		lsrRely=lasPose->y;
			//		lsrRelTh=0;

			UPose uNewPose = poseHist->getPoseAtTime(data->getScanTime());
			UTime scanTime = data->getScanTime();

			if (lastScanTime.getDecSec() == -1) {
				if (silent == 0)
					cout << "processingFirstScan\n";
			} else {
				updateDisplacement(lastScanTime, pose, poseCov, scanTime, poseHist,
						silent, varOdoB->getValued(), varKR->getValued(), varKL->getValued());
				//				UPoseTVQ uPoseTime1;
				//				UPoseTVQ uPoseTime2;
				//				poseHist->getPoseNearTime(lastScanTime, &uPoseTime1,
				//						&uPoseTime2);
				//				UPose P1 = uPoseTime1.getPose();
				//				//printf("scanTime:%f\n", scanTime);
				//				while (1) {
				//					if (poseHist->getNewest(0).t == uPoseTime1.t) {
				//						if (silent == 0)
				//							cout << "\tpose list end\n";
				//						break;
				//					}
				//					if (not poseHist->getPoseNearTime(uPoseTime1.t + 0.001,
				//							&uPoseTime1, &uPoseTime2))
				//						break;
				//					UPose P2 = uPoseTime2.getPose();
				//					//cout << "\tPose Time Diff:" << uPoseTime2.t.getDecSec()- scanTime << "\n";
				//					if (uPoseTime2.t > (scanTime + 0.001)) {
				//						//cout << "\time at scan reached\n";
				//						break;
				//					}
				//					double D, L, delSr, delSl;
				//					D = fmod(P2.h - P1.h + 3 * M_PI, 2 * M_PI) - M_PI;
				//					double xDif = (P2.x - P1.x);
				//					double yDif = (P2.y - P1.y);
				//					L = sqrt(xDif * xDif + yDif * yDif) * ((cos(P1.h) * xDif
				//							+ sin(P1.h) * yDif > 0) * 2 - 1);
				//					//cout << "\tD:" << D << " L:" << L << "\n";
				//
				//					double odoB = varOdoB->getValued();
				//					delSr = L + odoB * D / 2;
				//					delSl = L - odoB * D / 2;
				//
				//					double th_ = pose(2, 0) + D / 2;
				//
				//					double thd = P2.h - P1.h;
				//					double xd = cos(P1.h) * (P2.x - P1.x) + sin(P1.h) * (P2.y
				//							- P1.y);
				//					double yd = -sin(P1.h) * (P2.x - P1.x) + cos(P1.h) * (P2.y
				//							- P1.y);
				//					double thl = pose(2, 0);
				//
				//					Matrix<double,3, 1> poseDif;
				//					poseDif = cos(thl) * xd - sin(thl) * yd, sin(thl) * xd
				//							+ cos(thl) * yd, thd;
				//
				//					pose += poseDif;
				//
				//					Matrix<double,3, 2> delPnew_delY;
				//					delPnew_delY = cos(th_) / 2 - L * sin(th_) / (2 * odoB), cos(
				//							th_) / 2 + L * sin(th_) / (2 * odoB), sin(th_) / 2
				//							+ L * cos(th_) / (2 * odoB), sin(th_) / 2 - L
				//							* cos(th_) / (2 * odoB), 1 / odoB, -1 / odoB;
				//
				//					Matrix<double,3, 3> delPnew_delPold;
				//					delPnew_delPold = 1, 0, -L * sin(th_), 0, 1, L * cos(th_), 0, 0, 1;
				//
				//					Matrix<double,2, 2> covU;
				//					covU = sqr(varKR->getValued()) * fabs(delSr), 0, 0, sqr(
				//							varKL->getValued()) * fabs(delSl);
				//
				//					//printf("Robot %gbase l=%g R=%g\b", odoB, varKL->getValued(), varKR->getValued());
				//
				//					//covOut = delPnew_delPold*covIn*delPnew_delPold'+delPnew_delY*covU*delPnew_delY';*/
				//
				//					poseCov = delPnew_delY * covU * delPnew_delY.transpose()
				//							+ delPnew_delPold * poseCov
				//									* delPnew_delPold.transpose();
				//
				//					uPoseTime1 = uPoseTime2;
				//					P1 = P2;
				//					poseIndex++;
				//
				//				}
				//
				//				if (silent == 0)
				//					cout << "\tupdated displacement\n";
			}

			double devThreshold = 3 * 3;
			double pointsInThreshold = varPointsInThreshold->getDouble();
			double minRange = 0.05;

			const int MAXNOOFSCANPOINTS = 2000;
			double X[MAXNOOFSCANPOINTS];
			double Y[MAXNOOFSCANPOINTS];

			if (data->getRangeCnt() > MAXNOOFSCANPOINTS)
				printf(
						"UFUNCLOCALIZE::HANDLELOCALIZEUKF: Warning, num of scan points larger that the specified maximum");

			int dataI;
			for (i = 0, dataI = 0; i < data->getRangeCnt(); i++) { // range are stored as an integer in current units
				bool rangeValid;
				double r = data->getRangeMeter(i, &rangeValid);
				if (!rangeValid || r < minRange)
					continue;
				double alpha = data->getAngleRad(i);
				X[dataI] = r * cos(alpha);
				Y[dataI] = r * sin(alpha);
				dataI++;
			}

			list<LEL_GFLine> GFLL;
			ransac(X, Y, dataI, GFLL);

			list<LEL_GFLine>::iterator itLas;

			for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++) {
				LEL_ARLine line = (*itLas).toARLine();
				if (silent == 0)
					cout << "\nline params:" << line.alpha << ", " << line.r;
			}

			list<LEL_ARLine>::iterator itWrld;
			if (silent == 0) {
				cout
				<< "\nprinting pose and poseCov before measurement update\n";
				cout << pose;
				cout << poseCov << "\n";
			}
			//Matrix<double,3, 1> newPose = pose;
			LEL_ARLine bestLine;
			Matrix<double,2, 3> delH_delPMin;
			Matrix<double,2, 1> lineDifMin;
			Matrix<double,2, 2> lineDifCovMin;
			double varAlpha = varLaserAlpha->getValued(), varR = varLaserR->getValued();
			for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++) {
				LEL_ARLine projLine;
				Matrix<double,2, 3> delH_delP;
				Matrix<double,2, 2> lineDifCov;
				projectToLaser(*itWrld, pose, poseCov, projLine, lineDifCov,
						delH_delP);
				lineDifCov(0, 0) += varAlpha;
				lineDifCov(1, 1) += varR;
				double minMahDist = devThreshold;
				LEL_ARLine closestLine;
				for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++) {
					LEL_ARLine line = (*itLas).toARLine();
					double angleDif = fmod(line.alpha - projLine.alpha + 3
							* M_PI, 2 * M_PI) - M_PI;
					double rDif;
					if (fabs(angleDif) > M_PI / 2) {
						angleDif = fmod(angleDif + 2 * M_PI, 2 * M_PI) - M_PI;
						rDif = -line.r - projLine.r;
					} else
						rDif = line.r - projLine.r;
					Matrix<double,2, 1> lineDif;
					lineDif << angleDif, rDif;
					double mahDist = lineDif.transpose()
									* lineDifCov.inverse() * lineDif;
					if (mahDist < minMahDist) {
						if (projLine.limited) {
							int pointsIn = 0;
							for (int p = 0; p < itLas->edgeCount; p++) {
								double x = itLas->edgesX[p];
								double y = itLas->edgesY[p];
								double lp = projLine.positionAlongLine(x, y);
								if (projLine.lb < lp and lp < projLine.le)
									pointsIn++;
							}
							if (pointsIn < pointsInThreshold * itLas->edgeCount) {
								// cout<<"Match ruled out due to line limits mismatch\n";
								continue;
							}
						}
						minMahDist = mahDist;
						delH_delPMin = delH_delP;
						lineDifMin = lineDif;
						lineDifCovMin = lineDifCov;
						closestLine = line;
						bestLine = *itWrld;
					}
				}
				if (minMahDist < devThreshold) {
					if ((silent == 0) || (silent == 1))
						cout << "Localizer: Line match to " << bestLine.name
						<< "\n";
					/* // We have a match!
					 Matrix<double,3,1> poseDif = newPose-pose;

					 Matrix<double,2,1> v = lineDifMin-(delH_delPMin*poseDif);

					 Matrix<double,3,2> K = poseCov*delH_delPMin.transpose()*lineDifCovMin.invgauss();
					 //      		  cout<<"\n printing K:\n";
					 //      		  mprint(K);

					 newPose+=K*v;

					 poseCov-=K*lineDifCovMin*K.transpose();
					 match++;;*/

					Matrix<double,2, 1> meas;
					meas << closestLine.alpha, closestLine.r;
					Matrix<double,2, 2> noisecov;
					noisecov << varAlpha, 0, 0, varR;
					Matrix<double,5, 1> auxin;
					auxin << bestLine.alpha, bestLine.r, lasPose.x, lasPose.y, lasPose.Kappa;
					iau_ukf::update(pose, poseCov, meas, noisecov, auxin,
							*this, &UFuncLocalize::projectToLaserUKF);
					cout << "updated\n";
					match++;
				}

			}

			//pose=newPose;
			//cout<<poseCov;
			if (silent == 0) {
				cout
				<< "\nprinting pose and poseCov after measurement update\n";
				cout << pose;
				cout.setf(std::ios_base::scientific);
				cout << "\n Pose Covariance: \n";
				cout << poseCov << "\n";
			}
			cout.setf(std::ios_base::fixed);

			UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());

			UPose trans;

			double poseX = pose(0, 0);
			double poseY = pose(1, 0);
			double poseTh = pose(2, 0);

			trans.h = poseAtScan.h - poseTh;
			trans.x = poseAtScan.x - (cos(trans.h) * poseX - sin(trans.h)
					* poseY);
			trans.y = poseAtScan.y - (sin(trans.h) * poseX + cos(trans.h)
					* poseY);

			if (match > 0)
				matchMiss = 0;
			else
				matchMiss += 1;
			if (silent == 1)
				printf("Localizer: got %d line matches\n", match);
			if (match)
				varUpdates->add(1.0);
			//
			updateStatus(poseX, poseY, poseTh, data->getScanTime());
			//
			lastScanTime = scanTime;

			//Save the trans variables to class-variables
			transX = trans.x;
			transY = trans.y;
			transTh = trans.h;

			/**
			 "Normal" XML reply format */
			/*      snprintf(reply, MRL, "<%s range=\"%g\" azimuth=\"%g\" x=\"%g\" y=\"%g\" today=\"true\"/>\n",
			 msg->tag.getTagName(), minRange, minAngle,
			 cos(minAngle * M_PI / 180.0) * minRange,
			 sin(minAngle * M_PI / 180.0) * minRange);*/
			/**
			 SMRDEMO reply format */
			if (msg->client >= 0) { // send to real client only, not a push in the .ini file
				snprintf(reply, MRL,
						"<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n", trans.x,
						trans.y, trans.h);

				// send this string as the reply to the client
				sendMsg(msg, reply);
			}
		} else
			sendWarning(msg, "No scandata available");
	}
	// return true if the function is handled with a positive result
	return true;
}

void UFuncLocalize::projectToLaser(LEL_ARLine worldLine, Matrix<double,3, 1> & pose,
                                   Matrix<double,3, 3> & poseCov, LEL_ARLine &projLine,
                                   Matrix<double,2, 2> &lineCov, Matrix<double,2, 3> & delH_delP) {
  double posex = pose(0, 0);
  double posey = pose(1, 0);
  double poseTh = pose(2, 0);
  double alpha = worldLine.alpha - poseTh;
  projLine.alpha = alpha - lasPose.Kappa;
  projLine.r = worldLine.r -
              (posex * cos(worldLine.alpha) + posey * sin(worldLine.alpha)) -
              (lasPose.x * cos(alpha) + lasPose.y  * sin(alpha));
  if (worldLine.limited) {
    projLine.limited = true;
    double lsrX = posex + lasPose.x * cos(poseTh) - lasPose.y * sin(poseTh);
    double lsrY = posey + lasPose.x * sin(poseTh) + lasPose.y * cos(poseTh);
    double ll = worldLine.positionAlongLine(lsrX, lsrY);
    projLine.lb = worldLine.lb - ll;
    projLine.le = worldLine.le - ll;
  }
  delH_delP << 0, 0, -1,
              -cos(worldLine.alpha),
              -sin(worldLine.alpha),
              -lasPose.x  * sin(alpha) + lasPose.y * cos(alpha);
  lineCov = delH_delP * poseCov * delH_delP.transpose();
}

void UFuncLocalize::updateStatus(double mapX, double mapY, double mapTh,
		UTime time) {
	UPoseTVQ poseTVQ(mapX, mapY, mapTh, time, 0.0, 0.0);
	UResPoseHist * mappose;
	const int LOCALIZE_ID = 15;

	poseTVQ.q = 1.0 - fmin(matchMiss / 100.0, 1.0);
	varFailed->setDouble(matchMiss);
	//
	mappose = (UResPoseHist *) getStaticResource("mapPose", true);
	if (mappose != NULL)
		mappose->addIfNeeded(poseTVQ, LOCALIZE_ID);
	//
	updateCovStatus();
}

void UFuncLocalize::updateCovStatus() {
	UMatrix4 mc, mv(2, 2), ma;
	bool isCompl;
	//
	varCovar->setValueM(poseCov.rows(), poseCov.cols(), &poseCov(0, 0));
	mc.setMat(poseCov.rows(), poseCov.cols(), &poseCov(0, 0));
	// mc.print("Covar as matrix");
	// get eigenvector/eigenvalues for x,y part of matrix
	ma = mc.eig2x2(&isCompl, &mv);
	// set covariance vector
	varCovarV->setValueM(mv.rows(), mv.cols(), mv.getData());
	// expand values to include heading deviation
	ma.setSize(3, 1);
	// convert to deviation values and degrees
	ma.set(sqrt(ma.get(0, 0)), sqrt(ma.get(1, 0)), sqrt(mc.get(2, 2)) * 180.0
			/ M_PI);
	// set to global variable
	varCovarA->setValueM(ma.rows(), ma.cols(), ma.getData());
	//
}

bool UFuncLocalize::handleLocalize(UServerInMsg * msg, void * extra) { // handle a plugin command

  string const nameFile("/home/smr/k388/sim/localize_pos.txt");
  ofstream myFlux(nameFile.c_str());

  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  const int MVL = 30;
  char value[MVL];
  ULaserData * data;
  int match = 0;
  bool aList = false, getonly = false;
  UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
  UPolyItem * poly;
  //
  int i;
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  aList = msg->tag.getAttValue("list", value, MVL);
  getonly = msg->tag.getAttValue("getonly", value, MVL);
  msg->tag.getAttBool("silent", &silent, true);
  if (ask4help) { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "LOCALIZE");
    sendText("--- localize options:\n");
    sendText("list             list loaded lines (no localize performed)\n");
    sendText("silent[=false]   0=verbose, 1=Print only match/no match to console, 2=nothing\n");
    sendText("getonly          Return MRC laservariables only (no localize performed)\n");
    sendText("--- Description of localize command ---\n");
    sendText(" localize first extracts lines from the most recent laser scan ");
    sendText("and uses that combined with a map of lines to estimate the current ");
    sendText("robot pose in the map frame. It has two output channels. First, it ");
    sendText("sends the map (world) origin in the odometry frame through the mrc ");
    sendText("variables $l0 $l1 $l2 (x y th).\n");
    sendText(" Second, it sends pose-time information ");
    sendText("to \"mappose\" whenever it's called.\n\n");
    sendText("--- Usage ---\n");
    sendText("LOCALIZE requires the following to have been called:\n");
    sendText("SETINITPOSE - to set the initial pose\n");
    sendText("SETINITCOV  - to set the initial pose covariance\n");
    sendText("ADDLINE     - to add lines to the map\n");
    sendText("Other update parameters see: 'VAR LOCALIZE'\n");
    sendHelpDone(msg);
  } else if (aList) {
    list<LEL_ARLine>::iterator itWrld;
    int n;
    //
    n = 0;
    snprintf(reply, MRL, "List of loaded %d localizer line(s)",
        (int) lineList.size());
    sendHelpStart(reply);
    for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++) {
      if (itWrld->limited) {
        snprintf(
            reply,
            MRL,
            "   %d a=%8.5frad r=%7.4fm (%7.3fx,%7.3fy to %7.3fx,%7.3fy) %s\n",
            n, itWrld->alpha, itWrld->r, itWrld->p1[0],
            itWrld->p1[1], itWrld->p2[0], itWrld->p2[1],
            itWrld->name);
        //printf("%s", reply);
      } else
        snprintf(reply, MRL, "   %d a=%8.5frad r=%7.4fm (unlimited)\n",
            n, itWrld->alpha, itWrld->r);
      sendText(reply);
      n++;
    }
    sendHelpDone();
  }
  //Only return laservariables without running localizer
  else if (getonly) {
    snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n",
        transX, transY, transTh);
    // send this string as the reply to the client
    sendMsg(msg, reply);

  } else { // do some action and send a reply
    data = getScan(msg, (ULaserData*) extra);
    //
    if (data->isValid()) { // make analysis for closest measurement
      ULaserDevice * device = getDevice(msg, data);
      lasPose = device->getDevicePose();

      //		lsrRelx=lasPose->x;
      //		lsrRely=lasPose->y;
      //		lsrRelTh=0;

      UPose uNewPose = poseHist->getPoseAtTime(data->getScanTime());
      UTime scanTime = data->getScanTime();

      if (lastScanTime.getDecSec() == -1) {
        if (not silent)
          cout << "processingFirstScan\n";
      } else {
        if (not silent)
        {
          UPose lastPose = poseHist->getPoseAtTime(data->getScanTime());
          double dist = uNewPose.getDistance(&lastPose);
          printf("Pose preupd %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh - moved %.3f m\n",
              pose(0,0), pose(1,0), pose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2), dist);

        }
        /// update since last localize
        updateDisplacement(lastScanTime, pose, poseCov, scanTime,
            poseHist, silent, varOdoB->getValued(),
            varKR->getValued(), varKL->getValued());

        //			UPoseTVQ uPoseTime1;
        //			UPoseTVQ uPoseTime2;
        //			poseHist->getPoseNearTime(lastScanTime,&uPoseTime1,&uPoseTime2);
        //			UPose P1 = uPoseTime1.getPose();
        //			//printf("scanTime:%f\n", scanTime);
        //			while (1) {
        //				if (poseHist->getNewest(0).t==uPoseTime1.t) {
        //					if (silent == 0)
        //						cout << "\tpose list end\n";
        //					break;
        //				}
        //				if (not poseHist->getPoseNearTime(uPoseTime1.t+0.001,&uPoseTime1,&uPoseTime2))
        //					break;
        //				UPose P2 = uPoseTime2.getPose();
        //				//cout << "\tPose Time Diff:" << uPoseTime2.t.getDecSec()- scanTime << "\n";
        //				if (uPoseTime2.t > (scanTime + 0.001)) {
        //					//cout << "\time at scan reached\n";
        //					break;
        //				}
        //				double D, L, delSr, delSl;
        //				D = fmod(P2.h - P1.h + 3 * M_PI, 2 * M_PI) - M_PI;
        //				double xDif = (P2.x - P1.x);
        //				double yDif = (P2.y - P1.y);
        //				L = sqrt( xDif * xDif +
        //						yDif * yDif)*((cos(P1.h)*xDif+sin(P1.h)*yDif>0)*2-1);
        //				//cout << "\tD:" << D << " L:" << L << "\n";
        //
        //				double odoB = varOdoB->getValued();
        //				delSr = L + odoB*D/2;
        //				delSl = L - odoB*D/2;
        //
        //				double th_ = pose(2,0)+D/2;
        //
        //				double thd = P2.h-P1.h;
        //				double xd = cos(P1.h)*(P2.x-P1.x)+sin(P1.h)*(P2.y-P1.y);
        //				double yd = -sin(P1.h)*(P2.x-P1.x)+cos(P1.h)*(P2.y-P1.y);
        //				double thl = pose(2,0);
        //
        //				Matrix<double,3,1> poseDif;
        //				poseDif = cos(thl)*xd-sin(thl)*yd,sin(thl)*xd+cos(thl)*yd,thd;
        //				pose+=poseDif;
        //
        //				Matrix<double,3,2> delPnew_delY;
        //				delPnew_delY=cos(th_)/2-L*sin(th_)/(2*odoB),    cos(th_)/2+L*sin(th_)/(2*odoB),
        //				             sin(th_)/2+L*cos(th_)/(2*odoB),    sin(th_)/2-L*cos(th_)/(2*odoB),
        //				             1/odoB,                        	-1/odoB;
        //
        //				Matrix<double,3,3> delPnew_delPold;
        //				delPnew_delPold=  1,    0,      -L*sin(th_),
        //								  0,    1,      L*cos(th_),
        //								  0,    0,      1;
        //
        //				Matrix<double,2,2> covU;
        //				covU= sqr(varKR->getValued()) * fabs(delSr),    0,
        //				      0                                    ,    sqr(varKL->getValued()) * fabs(delSl);
        //
        //				//printf("Robot %gbase l=%g R=%g\b", odoB, varKL->getValued(), varKR->getValued());
        //
        //				//covOut = delPnew_delPold*covIn*delPnew_delPold'+delPnew_delY*covU*delPnew_delY';*/
        //
        //				poseCov = delPnew_delY *covU * delPnew_delY.transpose()
        //						  +delPnew_delPold * poseCov * delPnew_delPold.transpose();
        //
        //				uPoseTime1 = uPoseTime2;
        //				P1 = P2;
        //				poseIndex++;
        //
        //			}
        //
        //			if (silent == 0)
        //				cout << "\tupdated displacement\n";
      }

      double devThreshold = 3 * 3;
      double pointsInThreshold = varPointsInThreshold->getDouble();
      double minRange = 0.05;

      const int MAXNOOFSCANPOINTS = 2000;
      double X[MAXNOOFSCANPOINTS];
      double Y[MAXNOOFSCANPOINTS];

      if (data->getRangeCnt() > MAXNOOFSCANPOINTS)
        printf(
            "UFUNCLOCALIZE::HANDLELOCALIZEUKF: Warning, num of scan points larger that the specified maximum");

      int dataI;
      for (i = 0, dataI = 0; i < data->getRangeCnt(); i++) { // range are stored as an integer in current units
        bool rangeValid;
        double r = data->getRangeMeter(i, &rangeValid);
        if (!rangeValid || r < minRange)
          continue;
        double alpha = data->getAngleRad(i);
        X[dataI] = r * cos(alpha);
        Y[dataI] = r * sin(alpha);
        dataI++;
      }

      list<LEL_GFLine> GFLL;
      ransac(X, Y, dataI, GFLL);

      list<LEL_GFLine>::iterator itLas;

      for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++) {
        LEL_ARLine line = (*itLas).toARLine();
        if (false and not silent)
          cout << "\nline params:" << line.alpha << ", " << line.r;
      }

      list<LEL_ARLine>::iterator itWrld;
      if (not silent) {
// 				cout
// 				<< "\nprinting pose and poseCov before measurement update\n";
// 				cout << pose;
// 				cout << poseCov << "\n";
        printf("Pose before %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh\n",
              pose(0,0), pose(1,0), pose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2));
      }
      Matrix<double,3, 1> newPose = pose;
      LEL_ARLine bestLine;
      Matrix<double,2, 3> delH_delPMin;
      Matrix<double,2, 1> lineDifMin;
      Matrix<double,2, 2> lineDifCovMin;
      double varAlpha = varLaserAlpha->getValued();
      double varR = varLaserR->getValued();
      for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++)
      { // remove matched lines
        if (not silent)
        {
          poly = resPoly->getItem(itWrld->name);
          if (poly != NULL)
          {
            poly->lock();
            // remove old polygon - set to one point
            poly->setPointsCnt(1);
            poly->setColor("b1dd");
            poly->setUpdated();
            poly->unlock();
            resPoly->gotNewData();
          }
        }
        LEL_ARLine projLine;
        Matrix<double,2, 3> delH_delP;
        Matrix<double,2, 2> lineDifCov;
        UPosition pos1, pos2, pe1, pe2;
        double lmin, lmax, difA, difR;
        double angleDif;
        projectToLaser(*itWrld, pose, poseCov, projLine, lineDifCov,
            delH_delP);
        lineDifCov(0, 0) += varAlpha;
        lineDifCov(1, 1) += varR;
        double minMahDist = devThreshold;
        //debug
        minMahDist = 1e3;
        // debug end
        LEL_GFLine closestLine;
        for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++) {
          LEL_ARLine line = (*itLas).toARLine();
          angleDif = fmod(line.alpha - projLine.alpha + 3
              * M_PI, 2 * M_PI) - M_PI;
          double rDif;
          if (fabs(angleDif) > M_PI / 2) {
            angleDif = fmod(angleDif + 2 * M_PI, 2 * M_PI) - M_PI;
            rDif = -line.r - projLine.r;
          } else
            rDif = line.r - projLine.r;
          Matrix<double,2, 1> lineDif;
          lineDif << angleDif, rDif;
          double mahDist = lineDif.transpose()
                  * lineDifCov.inverse() * lineDif;
          if (mahDist < minMahDist) {
            if (projLine.limited) {
              int pointsIn = 0;
              for (int p = 0; p < itLas->edgeCount; p++) {
                double x = itLas->edgesX[p];
                double y = itLas->edgesY[p];
                double lp = projLine.positionAlongLine(x, y);
                if (p == 0)
                {
                  lmin = lp;
                  lmax = lp;
                  pos1.set(x,y);
                  pos2.set(x,y);
                }
                else if (lp < lmin)
                {
                  lmin = lp;
                  pos1.set(x,y);
                }
                else if (lp > lmax)
                { 
                  lmax = lp;
                  pos2.set(x,y);
                }
                if (projLine.lb < lp and lp < projLine.le)
                  pointsIn++;
              }
              if (pointsIn < pointsInThreshold * itLas->edgeCount) {
                // cout<<"Match ruled out due to line limits mismatch\n";
                continue;
              }
            }
            minMahDist = mahDist;
            delH_delPMin = delH_delP;
            lineDifMin = lineDif;
            lineDifCovMin = lineDifCov;
            closestLine = *itLas;
            bestLine = *itWrld;
            pe1 = pos1;
            pe2 = pos2;
            difR = rDif;
            difA = angleDif;
          }
        }
        if (not silent and minMahDist < 1e3)
        { // print line correlation result
          char c = ' ';
          if (minMahDist < devThreshold)
            c = '*';
          printf("Line %s (%.2fx,%.2fy to %.2fx,%.2fy) <-> (%.2fx,%.2fy to %.2fx,%.2fy) a-dif %.4f, R-dif %.4f, maha dist %.3f %c\n",
                  bestLine.name, pe1.x, pe1.y, pe2.x, pe2.y,
                  bestLine.p1[0], bestLine.p1[1], bestLine.p2[0], bestLine.p2[1], difA, difR, sqrt(minMahDist), c);
          // cout << "Localizer: Line match to " << bestLine.name  << "\n";
          if (resPoly != NULL and minMahDist < devThreshold)
          {
            poly = resPoly->add(bestLine.name, bestLine.p1[0], bestLine.p1[1]);
            poly->lock();
            poly->cooSys = 2;
            poly->setColor("r3dd");
            poly->setAsPolyline();
            poly = resPoly->add(bestLine.name, bestLine.p2[0], bestLine.p2[1]);
            poly->setUpdated();
            resPoly->gotNewData();
            poly->unlock();
          }
        }
        if (minMahDist < devThreshold) {
          // We have a match!
          Matrix<double,3, 1> poseDif = newPose - pose;

          Matrix<double,2, 1> v = lineDifMin - (delH_delPMin * poseDif);

          Matrix<double,3, 2> K = poseCov * delH_delPMin.transpose()
                  * lineDifCovMin.inverse();
          //      		  cout<<"\n printing K:\n";
          //      		  mprint(K);

          newPose += K * v;

          poseCov -= K * lineDifCovMin * K.transpose();
          match++;
          ;
          if (not silent) {
            printf("PoseDif %.3fx, %.3fy, %.3frad\n", poseDif(0,0), poseDif(1,0), poseDif(2,0));
            printf("LineDif %.3fx, %.3fy\n", lineDifMin(0,0), lineDifMin(1,0));
            printf("  ldCov %.6f, %.6f \n", lineDifCovMin(0,0), lineDifCovMin(0,1));
            printf("  ldCov %.6f, %.6f \n", lineDifCovMin(1,0), lineDifCovMin(1,1));
            printf("   dHdP %.6f, %.6f %.6f\n", delH_delPMin(0,0), delH_delPMin(0,1), delH_delPMin(0,2));
            printf("   dHdP %.6f, %.6f %.6f\n", delH_delPMin(1,0), delH_delPMin(1,1), delH_delPMin(1,2));
            printf("      v %.3fx, %.3fy\n", v(0,0), v(1,0));
            printf("      k %.6f, %.6f \n", K(0,0), K(0,1));
            printf("      k %.6f, %.6f \n", K(1,0), K(1,1));
            printf("      k %.6f, %.6f \n", K(2,0), K(2,1));
            printf("poseCov %.6f, %.6f %.6f\n", poseCov(0,0), poseCov(0,1), poseCov(0,2));
            printf("poseCov %.6f, %.6f %.6f\n", poseCov(1,0), poseCov(1,1), poseCov(1,2));
            printf("poseCov %.6f, %.6f %.6f\n", poseCov(2,0), poseCov(2,1), poseCov(2,2));
            printf("Pose after match %d  %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh\n",
                  match, newPose(0,0), newPose(1,0), newPose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2));
          }
        }
      }

      pose = newPose;
      //cout<<poseCov;
      if (not silent) {
        printf("Pose after  %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh\n",
              pose(0,0), pose(1,0), pose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2));
// 				cout
// 				<< "\nprinting pose and poseCov after measurement update\n";
// 				cout << pose;
// 				cout.setf(std::ios_base::scientific);
// 				cout << "\n Pose Covariance: \n";
// 				cout << poseCov << "\n";


         if(myFlux)
           myFlux << pose(0,0) << " " << pose(1,0) << " " << pose(2,0);

      }
      cout.setf(std::ios_base::fixed);

      UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());

      UPose trans;

      double poseX = pose(0, 0);
      double poseY = pose(1, 0);
      double poseTh = pose(2, 0);

      trans.h = poseAtScan.h - poseTh;
      trans.x = poseAtScan.x - (cos(trans.h) * poseX - sin(trans.h)
          * poseY);
      trans.y = poseAtScan.y - (sin(trans.h) * poseX + cos(trans.h)
          * poseY);

      if (match > 0)
        matchMiss = 0;
      else
        matchMiss += 1;
      if (not silent)
        printf("Localizer: got %d line matches\n", match);
      if (match)
        varUpdates->add(1.0);
      //
      // update map-pose and global data
      updateStatus(poseX, poseY, poseTh, data->getScanTime());
      //
      lastScanTime = scanTime;

      //Save the trans variables to class-variables
      transX = trans.x;
      transY = trans.y;
      transTh = trans.h;

      /**
      "Normal" XML reply format */
      /*      snprintf(reply, MRL, "<%s range=\"%g\" azimuth=\"%g\" x=\"%g\" y=\"%g\" today=\"true\"/>\n",
      msg->tag.getTagName(), minRange, minAngle,
      cos(minAngle * M_PI / 180.0) * minRange,
      sin(minAngle * M_PI / 180.0) * minRange);*/
      /**
      SMRDEMO reply format */
      if (msg->client >= 0) { // send to real client only, not a push in the .ini file
        snprintf(reply, MRL,
            "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n", trans.x,
            trans.y, trans.h);

        // send this string as the reply to the client
        sendMsg(msg, reply);
      }
    } else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  return true;
}

