#ifndef __MAV_MESSAGE_H
#define __MAV_MESSAGE_H
#include <valarray>
/*!
	\file MavMessage.h
	\brief Header of the mav message entity 
*/
namespace Enki
{
	//! A simple model of message sent from one mav to another.
	class MavMessage
	{
	protected:
		//! The id of the sender mav
		int id;
		//! The number of mavs that can communicate with the sender mav
		int density;
		//! The global track of the sender mav
		double orientation;
		//! True if the sender mav is orbitting, false otherwise
		bool orbitting;
		//! An arbitrary message
		char *msg;
		//! The mav that is currently being saved by the sender mav
		int savedMav;
		//! The number of hops from the base station to the mav
		int hopCount;
		//! The number of hops from the user station to the mav
		int bHopCount;
		//! The number of hops from the base station to the user station
		int minHopCount;
		double pheromoneLevel;
		double pheromoneLeft;
		double pheromoneRight;
		double pheromoneStraight;
		double pheromoneLeftB;
		double pheromoneRightB;
		double pheromoneStraightB;
		int side;
		int sideb;
		int state;
		int targetId;
		bool atNode;
	public:
		//! Create a message containing all the information to be sent.
		MavMessage(int id,int density, double orientation, bool orbitting, char *msg, int savedMav, int hopCount);
		//! Create a message containing all the information to be sent.
		MavMessage(int id,int density, double orientation, bool orbitting, char *msg, int savedMav, int hopCount,int bHopCount);
		//! Create a message containing all the information to be sent.
		MavMessage(int id,int density, double orientation,  char *msg, int hopCount, int bHopCount, int minHopCount);
		//! Create a message containing all the information to be sent.
		MavMessage(int id,int density, double orientation, int hopCount, int bHopCount);
		////! Create a message containing all the information to be sent.
		MavMessage(int id,int density, double orientation, int hopCount);
		//! Create a message containing all the information to be sent.
		MavMessage(int id,int density, double orientation);

		MavMessage(int id, int density, int side, int sideb, int state, double pheromoneLeft, double pheromoneStraight, double pheromoneRight,double pheromoneLeftB, double pheromoneStraightB, double pheromoneRightB, int targetId, bool atNode);
		//! The id of the sender mav
		void setId(int id);
		//! The number of mavs that can communicate with the sender mav
		void setDensity(int density);
		//! The global track of the sender mav
		void setOrientation(double orientation);
		//! An arbitrary message relevant to the different algorithms used
		void setMsg(char *msg);
		//! The mav that is currently being saved by the  sender mav
		void setSavedMav(int id);
		//! True if the sender mav is orbitting, false otherwise
		void setOrbitting(bool orbitting);
		//! The number of hops from the base station to the mav
		void setHopCount(int hopCount);
		//! The number of hops from the user station to the mav
		void setBHopCount(int bHopCount);
		//! The number of hops from the base station to the user station
		void setMinHopCount(int minHopCount);
		//! Returns the id of the sender mav
		int getId();
		//! Returns the number of mavs that can communicate with the sender mav
		int getDensity();
		//! Returns the global track of the sender mav
		double getOrientation();
		//! True if the sender mav is orbitting, false otherwise
		bool isOrbitting();
		//! Returns the message
		char *getMsg();
		//! Returns the mav that is currently being saved by the sender mav
		int getSavedMav();
		//! Returns the number of hops from the base station to the mav
		int getHopCount();
		//! Returns the number of hops from the user station to the mav
		int getBHopCount();
		//! Returns the number of hops from the base station to the user station
		int getMinHopCount();
		double getPheromoneLevel();
		int getSide();
		int getSideB();
		int getTargetId();
		int getState();
		void setPheromoneLevel(double pheromoneLevel);
		void setPheromoneLeft(double pheromoneLevel);
		void setPheromoneRight(double pheromoneLevel);
		void setPheromoneStraight(double pheromoneLevel);
		void setPheromoneLeftB(double pheromoneLevel);
		void setPheromoneRightB(double pheromoneLevel);
		void setPheromoneStraightB(double pheromoneLevel);
		double getPheromoneLeft();
		double getPheromoneRight();
		double getPheromoneStraight();
		double getPheromoneLeftB();
		double getPheromoneRightB();
		double getPheromoneStraightB();
		void setSide(int side);
		void setSideB(int sideb);
		void setState(int state);
		void setTargetId(int targetId);
		void setAtNode(bool atNode);
		bool getAtNode();
	};
}

#endif

