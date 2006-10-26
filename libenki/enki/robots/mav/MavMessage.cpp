#include "MavMessage.h"

/*!
	\file MavMessage.h
	\brief Implementation of the mav message entity 
*/
namespace Enki
{
	MavMessage::MavMessage(int id,int density, double orientation, bool orbitting, char *msg, int savedMav, int hopCount)
	{
		this->id = id;
		this->density = density;
		this->orientation = orientation;
		this->orbitting=orbitting;
		this->msg=msg;
		this->savedMav=savedMav; 
		this->hopCount=hopCount;
	}
	MavMessage::MavMessage(int id,int density, double orientation, bool orbitting, char *msg, int savedMav, int hopCount, int bHopCount)
	{
		this->id = id;
		this->density = density;
		this->orientation = orientation;
		this->orbitting=orbitting;
		this->msg=msg;
		this->savedMav=savedMav; 
		this->hopCount=hopCount;
		this->bHopCount=bHopCount;
	}
	MavMessage::MavMessage(int id,int density, double orientation, int hopCount,int bHopCount)
	{
		this->id = id;
		this->density = density;
		this->orientation = orientation;
		this->hopCount=hopCount;
		this->bHopCount=bHopCount;
	}
	MavMessage::MavMessage(int id,int density, double orientation,char *msg, int hopCount,int bHopCount,int minHopCount)
	{
		this->id = id;
		this->density = density;
		this->orientation = orientation;
		this->msg=msg;
		this->hopCount=hopCount;
		this->bHopCount=bHopCount;
		this->minHopCount=minHopCount;
	}
	MavMessage::MavMessage(int id,int density, double orientation, int hopCount)
	{
		this->id = id;
		this->density = density;
		this->orientation = orientation;
		this->orbitting=orbitting;
		this->hopCount=hopCount;
	}
	MavMessage::MavMessage(int id,int density, double orientation)
	{
		this->id = id;
		this->density = density;
		this->orientation = orientation;
	}
	MavMessage::MavMessage(int id, int density, int side, int sideb, int state, double pheromoneLeft, double pheromoneStraight, double pheromoneRight,double pheromoneLeftB, double pheromoneStraightB, double pheromoneRightB, int targetId, bool atNode)
	{
		this->id = id;
		this->density=density;
		this->pheromoneLeft=pheromoneLeft;
		this->pheromoneRight=pheromoneRight;
		this->pheromoneStraight=pheromoneStraight;
		this->pheromoneLeftB=pheromoneLeftB;
		this->pheromoneRightB=pheromoneRightB;
		this->pheromoneStraightB=pheromoneStraightB;
		this->state=state;	
		this->targetId=targetId;
		this->sideb=sideb;
		this->side=side;
		this->atNode=atNode;
	}
	void  MavMessage::setId(int id)
	{
		this->id=id;
	}
	void  MavMessage::setDensity(int density)
	{
		this->density=density;
	}
	void  MavMessage::setOrientation(double orientation)
	{
		this->orientation=orientation;
	}
	void  MavMessage::setMsg(char *msg)
	{
		this->msg=msg;
	}
	char *MavMessage::getMsg()
	{
		return msg;
	}
	void MavMessage::setSavedMav(int id)
	{
		this->savedMav=id;  
	}
	
	void MavMessage::setHopCount(int hopCount)
	{
		this->hopCount=hopCount;
	}
	void MavMessage::setBHopCount(int bHopCount)
	{
		this->bHopCount=bHopCount;
	}
	void MavMessage::setMinHopCount(int minHopCount)
	{
		this->minHopCount=minHopCount;
	}
	int  MavMessage::getId()
	{
		return id;
	}
	int  MavMessage::getDensity()
	{
		return density;
	}
	double  MavMessage::getOrientation()
	{
		return orientation;
	}
	
	bool  MavMessage::isOrbitting()
	{
		return orbitting;
	}
	void MavMessage::setOrbitting(bool orbitting)
	{
		this->orbitting=orbitting;
	}
	int  MavMessage::getSavedMav()
	{
		return savedMav;
	}
	int  MavMessage::getHopCount()
	{
		return hopCount;
	}
	int  MavMessage::getBHopCount()
	{
		return bHopCount;
	}
	int  MavMessage::getMinHopCount()
	{
		return minHopCount;
	}
	double  MavMessage::getPheromoneLevel()
	{
		return pheromoneLevel;
	}
	double  MavMessage::getPheromoneLeft()
	{
		return pheromoneLeft;
	}
	double  MavMessage::getPheromoneRight()
	{
		return pheromoneRight;
	}
	double  MavMessage::getPheromoneStraight()
	{
		return pheromoneStraight;
	}
	double  MavMessage::getPheromoneLeftB()
	{
		return pheromoneLeftB;
	}
	double  MavMessage::getPheromoneRightB()
	{
		return pheromoneRightB;
	}
	double  MavMessage::getPheromoneStraightB()
	{
		return pheromoneStraightB;
	}
	int  MavMessage::getSide()
	{
		return side;
	}
	int  MavMessage::getSideB()
	{
		return sideb;
	}
	int  MavMessage::getTargetId()
	{
		return targetId;
	}
	int  MavMessage::getState()
	{
		return state;
	}
	void MavMessage::setPheromoneLevel(double pheromoneLevel)
	{
		this->pheromoneLevel=pheromoneLevel;
	}
	void MavMessage::setPheromoneLeft(double pheromoneLevel)
	{
		this->pheromoneLeft=pheromoneLevel;
	}
	void MavMessage::setPheromoneRight(double pheromoneLevel)
	{
		this->pheromoneRight=pheromoneLevel;
	}
	void MavMessage::setPheromoneStraight(double pheromoneLevel)
	{
		this->pheromoneStraight=pheromoneLevel;
	}
	void MavMessage::setPheromoneLeftB(double pheromoneLevel)
	{
		this->pheromoneLeftB=pheromoneLevel;
	}
	void MavMessage::setPheromoneRightB(double pheromoneLevel)
	{
		this->pheromoneRightB=pheromoneLevel;
	}
	void MavMessage::setPheromoneStraightB(double pheromoneLevel)
	{
		this->pheromoneStraightB=pheromoneLevel;
	}
	void  MavMessage::setSide(int side)
	{
		this->side=side;
	}
	void  MavMessage::setSideB(int sideb)
	{
		this->sideb=sideb;
	}
	void  MavMessage::setState(int state)
	{
		this->state=state;
	}
	void  MavMessage::setTargetId(int targetId)
	{
		this->targetId=targetId;
	}
	void  MavMessage::setAtNode(bool atNode)
	{
		this->atNode=atNode;
	}
	bool MavMessage::getAtNode()
	{
		return atNode;
	}
}
