/*
  Enki - a fast 2D robot simulator
  Copyright © 2017 Nicolas Palard <nicolas.palard@etu.u-bordeaux.fr>
  Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <enki/Serialize.h>
#include <enki/robots/thymio2/Thymio2.h>
#include "WorldGenerator.h"
#include <chrono>

/*
	This test aims to provide a simple way to validate our protocol in terms of
	execution time and size of the generated data.

	To test a functionality, you need to implement two methods :
	serializeXXX(void* XXX) and deserializeXXX(void* XXX).

	In order to understand how you can implement these methods you can look
	at the serializeThymio(void* t) and deserializeThymio(void* t) functions.

	Once you've implemented the methods you can now call them with the main loop
	that will compute the stats for you : serializeIt(functionName, argument,
	iterationNumber) and deserializeIt(functionName, argument, iterationNumber)
*/

using namespace Enki;
using namespace std;

const int ITERATION_NUMBER = 100;

typedef chrono::system_clock::time_point timepoint;

struct perfs
{
	double average = 0.0;
	double max = 0.0;
	double min = 99999.99;
	bool isUsed = false;
};

struct stats
{
	perfs time;
	perfs size;
};

static void printSeparator(char c, int size)
{
	for (int i = 0; i < size; i++)
		cerr << c;
	cerr << endl;
}

void dumpTimeStats(perfs sTime, string name)
{
	cerr << "Reported TIME stats on " << name << " : " << endl;
	printSeparator('-', 10);
	cerr << "Max " << name << " time (elapsed) : " << sTime.max << "ms" << endl;
	cerr << "Min " << name << " time (elapsed) : " << sTime.min << "ms" << endl;
	cerr << "Average " << name << " time (elapsed) : " << sTime.average << "ms" << endl;
}

void dumpSizeStats(perfs sSize, string name)
{
	cerr << "Reported SIZE stats on " << name << " :" << endl;
	printSeparator('-', 10);
	cerr << "Max " << name << " size : " << sSize.max
		 << " bytes (" << sSize.max / 1000.0 << "kB)" << endl;
	cerr << "Min " << name << " size : " << sSize.min
		 << " bytes (" << sSize.min / 1000.0 << "kB)" << endl;
	cerr << "Average " << name << " size : " << sSize.average
		 << " bytes (" << sSize.average / 1000.0 << "kB)" << endl;
}

void dumpStats(stats s, string name)
{
	if (s.time.isUsed)
	{
		dumpTimeStats(s.time, name);
		printSeparator('=', 30);
	}
	if (s.size.isUsed)
	{
		dumpSizeStats(s.size, name);
		printSeparator('=', 30);
	}
}

// Return the elapsed time in milliseconds between two timepoints objects.
double msTime(timepoint start, timepoint end)
{
	return chrono::duration_cast<chrono::nanoseconds>(end - start).count() / 1000000.0;
}

// Update the perfs structure with a given value
void updateStats(perfs* p, double value)
{
	if (p->max < value)
	{
		p->max = value;
	}
	else if (p->min > value)
	{
		p->min = value;
	}
	p->average += value;
	p->isUsed = true;
}

// Serialization functions need to be tested in terms of executionTime but also
// in term of size of the generated data.
stats serializeIt(double (*func)(void*), void* param, int iteration)
{
	stats stats;
	for (int i = 0; i < iteration; i++)
	{
		timepoint start = chrono::system_clock::now();
		double dataSize = func(param);
		timepoint end = chrono::system_clock::now();

		double executionTime = msTime(start, end);
		updateStats(&stats.time, executionTime);
		updateStats(&stats.size, dataSize);
	}

	stats.time.average /= iteration;
	stats.size.average /= iteration;

	return stats;
}

double globalSerialization(void* w)
{
	World* world = (World*) w;
	string data = serialize(world);

	return data.size();
}

// Consider putting the outputstream as a global variable in order to remove
// the time wasted to create the objects from the computations. This also means
// clear the stream in serializeIt() at each call.
double thymioSerialization(void* t)
{
	Thymio2* thymio = (Thymio2*) t;
	ostringstream output;
	serializeThymio(thymio, output);

	return output.str().size();
}

double colorSerialization(void* c)
{
	Color* color = (Color*) c;
	ostringstream output;
	serializeColor(*color, output);

	return output.str().size();
}

// Measure the executing time of FUNC function pointer called
// with PARAM
stats deserializeIt(double (*func) (void*), void* param, int iteration)
{
	stats stats;
	for (int i = 0; i < iteration; i++)
	{
		double executionTime = func(param);
		updateStats(&stats.time, executionTime);
	}
	stats.time.average /= iteration;

	return stats;
}

double globalDeserialization(void* w)
{
	string data = serialize((World*) w);

	timepoint start = chrono::system_clock::now();
	deserialize(data);
	timepoint end = chrono::system_clock::now();

	return msTime(start, end);
}

double thymioDeserialization(void* t)
{
	ostringstream output;
	serializeThymio((Thymio2*) t, output);

	timepoint start = chrono::system_clock::now();
	deserializeThymio(output.str());
	timepoint end = chrono::system_clock::now();

	return msTime(start, end);
}

double colorDeserialization(void* c)
{
	ostringstream output;
	serializeColor(*((Color*)c), output);

	timepoint start = chrono::system_clock::now();
	deserializeColor(output.str());
	timepoint end = chrono::system_clock::now();

	return msTime(start, end);
}

int main()
{
	WorldGenerator gen; gen.add(Randomizer::THYMIO2_, 10);
	World* w = gen.getWorld();
	stats sWorld = serializeIt(globalSerialization, (void*)w, ITERATION_NUMBER);
	dumpStats(sWorld, "global serialization");

	Thymio2* t = gen.getRandomizer()->randThymio();
	stats sThymio = serializeIt(thymioSerialization, (void*)t, ITERATION_NUMBER);
	dumpStats(sThymio, "thymio serialization");

	Color c = gen.getRandomizer()->randColor();
	stats sColor = serializeIt(colorSerialization, (void*)&c, ITERATION_NUMBER);
	dumpStats(sColor, "color serialization");

	stats dWorld = deserializeIt(globalDeserialization, (void*)w, ITERATION_NUMBER);
	dumpStats(dWorld, "global deserialization");

	stats dThymio = deserializeIt(thymioDeserialization, (void*)t, ITERATION_NUMBER);
	dumpStats(dThymio, "thymio deserialization");

	stats dColor = deserializeIt(colorDeserialization, (void*)&c, ITERATION_NUMBER);
	dumpStats(dColor, "color deserialization");
}
