/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/return_value_policy.hpp>
#include "../enki/Types.h"
#include "../enki/Geometry.h"
#include "../enki/PhysicalEngine.h"
#include "../enki/robots/e-puck/EPuck.h"
#include "../enki/robots/thymio2/Thymio2.h"
#include "../viewer/Viewer.h"
#include <QApplication>
#include <QImage>
#include <QGLWidget>

using namespace boost::python;
using namespace Enki;

tuple getColorComponents(const Color& color)
{
	return make_tuple(
		color.components[0],
		color.components[1],
		color.components[2],
		color.components[3]
	);
}

void setColorComponents(Color& color, tuple values)
{
	if (len(values) != 4)
		throw std::runtime_error("Tuple used to set components must be of length 4");
	color.components[0] = extract<double>(values[0]);
	color.components[1] = extract<double>(values[1]);
	color.components[2] = extract<double>(values[2]);
	color.components[3] = extract<double>(values[3]);
}

#define def_readwrite_by_value(name, target) \
	add_property(\
		(name), \
		make_getter((target), return_value_policy<return_by_value>()), \
		make_setter((target), return_value_policy<return_by_value>()) \
	)

// vector convertion

struct Vector_to_python_tuple
{
	static PyObject* convert(const Vector& value)
	{
		return incref(make_tuple(value.x, value.y).ptr());
	}
};
struct Vector_from_python
{
	Vector_from_python()
	{
		converter::registry::push_back(
			&convertible,
			&construct,
			type_id<Vector>()
		);
	}
	
	static void* convertible(PyObject* objPtr)
	{
		if (PyTuple_Check(objPtr))
		{
			Py_ssize_t l = PyTuple_Size(objPtr);
			if (l != 2)
				return 0;
			
			PyObject* item0(PyTuple_GetItem(objPtr, 0));
			assert (item0);
			if (!(PyFloat_Check(item0) || PyInt_Check(item0)))
				return 0;
			PyObject* item1(PyTuple_GetItem(objPtr, 1));
			assert (item1);
			if (!(PyFloat_Check(item1) || PyInt_Check(item1)))
				return 0;
		}
		else
		{
			Py_ssize_t l = PyObject_Length(objPtr);
			if (l != 2)
				return 0;
			
			PyObject* item0(PyList_GetItem(objPtr, 0));
			assert (item0);
			if (!(PyFloat_Check(item0) || PyInt_Check(item0)))
				return 0;
			PyObject* item1(PyList_GetItem(objPtr, 1));
			assert (item1);
			if (!(PyFloat_Check(item1) || PyInt_Check(item1)))
				return 0;
		}
		
		return objPtr;
	}
	
	static void construct(PyObject* objPtr, converter::rvalue_from_python_stage1_data* data)
	{
		double x,y;
		
		if (PyTuple_Check(objPtr))
		{
			x = PyFloat_AsDouble(PyTuple_GetItem(objPtr, 0));
			y = PyFloat_AsDouble(PyTuple_GetItem(objPtr, 1));
		}
		else
		{
			x = PyFloat_AsDouble(PyList_GetItem(objPtr, 0));
			y = PyFloat_AsDouble(PyList_GetItem(objPtr, 1));
		}
		
		void* storage = ((converter::rvalue_from_python_storage<Vector>*)data)->storage.bytes;
		new (storage) Vector(x,y);
		data->convertible = storage;
	}
};

// wrappers for world

static World::GroundTexture loadTexture(const std::string& fileName)
{
	/*World::GroundTexture t;
	
	std::ifstream ifs(ppmFileName.c_str(), std::ifstream::in);
	if (!ifs.good())
		throw std::runtime_error("Cannot open file " + ppmFileName);
	std::string magic;
	ifs >> magic;
	if (magic != "P3")
		throw std::runtime_error("Not a PPM file: " + ppmFileName);
	ifs >> t.width;
	ifs >> t.height;
	int valuesScale;
	ifs >> valuesScale;
	t.data.reserve(t.width*t.height);
	for (int y = 0; y < t.height; ++y)
	{
		for (int x = 0; x < t.width; ++x)
		{
			unsigned r, g, b;
			ifs >> r >> g >> b;
			if (ifs.eof())
				throw std::runtime_error("Early end-of-file: " + ppmFileName);
			r = (r * 255) / valuesScale;
			g = (g * 255) / valuesScale;
			b = (b * 255) / valuesScale;
			t.data.push_back(r|(g<<8)|(b<<16));
		}
	}
	
	return t;*/
	QImage gt(QGLWidget::convertToGLFormat(QImage(fileName.c_str())));
	
	#if QT_VERSION >= QT_VERSION_CHECK(4,7,0)
	return World::GroundTexture(gt.width(), gt.height(), (const uint32_t*)gt.constBits());
	#else
	return World::GroundTexture(gt.width(), gt.height(), (uint32_t*)gt.bits());
	#endif
}

struct WorldWithoutObjectsOwnership: public World
{
	WorldWithoutObjectsOwnership(double width, double height, const Color& wallsColor = Color::gray, const GroundTexture& groundTexture = GroundTexture()):
		World(width, height, wallsColor, groundTexture)
	{
		takeObjectOwnership = false;
	}
	
	WorldWithoutObjectsOwnership(double r, const Color& wallsColor = Color::gray, const GroundTexture& groundTexture = GroundTexture()):
		World(r, wallsColor, groundTexture)
	{
		takeObjectOwnership = false;
	}
	
	WorldWithoutObjectsOwnership()
	{
		takeObjectOwnership = false;
	}
};

struct WorldWithTexturedGround: public WorldWithoutObjectsOwnership
{
	WorldWithTexturedGround(double width, double height, const std::string& ppmFileName, const Color& wallsColor = Color::gray):
		WorldWithoutObjectsOwnership(width, height, wallsColor, loadTexture(ppmFileName))
	{
	}
	
	WorldWithTexturedGround(double r, const std::string& ppmFileName, const Color& wallsColor = Color::gray):
		WorldWithoutObjectsOwnership(r, wallsColor, loadTexture(ppmFileName))
	{
	}
};

// wrappers for objects

struct CircularPhysicalObject: public PhysicalObject
{
	CircularPhysicalObject(double radius, double height, double mass, const Color& color = Color())
	{
		setCylindric(radius, height, mass);
		setColor(color);
	}
};

struct RectangularPhysicalObject: public PhysicalObject
{
	RectangularPhysicalObject(double l1, double l2, double height, double mass, const Color& color = Color())
	{
		setRectangular(l1, l2, height, mass);
		setColor(color);
	}
};

// wrappers for robots

struct EPuckWrap: EPuck, wrapper<EPuck>
{
	EPuckWrap():
		EPuck(CAPABILITY_BASIC_SENSORS|CAPABILITY_CAMERA)
	{}
	
	virtual void controlStep(double dt)
	{
		if (override controlStep = this->get_override("controlStep"))
			controlStep(dt);
		
		EPuck::controlStep(dt);
	}
	
	list getProxSensorValues(void)
	{
		list l;
		l.append(infraredSensor0.getValue());
		l.append(infraredSensor1.getValue());
		l.append(infraredSensor2.getValue());
		l.append(infraredSensor3.getValue());
		l.append(infraredSensor4.getValue());
		l.append(infraredSensor5.getValue());
		l.append(infraredSensor6.getValue());
		l.append(infraredSensor7.getValue());
		return l;
	}
	
	list getProxSensorDistances(void)
	{
		list l;
		l.append(infraredSensor0.getDist());
		l.append(infraredSensor1.getDist());
		l.append(infraredSensor2.getDist());
		l.append(infraredSensor3.getDist());
		l.append(infraredSensor4.getDist());
		l.append(infraredSensor5.getDist());
		l.append(infraredSensor6.getDist());
		l.append(infraredSensor7.getDist());
		return l;
	}
	
	Texture getCameraImage(void)
	{
		Texture texture;
		texture.reserve(camera.image.size());
		for (size_t i = 0; i < camera.image.size(); ++i)
			texture.push_back(camera.image[i]);
		return texture;
	}
};

struct Thymio2Wrap: Thymio2, wrapper<Thymio2>
{
	virtual void controlStep(double dt)
	{
		if (override controlStep = this->get_override("controlStep"))
			controlStep(dt);
		
		Thymio2::controlStep(dt);
	}
	
	list getProxSensorValues(void)
	{
		list l;
		l.append(infraredSensor0.getValue());
		l.append(infraredSensor1.getValue());
		l.append(infraredSensor2.getValue());
		l.append(infraredSensor3.getValue());
		l.append(infraredSensor4.getValue());
		l.append(infraredSensor5.getValue());
		l.append(infraredSensor6.getValue());
		return l;
	}
	
	list getProxSensorDistances(void)
	{
		list l;
		l.append(infraredSensor0.getDist());
		l.append(infraredSensor1.getDist());
		l.append(infraredSensor2.getDist());
		l.append(infraredSensor3.getDist());
		l.append(infraredSensor4.getDist());
		l.append(infraredSensor5.getDist());
		l.append(infraredSensor6.getDist());
		return l;
	}
	
	list getGroundSensorValues(void)
	{
		list l;
		l.append(groundSensor0.getValue());
		l.append(groundSensor1.getValue());
		return l;
	}
};

struct PythonViewer: public ViewerWidget
{
	PyThreadState *pythonSavedState;
	 
	PythonViewer(World& world, Vector camPos, double camAltitude, double camYaw, double camPitch, double _wallsHeight):
		ViewerWidget(&world),
		pythonSavedState(0)
	{
		camera.pos.setX(camPos.x);
		camera.pos.setY(camPos.y);
		camera.altitude = camAltitude;
		camera.yaw = camYaw;
		camera.pitch = camPitch;
		wallsHeight = _wallsHeight;
		
		managedObjectsAliases[&typeid(EPuckWrap)] = &typeid(EPuck);
	}
	
	void timerEvent(QTimerEvent * event)
	{
		// get back Python lock
		if (pythonSavedState)
			PyEval_RestoreThread(pythonSavedState);
		// touch Python objects while locked
		ViewerWidget::timerEvent(event);
		// release Python lock
		if (pythonSavedState)
			pythonSavedState = PyEval_SaveThread();
	}
};

void runInViewer(World& world, Vector camPos = Vector(0,0), double camAltitude = 0, double camYaw = 0, double camPitch = 0, double wallsHeight = 10)
{
	int argc(1);
	char* argv[1] = {(char*)"dummy"}; // FIXME: recovery sys.argv
	QApplication app(argc, argv);
	PythonViewer viewer(world, camPos, camAltitude, camYaw, camPitch, wallsHeight);
	viewer.setWindowTitle("PyEnki Viewer");
	viewer.show();
	viewer.pythonSavedState = PyEval_SaveThread();
	app.exec();
	if (viewer.pythonSavedState)
		PyEval_RestoreThread(viewer.pythonSavedState);
}

void run(World& world, unsigned steps)
{
	for (unsigned i = 0; i < steps; ++i)
		world.step(1./30., 3);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(step_overloads, step, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(runInViewer_overloads, runInViewer, 1, 6)

BOOST_PYTHON_MODULE(pyenki)
{
	// setup converters
	to_python_converter<Vector, Vector_to_python_tuple>();
	Vector_from_python();
	
	// TODO: complete doc
	
	// Color and texture
	
	class_<Color>("Color",
		"A color in RGBA",
		init<optional<double, double, double, double> >(
			"Create a RGBA color.\n\n"
			"Arguments:\n"
			"    r -- red component [0..1], default: 0.0\n"
			"    g -- green component [0..1], default: 0.0\n"
			"    b -- blue component [0..1], default: 0.0\n"
			"    a -- alpha (transparency) component [0..1], default: 1.0\n",
			args("r", "g", "b", "a")
		)
	)
		.def(self += double())
		.def(self + double())
		.def(self -= double())
		.def(self - double())
		.def(self *= double())
		.def(self * double())
		.def(self /= double())
		.def(self / double())
		.def(self += self)
		.def(self + self)
		.def(self -= self)
		.def(self - self)
		.def(self == self)
		.def(self != self)
		.def(self_ns::str(self_ns::self))
		.def("threshold", &Color::threshold)
		.def("toGray", &Color::toGray)
		.def_readonly("black", &Color::black)
		.def_readonly("white", &Color::white)
		.def_readonly("gray", &Color::gray)
		.def_readonly("red", &Color::red)
		.def_readonly("green", &Color::green)
		.def_readonly("blue", &Color::blue)
		.add_property("r", &Color::r, &Color::setR)
		.add_property("g", &Color::g, &Color::setG)
		.add_property("b", &Color::b, &Color::setB)
		.add_property("a", &Color::a, &Color::setA)
		.add_property("components", getColorComponents, setColorComponents)
	;
	
	class_<Texture>("Texture")
		.def(vector_indexing_suite<Texture>())
	;
	
	class_<Textures>("Textures")
		.def(vector_indexing_suite<Textures>())
	;
	
	// Physical objects
	
	class_<PhysicalObject>("PhysicalObject", no_init)
		.def_readonly("radius", &PhysicalObject::getRadius)
		.def_readonly("height", &PhysicalObject::getHeight)
		.def_readonly("isCylindric", &PhysicalObject::isCylindric)
		.def_readonly("mass", &PhysicalObject::getMass)
		.def_readonly("momentOfInertia", &PhysicalObject::getMomentOfInertia)
		.def_readonly("interlacedDistance", &PhysicalObject::getInterlacedDistance)
		.def_readwrite("collisionElasticity", &PhysicalObject::collisionElasticity)
		.def_readwrite("dryFrictionCoefficient", &PhysicalObject::dryFrictionCoefficient)
		.def_readwrite("viscousFrictionCoefficient", &PhysicalObject::viscousFrictionCoefficient)
		.def_readwrite("viscousMomentFrictionCoefficient", &PhysicalObject::viscousMomentFrictionCoefficient)
		.def_readwrite_by_value("pos", &PhysicalObject::pos)
		.def_readwrite("angle", &PhysicalObject::angle)
		.def_readwrite_by_value("speed", &PhysicalObject::speed)
		.def_readwrite("angSpeed", &PhysicalObject::angSpeed)
		.add_property("color",  make_function(&PhysicalObject::getColor, return_value_policy<copy_const_reference>()), &PhysicalObject::setColor)
		// warning setting the "color" property at run time using the viewer from the non-gui thread will lead to a crash because it will do an OpenGL call from that thread
	;
	
	class_<CircularPhysicalObject, bases<PhysicalObject> >("CircularObject",
		init<double, double, double, optional<const Color&> >(args("radius", "height", "mass", "color"))
	);
	
	class_<RectangularPhysicalObject, bases<PhysicalObject> >("RectangularObject",
		init<double, double, double, double, optional<const Color&> >(args("l1", "l2", "height", "mass", "color"))
	);
	
	// Robots
	
	class_<Robot, bases<PhysicalObject> >("PhysicalObject", no_init)
	;
	
	class_<DifferentialWheeled, bases<Robot> >("DifferentialWheeled", no_init)
		.def_readwrite("leftSpeed", &DifferentialWheeled::leftSpeed)
		.def_readwrite("rightSpeed", &DifferentialWheeled::rightSpeed)
		.def_readonly("leftEncoder", &DifferentialWheeled::leftEncoder)
		.def_readonly("rightEncoder", &DifferentialWheeled::rightEncoder)
		.def_readonly("leftOdometry", &DifferentialWheeled::leftOdometry)
		.def_readonly("rightOdometry", &DifferentialWheeled::rightOdometry)
		.def("resetEncoders", &DifferentialWheeled::resetEncoders)
	;
	
	class_<EPuckWrap, bases<DifferentialWheeled>, boost::noncopyable>("EPuck")
		.def("controlStep", &EPuckWrap::controlStep)
		.def_readonly("proximitySensorValues", &EPuckWrap::getProxSensorValues)
		.def_readonly("proximitySensorDistances", &EPuckWrap::getProxSensorDistances)
		.def_readonly("cameraImage", &EPuckWrap::getCameraImage)
	;
	
	class_<Thymio2Wrap, bases<DifferentialWheeled>, boost::noncopyable>("Thymio2")
		.def("controlStep", &Thymio2Wrap::controlStep)
		.def_readonly("proximitySensorValues", &Thymio2Wrap::getProxSensorValues)
		.def_readonly("proximitySensorDistances", &Thymio2Wrap::getProxSensorDistances)
		.def_readonly("groundSensorValues", &Thymio2Wrap::getGroundSensorValues)
	;
	
	// World
	
	class_<World>("WorldBase", no_init)
	;
	
	class_<WorldWithoutObjectsOwnership, bases<World> >("World",
		"The world is the container of all objects and robots.\n"
		"It is either a rectangular arena with walls at all sides, a circular area with walls, or an infinite surface."
		,
		init<double, double, optional<const Color&> >(args("width", "height", "wallsColor"))
	)
		.def(init<double, optional<const Color&> >(args("r", "wallsColor")))
		.def(init<>())
		.def("step", &World::step, step_overloads(args("dt", "physicsOversampling")))
		.def("addObject", &World::addObject, with_custodian_and_ward<1,2>())
		.def("removeObject", &World::removeObject)
		.def("setRandomSeed", &World::setRandomSeed)
		.def("run", run)
		.def("runInViewer", runInViewer, runInViewer_overloads(args("self", "camPos", "camAltitude", "camYaw", "camPitch", "wallsHeight")))
	;
	
	class_<WorldWithTexturedGround, bases<World> >("WorldWithTexturedGround",
		init<double, double, const std::string&, optional<const Color&> >(args("width", "height", "ppmFileName", "wallsColor"))
	)
		.def(init<double, const std::string&, optional<const Color&> >(args("r", "ppmFileName", "wallsColor")))
	;
}
