
#pragma once
#include <array>
#include <vector>
#include <fstream>
#include <strStream>
#include "geometry.h"

struct Triangle {
	double midpointZ;
	char color = 'X';

	Triangle();
	Triangle(std::array<Eigen::Vector3d, 3> points);
	double light;
	void   setMidpointZ();
	double getMidpointZ() const;
	double getLight() const;
	void   setLight(double light);
	std::array<Eigen::Vector3d, 3> p;

};

bool compareByDepth(const Triangle& a, const Triangle& b);

struct Mesh {
private:

	Eigen::Vector3d origin;
public:
	char color = 'X';
	std::vector<Triangle> tris;
	Eigen::Vector3d getOrigin();
	bool loadFromObjectFile(std::string sFilename);
	void originToGeometry();
	void scale(double x, double y, double z);
	void rotate(double xAngle, double yAngle, double zAngle);
	void translate(double x, double y, double z);
};
Eigen::Vector3d Vector_IntersectPlane(Eigen::Vector3d& plane_p, Eigen::Vector3d& plane_n, Eigen::Vector3d& lineStart, Eigen::Vector3d& lineEnd);
int Triangle_ClipAgainstPlane(Eigen::Vector3d plane_p, Eigen::Vector3d plane_n, Triangle& in_tri, Triangle& out_tri1, Triangle& out_tri2);
