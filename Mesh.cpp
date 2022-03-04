
#include "Mesh.h"

Triangle::Triangle(std::array<Eigen::Vector3d, 3> points) {
	p = points;
}

Triangle::Triangle() {
	;
}

void Triangle::setMidpointZ() {
	midpointZ = (this->p[0].z() + this->p[1].z() + this->p[2].z())/3;
}

double Triangle::getMidpointZ() const {
	return midpointZ;
}

void   Triangle::setLight(double light) {
	light = light;
}

double Triangle::getLight() const {
	return light;
}

//Vector_IntersectPlane und Triangle_ClipAgaingstPlane von javidx9 https://github.com/OneLoneCoder/videos/blob/master/OneLoneCoder_olcEngine3D_Part2.cpp
Eigen::Vector3d Vector_IntersectPlane(Eigen::Vector3d& plane_p, Eigen::Vector3d& plane_n, Eigen::Vector3d& lineStart, Eigen::Vector3d& lineEnd)
{
	plane_n = plane_n.normalized();
	float plane_d = -plane_n.dot(plane_p);
	float ad = lineStart.dot(plane_n);
	float bd = lineEnd.dot(plane_n);
	float t = (-plane_d - ad) / (bd - ad);
	Eigen::Vector3d  lineStartToEnd = lineEnd - lineStart;
	Eigen::Vector3d  lineToIntersect = lineStartToEnd * t;
	return (lineStart +  lineToIntersect);
}



int Triangle_ClipAgainstPlane(Eigen::Vector3d plane_p, Eigen::Vector3d plane_n, Triangle& in_tri, Triangle& out_tri1, Triangle& out_tri2)
{
	
	// Make sure plane normal is indeed normal
	plane_n = plane_n.normalized();

	// Return signed shortest distance from point to plane, plane normal must be normalised
	auto dist = [&](Eigen::Vector3d& p)
	{
		Eigen::Vector3d n = p.normalized();
		return (plane_n.x() * p.x() + plane_n.y() * p.y() + plane_n.z() * p.z() - plane_n.dot(plane_p));
	};

	// Create two temporary storage arrays to classify points either side of plane
	// If distance sign is positive, point lies on "inside" of plane
	std::array<Eigen::Vector3d,3> inside_points;  int nInsidePointCount = 0;
	std::array<Eigen::Vector3d, 3> outside_points; int nOutsidePointCount = 0;

	// Get signed distance of each point in triangle to plane
	float d0 = dist(in_tri.p[0]);
	float d1 = dist(in_tri.p[1]);
	float d2 = dist(in_tri.p[2]);

	if (d0 >= 0) { inside_points[nInsidePointCount++] = in_tri.p[0]; }
	else { outside_points[nOutsidePointCount++] = in_tri.p[0]; }
	if (d1 >= 0) { inside_points[nInsidePointCount++] = in_tri.p[1]; }
	else { outside_points[nOutsidePointCount++] = in_tri.p[1]; }
	if (d2 >= 0) { inside_points[nInsidePointCount++] = in_tri.p[2]; }
	else { outside_points[nOutsidePointCount++] = in_tri.p[2]; }

	// Now classify triangle points, and break the input triangle into 
	// smaller output triangles if required. There are four possible
	// outcomes...

	if (nInsidePointCount == 0)
	{
		// All points lie on the outside of plane, so clip whole triangle
		// It ceases to exist

		return 0; // No returned triangles are valid
	}

	if (nInsidePointCount == 3)
	{
		// All points lie on the inside of plane, so do nothing
		// and allow the triangle to simply pass through
		out_tri1 = in_tri;

		return 1; // Just the one returned original triangle is valid
	}

	if (nInsidePointCount == 1 && nOutsidePointCount == 2)
	{
		// Triangle should be clipped. As two points lie outside
		// the plane, the triangle simply becomes a smaller triangle

		// Copy appearance info to new triangl
		out_tri1 = in_tri;
		//out_tri1.col = in_tri.col;
		//out_tri1.sym = in_tri.sym;

		// The inside point is valid, so keep that...
		out_tri1.p[0] = inside_points[0];

		// but the two new points are at the locations where the 
		// original sides of the triangle (lines) intersect with the plane
		out_tri1.p[1] = Vector_IntersectPlane(plane_p, plane_n, inside_points[0], outside_points[0]);//1 0 0
		out_tri1.p[2] = Vector_IntersectPlane(plane_p, plane_n, inside_points[0], outside_points[1]);//2 0 1

		return 1; // Return the newly formed single triangle
	}

	if (nInsidePointCount == 2 && nOutsidePointCount == 1)
	{
		// Triangle should be clipped. As two points lie inside the plane,
		// the clipped triangle becomes a "quad". Fortunately, we can
		// represent a quad with two new triangles

		// Copy appearance info to new triangles
		out_tri1 = in_tri;
		//out_tri1.col = in_tri.col;
		//out_tri1.sym = in_tri.sym;
		out_tri2 = in_tri;
		//out_tri2.col = in_tri.col;
		//out_tri2.sym = in_tri.sym;

		// The first triangle consists of the two inside points and a new
		// point determined by the location where one side of the triangle
		// intersects with the plane
		out_tri1.p[0] = inside_points[0]; // 0 0
		out_tri1.p[1] = inside_points[1]; // 1 1
		out_tri1.p[2] = Vector_IntersectPlane(plane_p, plane_n, inside_points[0], outside_points[0]); // 2 0 0

		// The second triangle is composed of one of he inside points, a
		// new point determined by the intersection of the other side of the 
		// triangle and the plane, and the newly created point above
		out_tri2.p[0] = inside_points[1]; 
		out_tri2.p[1] = out_tri1.p[2];	
		out_tri2.p[2] = Vector_IntersectPlane(plane_p, plane_n, inside_points[1], outside_points[0]); // 2 1 0

		return 2; 
	}

}


bool compareByDepth(const Triangle& a, const Triangle& b)
{
	return (a.midpointZ < b.midpointZ);
}

bool Mesh::loadFromObjectFile(std::string sFilename) {
	std::ifstream f(sFilename);
	if (!f.is_open()) {
		//std::cout << "Fehler beim Laden von " + sFilename + "\n";
		return false;
	}

	std::vector<Eigen::Vector3d> verts;

	while (!f.eof()) {
		char line[128];
		f.getline(line, 128);

		std::strstream s;

		s << line;

		char c;

		if (line[0] == 'v') {
			Eigen::Vector3d v;
			s >> c >> v.x() >> v.y() >> v.z();
			verts.push_back(v);

		}

		if (line[0] == 'f') {
			int f[3];
			s >> c >> f[0] >> f[1] >> f[2];
			tris.push_back(Triangle(std::array<Eigen::Vector3d,3>{verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1]}));

		}
	}
	return true;
}

void Mesh::originToGeometry() {
	origin = {0, 0, 0};
	int n = 0;
	for (auto &tri : tris) {
		n++;
		origin += (tri.p[0] + tri.p[1] + tri.p[2]) / 3;
	}

	if (n != 0)
		origin = origin / n;
	else
		origin = { 0, 0, 0 };
}

Eigen::Vector3d Mesh::getOrigin() {
	return origin;
}

void Mesh::scale(double x, double y, double z) {
	Eigen::Matrix4d scaleMatrix = scalingMatrix(x, y, z);
	for (auto& tri : tris) {
		for (int i = 0; i < 3; i++) {
			tri.p[i] = (scaleMatrix * tri.p[i].homogeneous()).head<3>();
		};
	}
}

void Mesh::rotate(double xAngle, double yAngle, double zAngle) {

	Eigen::Matrix4d rotation = 
		 translationMatrix(origin.x(), origin.y(), origin.z()) *
		 zRotationMatrix(zAngle) * yRotationMatrix(yAngle) * xRotationMatrix(xAngle) *
		 translationMatrix(-origin.x(), -origin.y(), -origin.z());
	

	for (auto& tri : tris) {
		
		for (auto &p : tri.p) {
			p = (rotation * p.homogeneous()).head<3>();
		}

	}
}

void Mesh::translate(double x, double y, double z) {
	
	for (auto& tri : tris) {
		for (int i = 0; i < 3; i++) {
			tri.p[i].x() = tri.p[i].x() + x;
			tri.p[i].y() = tri.p[i].y() + y;
			tri.p[i].z() = tri.p[i].z() + z;
		}
	}
}

