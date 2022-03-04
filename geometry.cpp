	#include "geometry.h"


Eigen::Matrix4d xRotationMatrix(double xRotation) {
	Eigen::Matrix4d xRotationMatrix;

	xRotationMatrix <<

		1, 0, 0, 0,
		0, cos(xRotation), -sin(xRotation), 0,
		0, sin(xRotation), cos(xRotation), 0,
		0, 0, 0, 1;


	return xRotationMatrix;
}

Eigen::Matrix4d yRotationMatrix(double yRotation) {
	Eigen::Matrix4d yRotationMatrix;

	yRotationMatrix <<

		cos(yRotation), 0, sin(yRotation), 0,
		0, 1, 0, 0,
		-sin(yRotation), 0, cos(yRotation), 0,
		0, 0, 0, 1;

	return yRotationMatrix;
}

Eigen::Matrix4d zRotationMatrix(double zRotation) {
	Eigen::Matrix4d zRotationMatrix;

	zRotationMatrix <<

		cos(zRotation), -sin(zRotation), 0, 0,
		sin(zRotation), cos(zRotation), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return zRotationMatrix;
}

Eigen::Matrix4d scalingMatrix(double x, double y, double z) {
	Eigen::Matrix4d scalingMatrix;

	scalingMatrix <<

		x, 0, 0, 0,
		0, y, 0, 0,
		0, 0, z, 0,
		0, 0, 0, 1;

	return scalingMatrix;
}

Eigen::Matrix4d translationMatrix(double x, double y, double z) {
	Eigen::Matrix4d translationMatrix;

	translationMatrix <<

		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;

	return translationMatrix;
}

Eigen::Matrix4d projectionMatrix(double right, double left, double top, double bottom, double nearPlane, double farPlane) {
	Eigen::Matrix4d proj;

	proj <<
		(2 * nearPlane) / (right - left),  0,								 (right + left) / (right - left),				 	   0,
		0,								  (2 * nearPlane) / (top - bottom),  (top + bottom) / (top - bottom),				       0,
		0,								   0,								-((farPlane + nearPlane) / (farPlane - nearPlane)), -((2 * farPlane * nearPlane) / (farPlane - nearPlane)),
		0,								   0,							     -1,												   0;

	return proj;
}

bool edgeFunction(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector2d p3)
{
	return ((p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) *  (p1.y() - p3.y()) <= 0);
}

Eigen::Vector3d lineIntersection(Eigen::Vector3d planePoint, Eigen::Vector3d planeNormal, Eigen::Vector3d linePoint, Eigen::Vector3d lineDirection) {
	if (planeNormal.dot(lineDirection.normalized()) == 0) {
		return Eigen::Vector3d{ 0,0,0 };
	}

	double t = (planeNormal.dot(planePoint) - planeNormal.dot(linePoint)) / planeNormal.dot(lineDirection.normalized());
	return linePoint + (lineDirection.normalized() * t);
}

void bhm_line(int x1, int y1, int x2, int y2, char(&raster)[100][100])
{
	int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;
	dx = x2 - x1;
	dy = y2 - y1;
	dx1 = fabs(dx);
	dy1 = fabs(dy);
	px = 2 * dy1 - dx1;
	py = 2 * dx1 - dy1;
	if (dy1 <= dx1)
	{
		if (dx >= 0)
		{
			x = x1;
			y = y1;
			xe = x2;
		}
		else
		{
			x = x2;
			y = y2;
			xe = x1;
		}
		raster[y][x] = 'O';
		for (i = 0; x < xe; i++)
		{
			x = x + 1;
			if (px < 0)
			{
				px = px + 2 * dy1;
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
				{
					y = y + 1;
				}
				else
				{
					y = y - 1;
				}
				px = px + 2 * (dy1 - dx1);
			}
			raster[y][x] = 'O';
		}
	}
	else
	{
		if (dy >= 0)
		{
			x = x1;
			y = y1;
			ye = y2;
		}
		else
		{
			x = x2;
			y = y2;
			ye = y1;
		}
		raster[y][x] = 'O';
		for (i = 0; y < ye; i++)
		{
			y = y + 1;
			if (py <= 0)
			{
				py = py + 2 * dx1;
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
				{
					x = x + 1;
				}
				else
				{
					x = x - 1;
				}
				py = py + 2 * (dx1 - dy1);
			}
			raster[y][x] = 'O';
		}
	}
}

