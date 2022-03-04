
#pragma once
#include <Eigen>



Eigen::Matrix4d xRotationMatrix(double xRotation);

Eigen::Matrix4d yRotationMatrix(double yRotation);

Eigen::Matrix4d zRotationMatrix(double zRotation);

Eigen::Matrix4d scalingMatrix(double x, double y, double z);

Eigen::Matrix4d translationMatrix(double x, double y, double z);

Eigen::Matrix4d projectionMatrix(double right, double left, double top, double bottom, double nearPlane, double farPlane);

bool edgeFunction(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector2d p3);

Eigen::Vector3d lineIntersection(Eigen::Vector3d planePoint, Eigen::Vector3d planeNormal, Eigen::Vector3d linePoint, Eigen::Vector3d lineDirection);



void bhm_line(int x1, int y1, int x2, int y2, char(&raster)[100][100]);