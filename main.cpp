#include <iostream>
#include <Windows.h>
#include "Mesh.h"
#include <thread>
#include <chrono>	
#include <algorithm>
#include "Variablen.h"


int main() {
	
	char raster[rasterHeight][rasterWidth];
	
	
	int shadingMode = 1; 
	std::string lichtwerte = "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\|()1{}[]?-_+~<>i!lI;:,\"^`'.";

	wchar_t* screen = new wchar_t[rasterHeight * rasterWidth];
	HANDLE console = CreateConsoleScreenBuffer(GENERIC_READ | GENERIC_WRITE, 0, NULL, CONSOLE_TEXTMODE_BUFFER, NULL);
	SetConsoleActiveScreenBuffer(console);
	DWORD bytesWritten = 0;

	Eigen::Matrix4d cameraToWorld;
	cameraToWorld <<
					1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, -0.1,
					0, 0, 0, 1;

	Eigen::Vector3d camera = { 0,0,0 }; 
	Eigen::Matrix4d worldToCamera = cameraToWorld.inverse();


	//Licht
	Eigen::Vector3d lightDirection = { -1, 0 , 0};
	lightDirection.normalize();
	Eigen::Matrix4d projection = projectionMatrix(1.0, -1.0, 1, -1, 1, 10);
	Eigen::Matrix4d rasterSpace = scalingMatrix(rasterWidth, rasterHeight, 1) * translationMatrix(0.5, 0.5, 0) * scalingMatrix(0.5, 0.5, 1);

	Mesh test;
	test.loadFromObjectFile("test.obj");
	test.color = 'M';
	

	std::vector<Mesh> objects;
	objects.push_back(test);
	

	std::vector<Triangle> triRenderList;
	double d = 15;
	double x = 0;
	double y = 3;
	double yR = 0;
	double xR = 0;

	Eigen::Vector4d cameraPosition = {0, 0, 5, 1};
	while (1) {
		triRenderList.clear();
		
		cameraToWorld <<
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		
		//Eingabe
		if (GetAsyncKeyState('A') & 0x8000)
		{
			Eigen::Vector4d fwd = { -0.1, 0, 0, 1 };
			fwd = yRotationMatrix(yR) * xRotationMatrix(xR) * fwd;
			cameraPosition += fwd;
			cameraPosition.w() = 1;
		
		}

		if (GetAsyncKeyState('D') & 0x8000)
		{
			Eigen::Vector4d fwd = { 0.1, 0, 0, 1 };
			fwd = yRotationMatrix(yR) * xRotationMatrix(xR) * fwd;
			cameraPosition += fwd;
			cameraPosition.w() = 1;
			
		}

		if (GetAsyncKeyState('W') & 0x8000)
		{
			Eigen::Vector4d fwd = { 0, 0, -0.1, 1 };
			fwd = yRotationMatrix(yR) * xRotationMatrix(xR) * fwd;
			cameraPosition += fwd;
			cameraPosition.w() = 1;
		}

		if (GetAsyncKeyState('S') & 0x8000)
		{
			Eigen::Vector4d fwd = { 0, 0, 0.1, 1 };
			fwd = yRotationMatrix(yR) * xRotationMatrix(xR) * fwd;
			cameraPosition += fwd;
			cameraPosition.w() = 1;
		}

		if (GetAsyncKeyState('C') & 0x8000)
		{
			
			cameraPosition.y() += 0.1;
			
		}

		if (GetAsyncKeyState('V') & 0x8000)
		{
			cameraPosition.y() -= 0.1;
			
		}

		if (GetAsyncKeyState(VK_UP) & 0x8000)
		{

			xR += 0.01;
		}

		if (GetAsyncKeyState(VK_DOWN) & 0x8000)
		{
			xR -= 0.01;
		}

		if (GetAsyncKeyState(VK_LEFT) & 0x8000)
		{
			yR += 0.01;
		}

		if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
		{
			yR -= 0.01;
		}
		
		cameraToWorld = translationMatrix(cameraPosition.x(), cameraPosition.y(), cameraPosition.z()) * yRotationMatrix(yR) * xRotationMatrix(xR); 
	
		worldToCamera = cameraToWorld.inverse();
		camera = (cameraToWorld * Eigen::Vector4d{ 0,0,0,1 }).head<3>();
	
		for (auto object : objects) {
			
			
			for (auto tri : object.tris) {


				Eigen::Vector3d line1 = tri.p[1] - tri.p[0];
				Eigen::Vector3d line2 = tri.p[2] - tri.p[0];
				Eigen::Vector3d normal = (line1.cross(line2));
				normal.normalize();
			
				double light = lightDirection.dot(normal); 

				tri.p[0] = (worldToCamera * tri.p[0].homogeneous()).head<3>();
				tri.p[1] = (worldToCamera * tri.p[1].homogeneous()).head<3>();
				tri.p[2] = (worldToCamera * tri.p[2].homogeneous()).head<3>();
			
				line1 = tri.p[1] - tri.p[0];
				line2 = tri.p[2] - tri.p[0];
				normal = (line1.cross(line2));
				normal.normalize();
			
				
				if (normal.dot(tri.p[0]) <  0) // Backface Culling
				{	Triangle triOut1;
					Triangle triOut2;	
					
					int clipStatus;
					
					clipStatus = Triangle_ClipAgainstPlane(Eigen::Vector3d{ 0,0, -1 }, Eigen::Vector3d{ 0,0, -1 }, tri, triOut1, triOut2);
					
			
					tri.setMidpointZ();
					

					if (clipStatus == 0) { continue; }

					if (clipStatus == 1) { // altes oder ein neues Dreieck
						
						Eigen::Vector4d triOut1_p0 = triOut1.p[0].homogeneous();
						Eigen::Vector4d triOut1_p1 = triOut1.p[1].homogeneous();
						Eigen::Vector4d triOut1_p2 = triOut1.p[2].homogeneous();
						
				

						triOut1_p0 = projection * triOut1_p0;
						triOut1_p1 = projection * triOut1_p1;
						triOut1_p2 = projection * triOut1_p2;
						
						//perspektivische Verk�rzung
						triOut1_p0 = triOut1_p0 / triOut1_p0.w();
						triOut1_p1 = triOut1_p1 / triOut1_p1.w();
						triOut1_p2 = triOut1_p2 / triOut1_p2.w();

						//skalieren auf Rastergr��e

						triOut1_p0 = rasterSpace * triOut1_p0;
						triOut1_p1 = rasterSpace * triOut1_p1;
						triOut1_p2 = rasterSpace * triOut1_p2;

						
						Triangle triOutFinal1(std::array<Eigen::Vector3d, 3>{triOut1_p0.head<3>(), triOut1_p1.head<3>(), triOut1_p2.head<3>()});
						triOutFinal1.light = light;
						triOutFinal1.color = object.color;
						triOutFinal1.midpointZ = tri.midpointZ;
						triRenderList.push_back(triOutFinal1);

					}

					if (clipStatus == 2) { // zwei neue Dreiecke
					
						Eigen::Vector4d triOut1_p0 = triOut1.p[0].homogeneous();
						Eigen::Vector4d triOut1_p1 = triOut1.p[1].homogeneous();
						Eigen::Vector4d triOut1_p2 = triOut1.p[2].homogeneous();
					
						Eigen::Vector4d triOut2_p0 = triOut2.p[0].homogeneous();
						Eigen::Vector4d triOut2_p1 = triOut2.p[1].homogeneous();
						Eigen::Vector4d triOut2_p2 = triOut2.p[2].homogeneous();
						
						triOut1_p0 = projection * triOut1_p0;
						triOut1_p1 = projection * triOut1_p1;
						triOut1_p2 = projection * triOut1_p2;

						triOut2_p0 = projection * triOut2_p0;
						triOut2_p1 = projection * triOut2_p1;
						triOut2_p2 = projection * triOut2_p2;

						//perspektivische Verk�rzung
						triOut1_p0 = triOut1_p0 / triOut1_p0.w();
						triOut1_p1 = triOut1_p1 / triOut1_p1.w();
						triOut1_p2 = triOut1_p2 / triOut1_p2.w();

						triOut2_p0 = triOut2_p0 / triOut2_p0.w();
						triOut2_p1 = triOut2_p1 / triOut2_p1.w();
						triOut2_p2 = triOut2_p2 / triOut2_p2.w();

						//skalieren auf Rastergr��e

						
						triOut1_p0 = rasterSpace * triOut1_p0;
						triOut1_p1 = rasterSpace * triOut1_p1;
						triOut1_p2 = rasterSpace * triOut1_p2;

						triOut2_p0 = rasterSpace * triOut2_p0;
						triOut2_p1 = rasterSpace * triOut2_p1;
						triOut2_p2 = rasterSpace * triOut2_p2;

						Triangle triOutFinal1(std::array<Eigen::Vector3d, 3>{triOut1_p0.head<3>(), triOut1_p1.head<3>(), triOut1_p2.head<3>()});
						Triangle triOutFinal2(std::array<Eigen::Vector3d, 3>{triOut2_p0.head<3>(), triOut2_p1.head<3>(), triOut2_p2.head<3>()});


						triOutFinal1.light =light;
						triOutFinal2.light =light;


						triOutFinal1.color = object.color;
						triOutFinal2.color = object.color;

						triOutFinal1.midpointZ = tri.midpointZ;
						triOutFinal2.midpointZ = tri.midpointZ;
						triRenderList.push_back(triOutFinal1);
						triRenderList.push_back(triOutFinal2);
						
					}
				}
			}
		}
		
		//Painter Algorithmus
		std::sort(triRenderList.begin(), triRenderList.end(), compareByDepth);

		//Dreicke zeichnen
		for (const auto& tri : triRenderList) { 
			//bounding box
			double l = rasterWidth, r = 0, t = 0, b = rasterHeight;

			for (int i = 0; i < 3; i++) {
				if (tri.p[i].x() < l) l = tri.p[i].x();
				if (tri.p[i].x() > r) r = tri.p[i].x();
				if (tri.p[i].y() < b) b = tri.p[i].y();
				if (tri.p[i].y() > t) t = tri.p[i].y();

			}
			if (b <= 0) b = 0;
			if (l <= 0) l = 0;

			
			
			double lightDensity = (tri.getLight() + 2.5) * 2;
			/*
			if ((r - l) < lightDensity) lightDensity  = lightDensity * 0.5;
			else if ((t - b)< lightDensity) lightDensity = lightDensity * 0.5;

			for (int y = b - 1; y < rasterHeight && y < t + 2; y += 1) {
			for (int x = l - 1; x < rasterWidth && x < r + 2; x += 1) {

					if ((!edgeFunction(tri.p[1], tri.p[2], Eigen::Vector2d{ (x),(y) }) &&
						!edgeFunction(tri.p[2], tri.p[0], Eigen::Vector2d{ (x),(y) }) &&
						!edgeFunction(tri.p[0], tri.p[1], Eigen::Vector2d{ (x),(y) })) ||
						(!edgeFunction(tri.p[1], tri.p[2], Eigen::Vector2d{ (x),(y) }) &&
						 !edgeFunction(tri.p[2], tri.p[0], Eigen::Vector2d{ (x),(y) }) &&
						 !edgeFunction(tri.p[0], tri.p[1], Eigen::Vector2d{ (x),(y) })))
					{
						 raster[y][x] = ' ';
					}

				}
			}
			if (shadingMode == 0) {
				 int lightDensity = 1;	

			}
			*/

			lightDensity = 1;

			for (int y = b - 1; y < rasterHeight && y < t + 2;  y += lightDensity) {
				for (int x = l - 1  ; x < rasterWidth && x < r + 2 ; x += lightDensity) {
					
					if ((!edgeFunction(tri.p[1], tri.p[2], Eigen::Vector2d{ (x),(y) }) &&
						!edgeFunction(tri.p[2], tri.p[0], Eigen::Vector2d{ (x),(y) }) &&
						!edgeFunction(tri.p[0], tri.	p[1], Eigen::Vector2d{ (x),(y) })) ||
						(edgeFunction(tri.p[1], tri.p[2], Eigen::Vector2d{ (x),(y) }) &&
							edgeFunction(tri.p[2], tri.p[0], Eigen::Vector2d{ (x),(y) }) &&
							edgeFunction(tri.p[0], tri.p[1], Eigen::Vector2d{ (x),(y) })))
					{

						/*
						double w0 = ((tri.p[1].y() - tri.p[2].y()) * (x - tri.p[1].x()) + (tri.p[2].x() - tri.p[1].x()) * (y - tri.p[2].y())) /
									((tri.p[1].y() - tri.p[2].y()) * (tri.p[0].x() - tri.p[2].x()) + (tri.p[2].x() - tri.p[1].x()) * (tri.p[0].y() - tri.p[2].y()));

						double w1 = ((tri.p[2].y() - tri.p[0].y()) * (x - tri.p[2].x()) + (tri.p[1].x() - tri.p[2].x()) * (y - tri.p[2].y())) /
									((tri.p[1].y() - tri.p[2].y()) * (tri.p[0].x() - tri.p[2].x()) + (tri.p[2].x() - tri.p[1].x()) * (tri.p[0].y() - tri.p[2].y()));

						double w2 = 1 - w0 - w1;


						
						double z = w0 * tri.p[0].z() + w1 * tri.p[1].z() + w2 * tri.p[2].z();
						*/
						{
							raster[y][x] = lichtwerte[(((tri.getLight() + 1.1) / 2) * lichtwerte.length())];
					
							//if (shadingMode == 1) 
							//	raster[y][x] = 'X';

							
						}
					
					}
				}
			}

			//raster[(int)(tri.p[0].y())][(int)(tri.p[0].x())] = 'X';
			//raster[(int)(tri.p[1].y())][(int)(tri.p[1].x())] = 'X';
			//raster[(int)(tri.p[2].y())][(int)(tri.p[2].x())] = 'X';
		}

		//Fenster Umrahmung
		for (int x = 0; x < rasterWidth; x++) {
				raster[0][x] = 'X';
				raster[rasterHeight-1][x] = 'X';
		}
		for (int y = 0; y < rasterHeight; y++) {
			raster[y][0] = 'X';
			raster[y][rasterWidth-1] = 'X';
		}

		int i = 0;
		for (int y = 0; y < rasterHeight; y++) {
			for (int x = 0; x < rasterWidth; x++) {
				screen[i] = raster[y][x];
				i++;
				raster[y][x] = ' ';
			}
		}

		//break;
		WriteConsoleOutputCharacter(console, screen, rasterHeight * rasterWidth, { 0, 0 }, &bytesWritten);
		//Sleep(20);
	}
	
	return 0;
}