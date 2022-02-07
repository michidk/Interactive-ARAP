#include <iostream>
#include <string>

#include <Eigen/StdVector>

#include <igl/readOFF.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/unproject.h>

#include <list>
#include <set>

#include "gui.h"
#include "utils.h"
#include "arap.h"

#define LEFT_MOUSE_BUTTON 0
#define CENTER_MOUSE_BUTTON 1
#define RIGHT_MOUSE_BUTTON 2

#define FIXED_POINTS_COLOR Eigen::RowVector3d(1, 0, 0)
#define HANDLE_COLOR Eigen::RowVector3d(0, 1, 0)

#define VERBOSE false

GUI::GUI(std::string modelFilePath)
{
	PRINT("Initializing GUI...");
	printUserInfo();
	// Load a mesh in OFF format
	igl::readOFF(modelFilePath, vertices, faces);
	// initialize ARAP
	arap = new ARAP(vertices, faces);

	// mouse button down callback
	viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer &viewer, int button, int) -> bool
	{
		if (mode == MOVE_HANDLE)
			return true;

		currMouseEvent = button;

		int fid;
		Eigen::Vector3f bc;

		// based on https://github.com/libigl/libigl/blob/3123b967c1e2cd8e9233937396c5736f2274b5e0/tutorial/708_Picking/main.cpp#L23
		double x = viewer.current_mouse_x;
		double y = viewer.core().viewport(3) - viewer.current_mouse_y;

		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view, viewer.core().proj, viewer.core().viewport, vertices, faces, fid, bc))
		{
			// get the vertices of the face with the id fid
			Eigen::MatrixXi tri = faces.row(fid);

			bool unselect = button == RIGHT_MOUSE_BUTTON;
			switch (mode)
			{
			case SELECT_FIXED_VERTICES:
				selectFixedVertex(getClosestVertex(tri, bc), unselect);
				return true;
			case SELECT_FIXED_FACES:
				selectFixedVertex(tri(0), unselect);
				selectFixedVertex(tri(1), unselect);
				selectFixedVertex(tri(2), unselect);
				return true;
			case SELECT_HANDLE:
				selectHandle(getClosestVertex(tri, bc), unselect);
				return true;
			default:
				break;
			}
		}

		return mode != ROTATE_OBJECT;
	};

	// mouse move callback
	viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer &viewer, int, int) -> bool
	{
		if (currMouseEvent == LEFT_MOUSE_BUTTON || currMouseEvent == RIGHT_MOUSE_BUTTON)
		{
			int fid;
			Eigen::Vector3f bc;

			// based on https://github.com/libigl/libigl/blob/3123b967c1e2cd8e9233937396c5736f2274b5e0/tutorial/708_Picking/main.cpp#L23
			double x = viewer.current_mouse_x;
			double y = viewer.core().viewport(3) - viewer.current_mouse_y;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view, viewer.core().proj,
										 viewer.core().viewport, vertices, faces, fid, bc))
			{
				// get the vertices of the face with the id fid
				Eigen::MatrixXi tri = faces.row(fid);

				bool unselect = currMouseEvent == RIGHT_MOUSE_BUTTON;
				switch (mode)
				{
				case SELECT_FIXED_VERTICES:
					selectFixedVertex(getClosestVertex(tri, bc), unselect);
					return true;
				case SELECT_FIXED_FACES:
					selectFixedVertex(tri(0), unselect);
					selectFixedVertex(tri(1), unselect);
					selectFixedVertex(tri(2), unselect);
					return true;
				case SELECT_HANDLE:
					selectHandle(getClosestVertex(tri, bc), unselect);
					return true;
				default:
					break;
				}
			}
		}

		return false;
	};

	// mouse button up callback
	viewer.callback_mouse_up = [this](igl::opengl::glfw::Viewer &viewer, int button, int) -> bool
	{
		currMouseEvent = -1;

		if (button == LEFT_MOUSE_BUTTON && mode == MOVE_HANDLE && currentHandle != -1)
		{
			double x = viewer.current_mouse_x;
			double y = viewer.core().viewport(3) - (float)viewer.current_mouse_y;

			Eigen::Vector3f handlePos = vertices.row(currentHandle).cast<float>();
			Eigen::Vector3d projection = igl::project(handlePos, viewer.core().view, viewer.core().proj, viewer.core().viewport).cast<double>();
			Eigen::Vector3d worldPos = igl::unproject(Eigen::Vector3f(x, y, (float)projection.z()), viewer.core().view, viewer.core().proj, viewer.core().viewport).cast<double>();
			Eigen::Vector3d move = handlePos.cast<double>() - worldPos; // this is just a movement vector that we will not deform

			PRINT("ARAP REQUEST: Grabbed handle " << currentHandle << " and move all by vector " << move.x() << " - " << move.y() << " - " << move.z())
			vertices = arap->deform(fixedVertices, handles, move);
			viewer.data().set_mesh(vertices, faces);
			redraw();
			return true;
		}
		return false;
	};

	// keyboard press callback
	viewer.callback_key_down = [this](igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) -> bool
	{
#if VERBOSE
		std::cout << "Key pressed: " << key << std::endl;
#endif
		switch (key)
		{
		case '1':
			mode = SELECT_FIXED_VERTICES;
			PRINT("Mode: select fixed vertices");
			return true;
		case '2':
			mode = SELECT_FIXED_FACES;
			PRINT("Mode: select fixed faces");
			return true;
		case '3':
			mode = SELECT_HANDLE;
			PRINT("Mode: select a handle");
			return true;
		case '4':
			mode = MOVE_HANDLE;
			PRINT("Mode: move selected handles");
			return true;
		case '5':
			mode = ROTATE_OBJECT;
			PRINT("Mode: rotate object")
			return true;
		case '0':
			viewer.open_dialog_save_mesh();
			PRINT("Opening save-dialog...");
			return true;
		default:
			break;
		}
		return false;
	};

	// Plot the mesh
	visualize();
}

void GUI::visualize()
{
	PRINT("Launching GUI...");
	viewer.data().set_mesh(vertices, faces);
	viewer.data().point_size = 10;
	viewer.data().set_colors(Eigen::MatrixXd::Constant(faces.rows(), 3, 1));
	viewer.data().show_lines = true;
	viewer.data().double_sided = true;
	viewer.launch();
	PRINT("Closing GUI");
}

void GUI::selectFixedVertex(int vertexId, bool removeSelection = false)
{
	if (!removeSelection)
	{
		if (!CONTAINS(fixedVertices, vertexId))
		{
			fixedVertices.insert(vertexId);
			viewer.data().add_points(vertices.row(vertexId), FIXED_POINTS_COLOR);
#if VERBOSE
			PRINT("Selected fixed vertex id: " << vertexId)
#endif
		}
	}
	else
	{
		if (CONTAINS(fixedVertices, vertexId))
		{
			fixedVertices.erase(vertexId);
			redraw();
#if VERBOSE
			PRINT("Deselected fixed vertex id: " << vertexId)
#endif
		}
	}
}

void GUI::selectHandle(int vertexId, bool removeSelection = false)
{
	if (!removeSelection)
	{
		if (!CONTAINS(handles, vertexId))
		{
			handles.insert(vertexId);
			viewer.data().add_points(vertices.row(vertexId), HANDLE_COLOR);
#if VERBOSE
			PRINT("Selected handle vertex id: " << vertexId)
#endif
		}
		if (currentHandle != vertexId)
		{
			currentHandle = vertexId;
		}
	}
	else
	{
		if (CONTAINS(handles, vertexId))
		{
			handles.erase(vertexId);
			redraw();
#if VERBOSE
			PRINT("Deselected handle vertex id: " << vertexId)
#endif
		}
		if (currentHandle == vertexId)
		{
			currentHandle = -1;
		}
	}
}

void GUI::redraw()
{
	viewer.data().clear_points();
	//#pragma omp parallel for
	for (auto itr = handles.begin(); itr != handles.end(); itr++)
	{
		viewer.data().add_points(vertices.row(*itr), HANDLE_COLOR);
	}
	if (currentHandle != -1)
		viewer.data().add_points(vertices.row(currentHandle), HANDLE_COLOR);
	//#pragma omp parallel for
	for (auto itr = fixedVertices.begin(); itr != fixedVertices.end(); itr++)
	{
		viewer.data().add_points(vertices.row(*itr), FIXED_POINTS_COLOR);
	}
}
