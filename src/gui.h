#pragma once

#include <set>
#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>

#include "arap.h"

enum InteractionMode
{
    ROTATE_OBJECT,
    SELECT_FIXED_VERTICES,
    SELECT_FIXED_FACES,
    SELECT_HANDLE,
    MOVE_HANDLE
};

class GUI
{
public:
    GUI(std::string modelFilePath);
    InteractionMode mode = SELECT_FIXED_FACES;

private:
    igl::opengl::glfw::Viewer viewer;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    std::set<int> fixedVertices;
    int currentHandle = -1;
    std::set<int> handles;
    InteractionMode modeBeforeMoving;
    int currMouseEvent = -1;
    ARAP *arap;

    void selectFixedVertex(int vertexId, bool unselect);
    void selectHandle(int vertexId, bool unselect);
    void visualize();
    void redraw();
};
