///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>
#include <algorithm>

// Helper: Check if a point is inside a single rectangle
bool pointInRect(double x, double y, const Rectangle &rect)
{
    return x >= rect.x && x <= rect.x + rect.width &&
           y >= rect.y && y <= rect.y + rect.height;
}

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const auto &rect : obstacles)
    {
        if (pointInRect(x, y, rect))
        {
            return false;
        }
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const auto &rect : obstacles)
    {
        double xmin = rect.x;
        double xmax = rect.x + rect.width;
        double ymin = rect.y;
        double ymax = rect.y + rect.height;

        // Find closest point on rectangle to circle center
        double closest_x = std::max(xmin, std::min(x, xmax));
        double closest_y = std::max(ymin, std::min(y, ymax));

        // Check distance from circle center to closest point
        double dx = x - closest_x;
        double dy = y - closest_y;
        if (dx * dx + dy * dy <= radius * radius)
        {
            return false;
        }
    }
    return true;
}

// Checks if segment (x1,y1)-(x2,y2) intersects segment (x3,y3)-(x4,y4)
bool segmentsIntersect(double x1, double y1, double x2, double y2,
                       double x3, double y3, double x4, double y4)
{
    auto cross = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };

    double d1 = cross(x3, y3, x4, y4, x1, y1);
    double d2 = cross(x3, y3, x4, y4, x2, y2);
    double d3 = cross(x1, y1, x2, y2, x3, y3);
    double d4 = cross(x1, y1, x2, y2, x4, y4);

    return (d1 * d2 < 0) && (d3 * d4 < 0);
}

// Checks if point (px,py) is inside square centered at (sx,sy) with rotation theta
bool pointInSquare(double px, double py, double sx, double sy, double theta, double sideLength)
{
    double dx = px - sx;
    double dy = py - sy;
    double localX = dx * cos(-theta) - dy * sin(-theta);
    double localY = dx * sin(-theta) + dy * cos(-theta);
    double half = sideLength / 2.0;
    return (localX >= -half && localX <= half && localY >= -half && localY <= half);
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    double half = sideLength / 2.0;
    double cosT = cos(theta);
    double sinT = sin(theta);

    double ax = x + cosT * (-half) - sinT * (-half);
    double ay = y + sinT * (-half) + cosT * (-half);
    double bx = x + cosT * (-half) - sinT * (half);
    double by = y + sinT * (-half) + cosT * (half);
    double cx = x + cosT * (half) - sinT * (half);
    double cy = y + sinT * (half) + cosT * (half);
    double dx = x + cosT * (half) - sinT * (-half);
    double dy = y + sinT * (half) + cosT * (-half);

    for (const auto &rect : obstacles)
    {
        double rxMin = rect.x;
        double ryMin = rect.y;
        double rxMax = rect.x + rect.width;
        double ryMax = rect.y + rect.height;

        if (pointInRect(ax, ay, rect) || pointInRect(bx, by, rect) ||
            pointInRect(cx, cy, rect) || pointInRect(dx, dy, rect))
            return false;

        if (pointInSquare(rxMin, ryMin, x, y, theta, sideLength) ||
            pointInSquare(rxMax, ryMin, x, y, theta, sideLength) ||
            pointInSquare(rxMin, ryMax, x, y, theta, sideLength) ||
            pointInSquare(rxMax, ryMax, x, y, theta, sideLength))
            return false;

        double sqEdges[4][4] = {{ax, ay, bx, by}, {bx, by, cx, cy}, {cx, cy, dx, dy}, {dx, dy, ax, ay}};
        double rectEdges[4][4] = {{rxMin, ryMin, rxMax, ryMin}, {rxMax, ryMin, rxMax, ryMax},
                                  {rxMax, ryMax, rxMin, ryMax}, {rxMin, ryMax, rxMin, ryMin}};

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                if (segmentsIntersect(sqEdges[i][0], sqEdges[i][1], sqEdges[i][2], sqEdges[i][3],
                                      rectEdges[j][0], rectEdges[j][1], rectEdges[j][2], rectEdges[j][3]))
                    return false;
    }
    return true;
}
