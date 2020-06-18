#include "benchmark/math_computing.h" 

std::vector<double> benchmark::intersectWith(pcl::PolygonMesh mesh, std::vector<double> point, std::vector<double> normal)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    double x0 = point[0];
    double y0 = point[1];
    double z0 = point[2];
    double t = INFINITY;
    size_t polygon_size = mesh.polygons.size();
    std::vector<double> intersection_point = {0.0, 0.0, 0.0, 1.0};

    for (size_t i = 0; i < polygon_size; i++)
    {
        // Parameters for the plane which contains the polygon
        std::vector<double> x = {0.0, 1.0, 0.0};
        std::vector<double> y = {0.0, 0.0, 1.0};
        std::vector<double> z = {1.0, 1.0, 1.0};
        std::vector<std::vector<double>> polygon;
        
        int vertices_size = mesh.polygons[i].vertices.size();
        for (int j = 0; j < vertices_size; j++)
        {
            if (j < 3)
            {
                x[j] = mesh_cloud->points[mesh.polygons[i].vertices[j]].x;
                y[j] = mesh_cloud->points[mesh.polygons[i].vertices[j]].y;
                z[j] = mesh_cloud->points[mesh.polygons[i].vertices[j]].z;
            }
            polygon.push_back({
                mesh_cloud->points[mesh.polygons[i].vertices[j]].x, 
                mesh_cloud->points[mesh.polygons[i].vertices[j]].y,
                mesh_cloud->points[mesh.polygons[i].vertices[j]].z
                });
        }

        double a = (y[1] - y[0]) * (z[2] - z[0]) - (y[2] - y[0]) * (z[1] -z[0]);
        double b = (z[1] - z[0]) * (x[2] - x[0]) - (z[2] - z[0]) * (x[1] -x[0]);
        double c = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] -y[0]);

        // Parameter for intersection point
        double tmp = ((x[0] - x0) * a + (y[0] - y0) * b + (z[0] - z0) * c) / (a * normal[0] + b * normal[1] + c * normal[2]);
        intersection_point[0] = x0 + normal[0] * tmp;
        intersection_point[1] = y0 + normal[1] * tmp;
        intersection_point[2] = z0 + normal[2] * tmp; 

        bool is_inside = benchmark::isInside(intersection_point, polygon);

        if ((fabs(tmp) < fabs(t)) and is_inside)
        {
            t = tmp;
        }
        else; // Do nothing

    }
    if (t == INFINITY)
    {
        intersection_point[0] = x0 + normal[0];
        intersection_point[1] = y0 + normal[1];
        intersection_point[2] = z0 + normal[2]; 
    }
    else
    {
        intersection_point[0] = x0 + normal[0] * t;
        intersection_point[1] = y0 + normal[1] * t;
        intersection_point[2] = z0 + normal[2] * t; 
        intersection_point[3] = t;
    }
    return intersection_point;
}

bool benchmark::isInside(std::vector<double> point, std::vector<std::vector<double>> polygon)
{
    std::vector<double> angles;
    for (auto itr = polygon.begin(); itr != polygon.end(); ++itr)
    {
        std::vector<double> vector1;
        std::vector<double> vector2;

        if ((itr+1) != polygon.end())
        {
            vector1.push_back((*itr)[0] - point[0]);
            vector1.push_back((*itr)[1] - point[1]);
            vector1.push_back((*itr)[2] - point[2]);
            itr++;
            vector2.push_back((*itr)[0] - point[0]);
            vector2.push_back((*itr)[1] - point[1]);
            vector2.push_back((*itr)[2] - point[2]);
            itr--;
        }
        else
        {
            vector1.push_back((*itr)[0] - point[0]);
            vector1.push_back((*itr)[1] - point[1]);
            vector1.push_back((*itr)[2] - point[2]);
            itr = polygon.begin();
            vector2.push_back((*itr)[0] - point[0]);
            vector2.push_back((*itr)[1] - point[1]);
            vector2.push_back((*itr)[2] - point[2]);
            itr = polygon.end();
            itr--;
        }

        double length1 = sqrt(pow(vector1[0], 2) + pow(vector1[1], 2) + pow(vector1[2], 2)); 
        double length2 = sqrt(pow(vector2[0], 2) + pow(vector2[1], 2) + pow(vector2[2], 2));
        if (length1 * length2 == 0)
            return true;
        double dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
        angles.push_back(acos(dot_product / (length1 * length2)));
    }

    double angle_sum = 0.0;
    for(auto itr = angles.begin(); itr != angles.end(); itr++)
    {
        angle_sum += (*itr);
    }
    // std::cout << angle_sum << std::endl;
    // std::cout << angle_sum / M_PI << std::endl;
    double angle_sum_rad = angle_sum / M_PI;
    if(angle_sum_rad > 1.999 and angle_sum_rad < 2.001)  // allow tiny errors
        return true;
    else
        return false;
}

double benchmark::calculateArea(std::vector<std::vector<double>> triangle)
{
    std::vector<double> vector1, vector2; // Two sides of the triangle
    for (int i = 0; i < 3; i++)
    {
        vector1.push_back(triangle[1][i] - triangle[0][i]);
        vector2.push_back(triangle[2][i] - triangle[1][i]);
    }

    // Calculate cross product
    double z = vector1[0] * vector2[1] - vector1[1] * vector2[0];
    double y = -1 * vector1[0] * vector2[2] + vector1[2] * vector2[0];
    double x = vector1[1] * vector2[2] - vector1[2] * vector2[1];
    double triangle_area = sqrt(pow(x,2) + pow(y,2) + pow(z,2)) * 0.5; // Magnitude of the vector
    return triangle_area;
}

double benchmark::calculateArea(pcl::PolygonMesh mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud); // Extract point cloud from the input mesh

    size_t polygon_size = mesh.polygons.size();
    double mesh_area = 0;
    for (size_t i = 0; i < polygon_size; i++)
    {
        std::vector<std::vector<double>> triangle;
        size_t vertex_size = mesh.polygons[i].vertices.size();

        if (vertex_size != 3)
        {
            throw "Input mesh contains an invalid polygon!"; // Input must be a riangle mesh
        }

        for (size_t j = 0; j < vertex_size; j++)
        {
            triangle.push_back({
                mesh_cloud->points[mesh.polygons[i].vertices[j]].x, 
                mesh_cloud->points[mesh.polygons[i].vertices[j]].y,
                mesh_cloud->points[mesh.polygons[i].vertices[j]].z
            });
        }
        double triangle_area = calculateArea(triangle);
        mesh_area += triangle_area;
    }
    return mesh_area;
}

std::vector<double> benchmark::calculateCentralSymmetryPoint(std::vector<double> point, std::vector<double> center_point)
{
    std::vector<double> symmetry_point;
    auto center_itr = center_point.begin();
    for (auto itr = point.begin(); itr != point.end(); itr++)
    {
        symmetry_point.push_back((*center_itr) * 2 - (*itr));
        center_itr++;
    }
    return symmetry_point;
} 

std::vector<double> benchmark::calculatePerpendicularIntersection(std::vector<double> point1, std::vector<double> point2, std::vector<double> point3)
{
    // Calculate the direction vector
    double m = point2[0] - point1[0];
    double n = point2[1] - point1[1];
    double p = point2[2] - point1[2];

    // Calculate the coordinates
    double numerator = m * (point3[0] - point1[0]) + n * (point3[1] - point1[1]) + p * (point3[2] - point1[2]);
    double denominator = pow(m, 2) + pow(n, 2) + pow(p, 2);
    double t = numerator / denominator;
    std::vector<double> result_point;
    result_point.push_back(point1[0] + t * m);
    result_point.push_back(point1[1] + t * n);
    result_point.push_back(point1[2] + t * p);
    return result_point;
}

std::vector<double> benchmark::uniformTriangleSampling(std::vector<std::vector<double>> triangle)
{
    std::vector<double> point1;
    std::vector<double> point2;
    std::vector<double> point3;
    std::vector<double> point4;
    std::vector<double> point5;
    std::vector<double> point6;
    std::vector<double> sample;

    double length1 = sqrt
    (
        pow(triangle[1][0]-triangle[0][0], 2) +
        pow(triangle[1][1]-triangle[0][1], 2) +
        pow(triangle[1][2]-triangle[0][2], 2)
    );

    double length2 = sqrt
    (
        pow(triangle[2][0]-triangle[1][0], 2) +
        pow(triangle[2][1]-triangle[1][1], 2) +
        pow(triangle[2][2]-triangle[1][2], 2)
    );

    double length3 = sqrt
    (
        pow(triangle[0][0]-triangle[2][0], 2) +
        pow(triangle[0][1]-triangle[2][1], 2) +
        pow(triangle[0][2]-triangle[2][2], 2)
    );

    double horizontal_length = length1;
    point1 = triangle[0];
    point2 = triangle[1];
    point3 = triangle[2];

    if (length2 >= horizontal_length) 
    {
        horizontal_length = length2;
        point1 = triangle[1];
        point2 = triangle[2];
        point3 = triangle[0];
    }
    if (length3 >= horizontal_length) 
    {
        horizontal_length = length3;
        point1 = triangle[2];
        point2 = triangle[0];
        point3 = triangle[1];
    }

    point4 = calculatePerpendicularIntersection(point1, point2, point3);

    std::vector<double> center_point;
    center_point.push_back((point2[0] + point3[0]) * 0.5);
    center_point.push_back((point2[1] + point3[1]) * 0.5);
    center_point.push_back((point2[2] + point3[2]) * 0.5);
    point5 = calculateCentralSymmetryPoint(point4, center_point);
    center_point[0] = (point1[0] + point3[0]) * 0.5;
    center_point[1] = (point1[1] + point3[1]) * 0.5;
    center_point[2] = (point1[2] + point3[2]) * 0.5;
    point6 = calculateCentralSymmetryPoint(point4, center_point);
    return point6;
}

