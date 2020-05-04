#include "hull_abstraction/reconstructor.h"

pcl::PolygonMesh hull_abstraction::Reconstructor::greedyTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    double resolution = hull_abstraction::computeCloudResolutionN(cloud_with_normals);
    // Define search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);
    // Configurations of triangulation
    greedy_projection_triangulation.setSearchRadius(5 * resolution);  //Search radium, that is the radium of KNN
    greedy_projection_triangulation.setMu(5);  //in case that the cloud is not perfectly uniform
    greedy_projection_triangulation.setMaximumNearestNeighbors(200);    //The largest searched neighbours，typical value: 50-100
    greedy_projection_triangulation.setMinimumAngle(M_PI/18); // 10°
    greedy_projection_triangulation.setMaximumAngle(2*M_PI/3);  // 120°
    greedy_projection_triangulation.setMaximumSurfaceAngle(M_PI/4); // 45°
    greedy_projection_triangulation.setNormalConsistency(false);
    greedy_projection_triangulation.setInputCloud(cloud_with_normals); // input the cloud with normal
    greedy_projection_triangulation.setSearchMethod(tree);
    greedy_projection_triangulation.reconstruct(mesh);
    return mesh;
}

pcl::PolygonMesh hull_abstraction::Reconstructor::poissonReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    // Define search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);

    // Configurations
    poisson.setConfidence(false); //whether uses the magnitude of normals as confidence information. when false, all normals are normalised.
    poisson.setDegree(2); //degree[1,5), the larger the value, the longer it takes.
    poisson.setDepth(10);
    poisson.setIsoDivide(5); //still dont know
    poisson.setManifold(false); //whether add the center of gravity of polygons. 
    poisson.setOutputPolygons(true);
    poisson.setSamplesPerNode(20.0);
    poisson.setScale(1.0);
    poisson.setInputCloud(cloud_with_normals);
    poisson.performReconstruction(mesh);
    return mesh;
}

pcl::PolygonMesh hull_abstraction::Reconstructor::marchingCubesReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    double resolution = hull_abstraction::computeCloudResolutionN(cloud_with_normals);
    // Define search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);
    pcl::MarchingCubes<pcl::PointNormal>::Ptr marching_cubes(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
    marching_cubes->setIsoLevel(0.0f);
    marching_cubes->setGridResolution(30, 30, 30);
    marching_cubes->setPercentageExtendGrid(0.0f);
    marching_cubes->setSearchMethod(tree);
    marching_cubes->setInputCloud(cloud_with_normals);
    marching_cubes->reconstruct(mesh);
    return mesh;
}

pcl::PolygonMesh hull_abstraction::Reconstructor::bsplineSurfaceFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::on_nurbs::NurbsDataSurface surface_data;
    pointCloud2Vector3d(cloud, surface_data.interior);

    // parameters
    unsigned polynomial_order = 3;
    unsigned refinement = 5;
    unsigned iterations = 10;
    unsigned mesh_resolution = 256;

    pcl::on_nurbs::FittingSurface::Parameter surface_parameters;
    surface_parameters.interior_smoothness = 0.2;
    surface_parameters.interior_weight = 1.0;
    surface_parameters.boundary_smoothness = 0.2;
    surface_parameters.boundary_weight = 0.0;

    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_parameters;
    curve_parameters.addCPsAccuracy = 5e-2;
    curve_parameters.addCPsIteration = 3;
    curve_parameters.maxCPs = 200;
    curve_parameters.accuracy = 1e-3;
    curve_parameters.iterations = 100;

    curve_parameters.param.closest_point_resolution = 0;
    curve_parameters.param.closest_point_weight = 1.0;
    curve_parameters.param.closest_point_sigma2 = 0.1;
    curve_parameters.param.interior_sigma2 = 0.00001;
    curve_parameters.param.smooth_concavity = 1.0;
    curve_parameters.param.smoothness = 1.0;

    // initialize
    ON_NurbsSurface surface_nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(polynomial_order, &surface_data);
    pcl::on_nurbs::FittingSurface fitting_surface(&surface_data, surface_nurbs);

    // mesh for visualization
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fitting_surface.m_nurbs, mesh, mesh_resolution);

    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fitting_surface.refine(0);
        fitting_surface.refine(1);
        fitting_surface.assemble(surface_parameters);
        fitting_surface.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fitting_surface.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    }

    // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fitting_surface.assemble (surface_parameters);
        fitting_surface.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fitting_surface.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    }
    
    // B-spline curve fitting
    // initialisation (circular)
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = surface_data.interior_param;
    curve_data.interior_weight_function.push_back(true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(polynomial_order, curve_data.interior);

    // curve fitting
    pcl::on_nurbs::FittingCurve2dASDM fitting_curve(&curve_data, curve_nurbs);
    // fittingCurve.setQuiet (false); // enable/disable debug output
    fitting_curve.fitting(curve_parameters);

    // triangulation of trimmed surface
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fitting_surface.m_nurbs, fitting_curve.m_nurbs, mesh, mesh_resolution);
    return mesh;
}

void hull_abstraction::Reconstructor::pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZ &p = cloud->at(i);
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
        data.push_back(Eigen::Vector3d (p.x, p.y, p.z));
    }
}
