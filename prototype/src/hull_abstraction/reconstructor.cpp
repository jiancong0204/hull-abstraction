#include "include/hull_abstraction/reconstructor.h"

pcl::PolygonMesh hull_abstraction::Reconstructor::greedyTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals)
{
    double resolution = hull_abstraction::computeCloudResolutionN(cloudWithNormals);

    // Define search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloudWithNormals);

    // Configurations of triangulation
    greedyProjection.setSearchRadius(5 * resolution);  //Search radium, that is the radium of KNN
    greedyProjection.setMu(5);  //in case that the cloud is not perfectly uniform
    greedyProjection.setMaximumNearestNeighbors(200);    //The largest searched neighbours，typical value: 50-100
    greedyProjection.setMinimumAngle(M_PI/18); // 10°
    greedyProjection.setMaximumAngle(2*M_PI/3);  // 120°
    greedyProjection.setMaximumSurfaceAngle(M_PI/4); // 45°
    greedyProjection.setNormalConsistency(false);
    greedyProjection.setInputCloud(cloudWithNormals); // input the cloud with normal
    greedyProjection.setSearchMethod(tree);
    greedyProjection.reconstruct(mesh);
    return mesh;
}

pcl::PolygonMesh hull_abstraction::Reconstructor::poissonReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals)
{
    // Define search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloudWithNormals);

    // Configurations
    poisson.setConfidence(false); //whether uses the magnitude of normals as confidence information. when false, all normals are normalised.
    poisson.setDegree(2); //degree[1,5), the larger the value, the longer it takes.
    poisson.setDepth(5);
    poisson.setIsoDivide(5); //still dont know
    poisson.setManifold(false); //whether add the center of gravity of polygons. 
    poisson.setOutputPolygons(true);
    poisson.setSamplesPerNode(20.0);
    poisson.setScale(1.25);
    poisson.setSolverDivide(8);
    //poisson.setIndices();
    poisson.setSearchMethod(tree);
    poisson.setInputCloud(cloudWithNormals);
    poisson.performReconstruction(mesh);
    return mesh;
}

pcl::PolygonMesh hull_abstraction::Reconstructor::marchingCubesReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals)
{
    double resolution = hull_abstraction::computeCloudResolutionN(cloudWithNormals);
    // Define search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloudWithNormals);

    pcl::MarchingCubes<pcl::PointNormal>::Ptr marchingCubes(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
    marchingCubes->setIsoLevel(0.0f);
    marchingCubes->setGridResolution(30, 30, 30);
    marchingCubes->setPercentageExtendGrid(0.0f);
    marchingCubes->setSearchMethod(tree);
    marchingCubes->setInputCloud(cloudWithNormals);
    marchingCubes->reconstruct(mesh);
    return mesh;
}

pcl::PolygonMesh hull_abstraction::Reconstructor::bsplineSurfaceFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::on_nurbs::NurbsDataSurface surfaceData;
    pointCloud2Vector3d (cloud, surfaceData.interior);

    // parameters
    unsigned polynomialOrder = 3;
    unsigned refinement = 5;
    unsigned iterations = 10;
    unsigned meshResolution = 256;

    pcl::on_nurbs::FittingSurface::Parameter surfaceParameters;
    surfaceParameters.interior_smoothness = 0.2;
    surfaceParameters.interior_weight = 1.0;
    surfaceParameters.boundary_smoothness = 0.2;
    surfaceParameters.boundary_weight = 0.0;

    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curveParameters;
    curveParameters.addCPsAccuracy = 5e-2;
    curveParameters.addCPsIteration = 3;
    curveParameters.maxCPs = 200;
    curveParameters.accuracy = 1e-3;
    curveParameters.iterations = 100;

    curveParameters.param.closest_point_resolution = 0;
    curveParameters.param.closest_point_weight = 1.0;
    curveParameters.param.closest_point_sigma2 = 0.1;
    curveParameters.param.interior_sigma2 = 0.00001;
    curveParameters.param.smooth_concavity = 1.0;
    curveParameters.param.smoothness = 1.0;

    // initialize
    ON_NurbsSurface surfaceNurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (polynomialOrder, &surfaceData);
    pcl::on_nurbs::FittingSurface fittingSurface (&surfaceData, surfaceNurbs);

    // mesh for visualization
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> meshVertices;
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fittingSurface.m_nurbs, mesh, meshResolution);

    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fittingSurface.refine (0);
        fittingSurface.refine (1);
        fittingSurface.assemble (surfaceParameters);
        fittingSurface.solve ();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices (fittingSurface.m_nurbs, meshCloud, meshVertices, meshResolution);

    }

    // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fittingSurface.assemble (surfaceParameters);
        fittingSurface.solve ();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices (fittingSurface.m_nurbs, meshCloud, meshVertices, meshResolution);
    }

    // B-spline curve fitting
    // initialisation (circular)
    pcl::on_nurbs::NurbsDataCurve2d curveData;
    curveData.interior = surfaceData.interior_param;
    curveData.interior_weight_function.push_back (true);
    ON_NurbsCurve curveNurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (polynomialOrder, curveData.interior);

    // curve fitting
    pcl::on_nurbs::FittingCurve2dASDM fittingCurve (&curveData, curveNurbs);
    // fittingCurve.setQuiet (false); // enable/disable debug output
    fittingCurve.fitting (curveParameters);

    // triangulation of trimmed surface
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fittingSurface.m_nurbs, fittingCurve.m_nurbs, mesh, meshResolution);
    return mesh;
}

void hull_abstraction::Reconstructor::pointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size (); i++)
  {
      pcl::PointXYZ &p = cloud->at (i);
      if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  }
}
